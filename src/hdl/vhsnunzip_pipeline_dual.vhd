library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.vhsnunzip_utils_pkg.all;
use work.vhsnunzip_int_pkg.all;

-- Speculative dual-issue Snappy decompression pipeline (contained 16 B/cycle
-- "fold"). It is a drop-in alternative to vhsnunzip_pipeline (same ports minus
-- the sim-only debug taps), instantiated by vhsnunzip_unbuffered when
-- SPEC_OFFSETS > 0. The single-issue pipeline.vhd is left untouched.
--
-- The front end (dual decoder + dual command generators) emits up to two
-- datapath commands per cycle as a dual_command_stream {cm0, cm1}. The fold
-- datapath retires both in one cycle into the SAME one-line output bus and the
-- SAME single long-term read/write port:
--
--  * cmd_gen_2_dual co-issues cm1 when the pair writes <= C_BYTES bytes (so at
--    most one line completes and each short-term lane is pushed at most once --
--    the cm0/cm1 lane ranges are provably disjoint). cm1 may read long-term:
--    the history URAM is mirrored into a second read bank (vhsnunzip_unbuffered),
--    so cm0 and cm1 have independent long-term read ports. So the datapath
--    ALWAYS folds when cm1 is valid; there is no runtime defer.
--  * The same-cycle RAW hazard (cm1's copy reading bytes cm0 produces this
--    cycle) is resolved by a per-lane forward overlay on cm1's short-term read:
--    read the (clocked, un-pushed) short-term bank at index - pushed0, and
--    substitute cm0's just-computed output byte when the post-push index
--    resolves to 0. This overlay was proven byte-exact against the SRL-faithful
--    Python reference (datapath_fold).
--
-- The doubling is confined to s1 (per-byte control for cm0 then cm1, threading
-- the hold-valid and literal-FIFO-level state through both) and s2 (two
-- rotators + the forward overlay + a per-lane merge). The s2->s3 holding /
-- output / last-line logic is byte-identical to vhsnunzip_pipeline: the merged
-- strobes (ext = ext0 or (ext1 and not cm0-crosses); int = int0 or int1) feed
-- it unchanged, and a fold is never the chunk's last command, so the trailing-
-- line / last-pending handling only ever runs in the cm0-only (cm1 invalid)
-- case, which reduces exactly to single-issue.
entity vhsnunzip_pipeline_dual is
  generic (
    LONG_CHUNKS : boolean := true;

    -- Number of speculative element-1 start offsets in the dual decoder. See
    -- C_SPEC_OFFSETS in vhsnunzip_utils_pkg.
    SPEC_OFFSETS : natural := C_SPEC_OFFSETS
  );
  port (
    clk         : in  std_logic;
    reset       : in  std_logic;

    -- Compressed data input stream.
    co          : in  compressed_stream_single;
    co_ready    : out std_logic;
    co_level    : out unsigned(5 downto 0);

    -- Long-term storage first line offset.
    lt_off_ld   : in  std_logic := '1';
    lt_off      : in  unsigned(C_AW downto 0) := (others => '0');

    -- Long-term storage read request/response for cm0 (the lead command).
    lt_rd_valid : out std_logic;
    lt_rd_ready : in  std_logic := '1';
    lt_rd_adev  : out unsigned(C_AW-1 downto 0);
    lt_rd_adod  : out unsigned(C_AW-1 downto 0);

    lt_rd_next  : in  std_logic;
    lt_rd_even  : in  byte_array(0 to C_BYTES-1);
    lt_rd_odd   : in  byte_array(0 to C_BYTES-1);

    -- Second long-term read request/response for cm1 (the co-issued command),
    -- backed by a mirror of the history URAM written identically. This lets a
    -- folded pair both read long-term in the same cycle.
    lt_rd_valid1 : out std_logic;
    lt_rd_ready1 : in  std_logic := '1';
    lt_rd_adev1  : out unsigned(C_AW-1 downto 0);
    lt_rd_adod1  : out unsigned(C_AW-1 downto 0);

    lt_rd_next1  : in  std_logic := '0';
    lt_rd_even1  : in  byte_array(0 to C_BYTES-1) := (others => (others => '0'));
    lt_rd_odd1   : in  byte_array(0 to C_BYTES-1) := (others => (others => '0'));

    -- Decompressed data output stream.
    de          : out decompressed_stream;
    de_ready    : in  std_logic;
    de_level    : out unsigned(5 downto 0)
  );
end vhsnunzip_pipeline_dual;

architecture behavior of vhsnunzip_pipeline_dual is

  -- Command-FIFO control word layout for one command (same as pipeline.vhd).
  constant ST_W      : natural := 5;
  constant ROL_W     : natural := C_WIN;
  constant END_W     : natural := C_CNT;
  constant O_LT_VAL  : natural := 0;
  constant O_LT_SWAP : natural := O_LT_VAL + 1;
  constant O_ST_ADDR : natural := O_LT_SWAP + 1;
  constant O_CP_ROL  : natural := O_ST_ADDR + ST_W;
  constant O_CP_RLE  : natural := O_CP_ROL + ROL_W;
  constant O_CP_END  : natural := O_CP_RLE + 1;
  constant O_LI_ROL  : natural := O_CP_END + END_W;
  constant O_LI_END  : natural := O_LI_ROL + ROL_W;
  constant O_LD_POP  : natural := O_LI_END + END_W;
  constant O_LAST    : natural := O_LD_POP + 1;
  constant CM_CTRL_W : natural := O_LAST + 1;

  -- Dual FIFO control word: cm0 word, cm1 word, and a cm1-valid bit.
  constant DUAL_CTRL_W : natural := 2*CM_CTRL_W + 1;

  -- Pack/unpack a command_stream to/from a control word (lt_adev/adod are not
  -- carried: the long-term read is issued from cm0 at the FIFO write side, just
  -- like single-issue).
  function cm_pack(cm : command_stream) return std_logic_vector is
    variable v : std_logic_vector(CM_CTRL_W-1 downto 0);
  begin
    v := (others => '0');
    v(O_LT_VAL)                         := cm.lt_val;
    v(O_LT_SWAP)                        := cm.lt_swap;
    v(O_ST_ADDR+ST_W-1 downto O_ST_ADDR):= std_logic_vector(cm.st_addr);
    v(O_CP_ROL+ROL_W-1 downto O_CP_ROL) := std_logic_vector(cm.cp_rol);
    v(O_CP_RLE)                         := cm.cp_rle;
    v(O_CP_END+END_W-1 downto O_CP_END) := std_logic_vector(cm.cp_end);
    v(O_LI_ROL+ROL_W-1 downto O_LI_ROL) := std_logic_vector(cm.li_rol);
    v(O_LI_END+END_W-1 downto O_LI_END) := std_logic_vector(cm.li_end);
    v(O_LD_POP)                         := cm.ld_pop;
    v(O_LAST)                           := cm.last;
    return v;
  end function;

  function cm_unpack(v : std_logic_vector(CM_CTRL_W-1 downto 0)) return command_stream is
    variable cm : command_stream := COMMAND_STREAM_INIT;
  begin
    cm.lt_val  := v(O_LT_VAL);
    cm.lt_swap := v(O_LT_SWAP);
    cm.st_addr := unsigned(v(O_ST_ADDR+ST_W-1 downto O_ST_ADDR));
    cm.cp_rol  := unsigned(v(O_CP_ROL+ROL_W-1 downto O_CP_ROL));
    cm.cp_rle  := v(O_CP_RLE);
    cm.cp_end  := unsigned(v(O_CP_END+END_W-1 downto O_CP_END));
    cm.li_rol  := unsigned(v(O_LI_ROL+ROL_W-1 downto O_LI_ROL));
    cm.li_end  := unsigned(v(O_LI_END+END_W-1 downto O_LI_END));
    cm.ld_pop  := v(O_LD_POP);
    cm.last    := v(O_LAST);
    return cm;
  end function;

  -- Compressed data FIFO signals.
  signal co_ctrl      : std_logic_vector(C_IDX downto 0);
  signal cs_ctrl      : std_logic_vector(C_IDX downto 0);

  signal cs           : compressed_stream_single;
  signal cs_ready     : std_logic;
  signal cs_strobe    : std_logic;

  signal cd           : compressed_stream_double;
  signal cd_ready     : std_logic;

  -- Dual element / command generator streams.
  signal el           : dual_element_stream;
  signal el_ready     : std_logic;
  signal c1           : dual_partial_command_stream;
  signal c1_ready     : std_logic;
  signal cm           : dual_command_stream;
  signal cm_ready     : std_logic;

  -- Dual command FIFO.
  signal cm_push      : std_logic;
  signal cm_ctrl      : std_logic_vector(DUAL_CTRL_W-1 downto 0);
  signal s1_ctrl      : std_logic_vector(DUAL_CTRL_W-1 downto 0);
  signal s1_rd_valid  : std_logic;
  signal s1_cm0       : command_stream;
  signal s1_cm1       : command_stream;

  signal s1_valid     : std_logic;
  signal s2_valid     : std_logic;

  type srl_addr_array is array (natural range <>) of unsigned(4 downto 0);
  type rol_array is array (natural range <>) of unsigned(C_IDX-1 downto 0);

  -- Literal SRL (two read banks: cm0 and cm1).
  signal s2_li_addr0  : srl_addr_array(0 to C_BYTES-1);
  signal s2_li_data0  : byte_array(0 to C_BYTES-1);
  signal s2_li_addr1  : srl_addr_array(0 to C_BYTES-1);
  signal s2_li_data1  : byte_array(0 to C_BYTES-1);

  -- Short-term SRL (two read banks, written identically).
  signal s2_st_addr0  : srl_addr_array(0 to C_BYTES-1);
  signal s2_st_data0  : byte_array(0 to C_BYTES-1);
  signal s2_st_addr1  : srl_addr_array(0 to C_BYTES-1);
  signal s2_st_dataB  : byte_array(0 to C_BYTES-1);
  signal s2_st_data1  : byte_array(0 to C_BYTES-1);

  -- Long-term memory data (cm0 read port, and the mirrored cm1 read port).
  signal s2_le_data   : byte_array(0 to C_BYTES-1);
  signal s2_lo_data   : byte_array(0 to C_BYTES-1);
  signal s2_le_data1  : byte_array(0 to C_BYTES-1);
  signal s2_lo_data1  : byte_array(0 to C_BYTES-1);

  -- cm0 copy-source mux and rotator.
  signal s2_lt_val    : std_logic;
  signal s2_lt_sel0   : std_logic_array(0 to C_BYTES-1);
  signal s2_cp_data0  : byte_array(0 to C_BYTES-1);
  signal s2_rol_sel0  : rol_array(0 to C_BYTES-1);
  signal s2_mux_sel0  : std_logic_array(0 to C_BYTES-1);
  signal s2_mux_data0 : byte_array(0 to C_BYTES-1);

  -- cm1 copy-source (long-term read, or short-term + forward overlay) and rotator.
  signal s2_lt_val1   : std_logic;
  signal s2_lt_sel1   : std_logic_array(0 to C_BYTES-1);
  signal s2_fwd1      : std_logic_array(0 to C_BYTES-1);
  signal s2_cp_data1  : byte_array(0 to C_BYTES-1);
  signal s2_rol_sel1  : rol_array(0 to C_BYTES-1);
  signal s2_mux_sel1  : std_logic_array(0 to C_BYTES-1);
  signal s2_mux_data1 : byte_array(0 to C_BYTES-1);

  -- Per-lane "cm1 wrote this lane" select and merged mux data.
  signal s2_cm1_lane  : std_logic_array(0 to C_BYTES-1);
  signal s2_mux_data  : byte_array(0 to C_BYTES-1);

  -- Merged byte strobes (feed the unchanged s2->s3 logic).
  signal s2_int_strb  : std_logic_array(0 to C_BYTES-1);
  signal s2_ext_strb  : std_logic_array(0 to C_BYTES-1);

  signal s2_last      : std_logic;
  signal s2_cnt       : unsigned(C_IDX-1 downto 0);
  signal s3_cnt       : unsigned(C_IDX-1 downto 0);

  signal s3_hold_data : byte_array(0 to C_BYTES-1);
  signal s3_out_push  : std_logic;
  signal s3_out_data  : byte_array(0 to C_BYTES-1);
  signal s3_out_last  : std_logic;
  signal s3_out_cnt   : unsigned(C_CNT-1 downto 0);
  signal s3_last_pend : std_logic;

  signal s3_out_ctrl  : std_logic_vector(C_CNT downto 0);
  signal de_ctrl      : std_logic_vector(C_CNT downto 0);
  signal de_level_s   : unsigned(5 downto 0);
  signal backpres     : std_logic;

begin

  -- Compressed data input FIFO.
  co_ctrl(0) <= co.last;
  co_ctrl(C_IDX downto 1) <= std_logic_vector(co.endi);

  co_fifo_inst: vhsnunzip_fifo
    generic map (
      DATA_WIDTH  => C_BYTES,
      CTRL_WIDTH  => C_IDX+1
    )
    port map (
      clk         => clk,
      reset       => reset,
      wr_valid    => co.valid,
      wr_ready    => co_ready,
      wr_data     => co.data,
      wr_ctrl     => co_ctrl,
      rd_valid    => cs.valid,
      rd_ready    => cs_ready,
      rd_data     => cs.data,
      rd_ctrl     => cs_ctrl,
      level       => co_level
    );

  cs.last <= cs_ctrl(0);
  cs.endi <= unsigned(cs_ctrl(C_IDX downto 1));

  -- Literal data SRLs. Two banks per byte, written identically by the input
  -- stream, read independently by cm0 (bank A) and cm1 (bank B).
  cs_strobe <= cs.valid and cs_ready;

  ld_srl_gen: for byte in 0 to C_BYTES-1 generate
  begin
    srl_a_inst: vhsnunzip_srl
      generic map (WIDTH => 8, DEPTH_LOG2 => 5)
      port map (
        clk => clk, wr_ena => cs_strobe, wr_data => cs.data(byte),
        rd_addr => s2_li_addr0(byte), rd_data => s2_li_data0(byte));
    srl_b_inst: vhsnunzip_srl
      generic map (WIDTH => 8, DEPTH_LOG2 => 5)
      port map (
        clk => clk, wr_ena => cs_strobe, wr_data => cs.data(byte),
        rd_addr => s2_li_addr1(byte), rd_data => s2_li_data1(byte));
  end generate;

  -- Pre-decoder.
  pre_dec_inst: vhsnunzip_pre_decoder
    generic map (LONG_CHUNKS => LONG_CHUNKS)
    port map (
      clk => clk, reset => reset,
      cs => cs, cs_ready => cs_ready,
      cd => cd, cd_ready => cd_ready);

  -- Speculative dual-issue decoder.
  main_dec_inst: vhsnunzip_decoder_dual
    generic map (
      SPEC_OFFSETS => SPEC_OFFSETS)
    port map (
      clk => clk, reset => reset,
      cd => cd, cd_ready => cd_ready,
      el => el, el_ready => el_ready);

  -- Dual command generator stage 1.
  cmd_gen_1_inst: vhsnunzip_cmd_gen_1_dual
    port map (
      clk => clk, reset => reset,
      el => el, el_ready => el_ready,
      c1 => c1, c1_ready => c1_ready);

  -- Dual command generator stage 2.
  cmd_gen_2_inst: vhsnunzip_cmd_gen_2_dual
    generic map (LONG_CHUNKS => LONG_CHUNKS)
    port map (
      clk => clk, reset => reset,
      c1 => c1, c1_ready => c1_ready,
      lt_off_ld => lt_off_ld, lt_off => lt_off,
      cm => cm, cm_ready => cm_ready);

  -- Backpressure / handshake. Both cm0 and cm1 may read long-term (cm1 via the
  -- mirror read port), so both read ports must be ready.
  cm_ready <= not backpres and (lt_rd_ready or not cm.cm0.lt_val)
                           and (lt_rd_ready1 or not (cm.cm1.valid and cm.cm1.lt_val));
  cm_push <= cm.cm0.valid and cm_ready;

  -- Issue the long-term read commands: cm0 on the primary port, cm1 on the
  -- mirror port. Both are issued from the same cm transfer, so their results
  -- return in lockstep (identical RAM latency).
  lt_rd_valid <= cm.cm0.lt_val and cm.cm0.valid and not backpres;
  lt_rd_adev <= cm.cm0.lt_adev;
  lt_rd_adod <= cm.cm0.lt_adod;

  lt_rd_valid1 <= cm.cm1.lt_val and cm.cm1.valid and not backpres;
  lt_rd_adev1 <= cm.cm1.lt_adev;
  lt_rd_adod1 <= cm.cm1.lt_adod;

  -- Dual command FIFO: pack cm0, cm1 and cm1's valid bit.
  cm_ctrl(CM_CTRL_W-1 downto 0)            <= cm_pack(cm.cm0);
  cm_ctrl(2*CM_CTRL_W-1 downto CM_CTRL_W)  <= cm_pack(cm.cm1);
  cm_ctrl(DUAL_CTRL_W-1)                   <= cm.cm1.valid;

  cm_fifo_inst: vhsnunzip_fifo
    generic map (CTRL_WIDTH => DUAL_CTRL_W)
    port map (
      clk => clk, reset => reset,
      wr_valid => cm_push, wr_ctrl => cm_ctrl,
      rd_valid => s1_rd_valid, rd_ready => s1_valid,
      rd_ctrl => s1_ctrl);

  s1_unpack_proc: process (s1_ctrl, s1_rd_valid) is
  begin
    s1_cm0 <= cm_unpack(s1_ctrl(CM_CTRL_W-1 downto 0));
    s1_cm0.valid <= s1_rd_valid;
    s1_cm1 <= cm_unpack(s1_ctrl(2*CM_CTRL_W-1 downto CM_CTRL_W));
    s1_cm1.valid <= s1_rd_valid and s1_ctrl(DUAL_CTRL_W-1);
  end process;

  -- All stage-0 sources ready (command + long-term result for cm0 and, when it
  -- reads long-term, cm1); insert a stall cycle after the last command so the
  -- holding register can be flushed.
  s1_valid <= s1_cm0.valid and (lt_rd_next or not s1_cm0.lt_val)
                           and (lt_rd_next1 or not (s1_cm1.valid and s1_cm1.lt_val))
                           and not s2_last;

  -- Stage 1 logic + stage 1-2 registers: per-byte control for cm0 then cm1,
  -- threading hold_valid and the literal FIFO level through both.
  s1_reg_proc: process (clk) is

    variable hold_valid   : std_logic_array(0 to C_BYTES-1) := (others => '0');
    variable cp_end_th    : std_logic_array(0 to 2*C_BYTES-1);
    variable li_end_th    : std_logic_array(0 to 2*C_BYTES-1);

    variable shift        : unsigned(C_IDX-1 downto 0);
    type lookahead_lookup_type is array (natural range <>) of std_logic_array(0 to 2*C_BYTES*C_BYTES-1);
    function lookahead_lookup_fn return lookahead_lookup_type is
      variable ret  : lookahead_lookup_type(0 to C_BYTES-1);
      variable acc  : unsigned(C_WIN-1 downto 0);
    begin
      for byte in 0 to C_BYTES-1 loop
        for shif in 0 to C_BYTES-1 loop
          for rot in 0 to 2*C_BYTES-1 loop
            acc := to_unsigned(byte, C_WIN) - rot - shif;
            ret(byte)(shif * (2*C_BYTES) + rot) := acc(C_IDX);
          end loop;
        end loop;
      end loop;
      return ret;
    end function;
    constant LOOKAHEAD_LOOKUP : lookahead_lookup_type := lookahead_lookup_fn;
    variable li_ahead     : std_logic;
    variable cp_ahead     : std_logic;

    variable li_level     : unsigned(4 downto 0) := (others => '1');
    variable st_addr      : unsigned(4 downto 0);
    variable idx1         : unsigned(4 downto 0);

    -- cm0 strobes (kept so cm1 can use cm0's pushes for the forward overlay).
    variable i_strb0      : std_logic_array(0 to C_BYTES-1);
    variable e_strb0      : std_logic_array(0 to C_BYTES-1);
    variable i_strb1      : std_logic_array(0 to C_BYTES-1);
    variable e_strb1      : std_logic_array(0 to C_BYTES-1);
    variable cross0       : std_logic;
    variable c1v          : std_logic;

  begin
    if rising_edge(clk) then

      s2_valid <= s1_valid;
      s2_lt_val <= s1_cm0.lt_val;
      s2_lt_val1 <= s1_cm1.lt_val;
      s2_last <= s1_cm0.last and s1_valid;
      s2_cnt <= s1_cm0.li_end(C_IDX-1 downto 0);

      -- Literal FIFO push happens before the read address is used.
      if cs_strobe = '1' then
        li_level := li_level + 1;
      end if;

      c1v := s1_valid and s1_cm1.valid;
      cross0 := s1_cm0.li_end(C_IDX);

      hold_valid(C_BYTES-1) := '0';

      -- ================= command 0 =================
      cp_end_th(2*C_BYTES-1) := '0';
      li_end_th(2*C_BYTES-1) := '0';
      for byte in 0 to 2*C_BYTES-2 loop
        if byte < s1_cm0.cp_end then
          cp_end_th(byte) := s1_valid;
        else
          cp_end_th(byte) := '0';
        end if;
        if byte < s1_cm0.li_end then
          li_end_th(byte) := s1_valid;
        else
          li_end_th(byte) := '0';
        end if;
      end loop;

      if s1_cm0.li_end(C_IDX) = '1' then
        shift := s1_cm0.li_end(C_IDX-1 downto 0);
      else
        shift := (others => '0');
      end if;

      for byte in 0 to C_BYTES-1 loop
        if (cp_end_th(byte) = '1' and li_end_th(byte + C_BYTES) = '0') or cp_end_th(byte + C_BYTES) = '1' then
          s2_mux_sel0(byte) <= '1';
          if s1_cm0.cp_rle = '1' then
            s2_rol_sel0(byte) <= s1_cm0.cp_rol(C_IDX-1 downto 0) - byte;
          else
            s2_rol_sel0(byte) <= s1_cm0.cp_rol(C_IDX-1 downto 0);
          end if;
        else
          s2_mux_sel0(byte) <= '0';
          s2_rol_sel0(byte) <= s1_cm0.li_rol(C_IDX-1 downto 0);
        end if;

        li_ahead := LOOKAHEAD_LOOKUP(byte)(to_integer(shift & s1_cm0.li_rol));
        cp_ahead := LOOKAHEAD_LOOKUP(byte)(to_integer(shift & s1_cm0.cp_rol));
        if s1_cm0.cp_rle = '1' then
          cp_ahead := '0';
        end if;

        s2_lt_sel0(byte) <= s1_cm0.lt_swap xor cp_ahead;

        st_addr := s1_cm0.st_addr;
        if hold_valid(byte) = '1' then
          st_addr := st_addr + 1;
        end if;
        if cp_ahead = '1' then
          st_addr := st_addr - 1;
        end if;
        s2_st_addr0(byte) <= st_addr;

        if li_ahead = '1' then
          s2_li_addr0(byte) <= li_level - 1;
        else
          s2_li_addr0(byte) <= li_level;
        end if;

        e_strb0(byte) := li_end_th(byte) and not hold_valid(byte);
        i_strb0(byte) := (li_end_th(byte) and not hold_valid(byte)) or li_end_th(byte + C_BYTES);
        hold_valid(byte) := ((hold_valid(byte) or li_end_th(byte)) and not li_end_th(C_BYTES-1)) or li_end_th(byte + C_BYTES);
      end loop;

      if s1_valid = '1' and s1_cm0.ld_pop = '1' then
        li_level := li_level - 1;
      end if;

      -- ================= command 1 (folded) =================
      -- hold_valid now carries the post-command-0 state.
      cp_end_th(2*C_BYTES-1) := '0';
      li_end_th(2*C_BYTES-1) := '0';
      for byte in 0 to 2*C_BYTES-2 loop
        if byte < s1_cm1.cp_end then
          cp_end_th(byte) := c1v;
        else
          cp_end_th(byte) := '0';
        end if;
        if byte < s1_cm1.li_end then
          li_end_th(byte) := c1v;
        else
          li_end_th(byte) := '0';
        end if;
      end loop;

      if s1_cm1.li_end(C_IDX) = '1' then
        shift := s1_cm1.li_end(C_IDX-1 downto 0);
      else
        shift := (others => '0');
      end if;

      for byte in 0 to C_BYTES-1 loop
        if (cp_end_th(byte) = '1' and li_end_th(byte + C_BYTES) = '0') or cp_end_th(byte + C_BYTES) = '1' then
          s2_mux_sel1(byte) <= '1';
          if s1_cm1.cp_rle = '1' then
            s2_rol_sel1(byte) <= s1_cm1.cp_rol(C_IDX-1 downto 0) - byte;
          else
            s2_rol_sel1(byte) <= s1_cm1.cp_rol(C_IDX-1 downto 0);
          end if;
        else
          s2_mux_sel1(byte) <= '0';
          s2_rol_sel1(byte) <= s1_cm1.li_rol(C_IDX-1 downto 0);
        end if;

        li_ahead := LOOKAHEAD_LOOKUP(byte)(to_integer(shift & s1_cm1.li_rol));
        cp_ahead := LOOKAHEAD_LOOKUP(byte)(to_integer(shift & s1_cm1.cp_rol));
        if s1_cm1.cp_rle = '1' then
          cp_ahead := '0';
        end if;

        -- Long-term even/odd select for cm1 (mirrors cm0's s2_lt_sel0); used by
        -- the cm1 copy-source mux when cm1 reads long-term.
        s2_lt_sel1(byte) <= s1_cm1.lt_swap xor cp_ahead;

        -- Short-term copy source (used when cm1 does not read long-term): idx1 is
        -- the post-command-0 intended index (using the post-command-0
        -- hold_valid); the clocked bank B is read at idx1 - pushed0, and command
        -- 0's just-computed byte is forwarded when idx1 resolves to 0 and command
        -- 0 pushed this lane.
        idx1 := s1_cm1.st_addr;
        if hold_valid(byte) = '1' then
          idx1 := idx1 + 1;
        end if;
        if cp_ahead = '1' then
          idx1 := idx1 - 1;
        end if;

        if i_strb0(byte) = '1' and idx1 = 0 then
          s2_fwd1(byte) <= '1';
        else
          s2_fwd1(byte) <= '0';
        end if;

        if i_strb0(byte) = '1' then
          s2_st_addr1(byte) <= idx1 - 1;
        else
          s2_st_addr1(byte) <= idx1;
        end if;

        if li_ahead = '1' then
          s2_li_addr1(byte) <= li_level - 1;
        else
          s2_li_addr1(byte) <= li_level;
        end if;

        e_strb1(byte) := li_end_th(byte) and not hold_valid(byte);
        i_strb1(byte) := (li_end_th(byte) and not hold_valid(byte)) or li_end_th(byte + C_BYTES);
        hold_valid(byte) := ((hold_valid(byte) or li_end_th(byte)) and not li_end_th(C_BYTES-1)) or li_end_th(byte + C_BYTES);
      end loop;

      if c1v = '1' and s1_cm1.ld_pop = '1' then
        li_level := li_level - 1;
      end if;

      -- Merge the strobes for the shared s2->s3 logic. The cm0/cm1 written
      -- lanes are disjoint; ext (output-line) excludes cm1 when command 0
      -- crossed the line boundary (cm1 then belongs to the next line).
      for byte in 0 to C_BYTES-1 loop
        s2_int_strb(byte) <= i_strb0(byte) or i_strb1(byte);
        s2_ext_strb(byte) <= e_strb0(byte) or (e_strb1(byte) and not cross0);
        s2_cm1_lane(byte) <= i_strb1(byte);
      end loop;

      -- The last command is never folded, so this only ever fires cm0-only.
      if s1_valid = '1' and s1_cm0.last = '1' then
        hold_valid := (others => '0');
      end if;

      if reset = '1' then
        s2_valid <= '0';
        hold_valid := (others => '0');
        li_level := (others => '1');
      end if;
    end if;
  end process;

  -- Short-term memory SRLs (two read banks, written identically with the merged
  -- push so each bank holds the same history).
  st_srl_gen: for byte in 0 to C_BYTES-1 generate
  begin
    srl_a_inst: vhsnunzip_srl
      generic map (WIDTH => 8, DEPTH_LOG2 => 5)
      port map (
        clk => clk, wr_ena => s2_int_strb(byte), wr_data => s2_mux_data(byte),
        rd_addr => s2_st_addr0(byte), rd_data => s2_st_data0(byte));
    srl_b_inst: vhsnunzip_srl
      generic map (WIDTH => 8, DEPTH_LOG2 => 5)
      port map (
        clk => clk, wr_ena => s2_int_strb(byte), wr_data => s2_mux_data(byte),
        rd_addr => s2_st_addr1(byte), rd_data => s2_st_dataB(byte));
  end generate;

  -- "Load" long-term memory data: cm0 from the primary read port, cm1 from the
  -- mirror read port.
  s2_le_data <= lt_rd_even;
  s2_lo_data <= lt_rd_odd;
  s2_le_data1 <= lt_rd_even1;
  s2_lo_data1 <= lt_rd_odd1;

  -- cm0 copy-source multiplexer (short-term / long-term even / odd).
  s2_cp_data0_proc: process (
    s2_st_data0, s2_le_data, s2_lo_data, s2_lt_val, s2_lt_sel0
  ) is
  begin
    for byte in 0 to C_BYTES-1 loop
      if s2_lt_val = '0' then
        s2_cp_data0(byte) <= s2_st_data0(byte);
      elsif s2_lt_sel0(byte) = '0' then
        s2_cp_data0(byte) <= s2_le_data(byte);
      else
        s2_cp_data0(byte) <= s2_lo_data(byte);
      end if;
    end loop;
  end process;

  -- cm0 main multiplexer/rotator.
  s2_mux_data0_proc: process (
    s2_li_data0, s2_cp_data0, s2_rol_sel0, s2_mux_sel0
  ) is
    variable idx : unsigned(C_IDX-1 downto 0);
  begin
    for byte in 0 to C_BYTES-1 loop
      idx := s2_rol_sel0(byte) + byte;
      if s2_mux_sel0(byte) = '0' then
        s2_mux_data0(byte) <= s2_li_data0(to_integer(idx));
      else
        s2_mux_data0(byte) <= s2_cp_data0(to_integer(idx));
      end if;
    end loop;
  end process;

  -- cm1 short-term copy source: the clocked bank-B read, with command 0's
  -- same-cycle output forwarded per lane (the RAW-hazard resolution).
  s2_st_data1_proc: process (s2_fwd1, s2_mux_data0, s2_st_dataB) is
  begin
    for byte in 0 to C_BYTES-1 loop
      if s2_fwd1(byte) = '1' then
        s2_st_data1(byte) <= s2_mux_data0(byte);
      else
        s2_st_data1(byte) <= s2_st_dataB(byte);
      end if;
    end loop;
  end process;

  -- cm1 copy-source multiplexer: long-term even/odd (from the mirror read port)
  -- when cm1 reads long-term, otherwise the short-term + forward overlay. A
  -- long-term cm1 reads committed history, so there is no same-cycle RAW hazard
  -- with cm0 and the forward overlay does not apply on those lanes.
  s2_cp_data1_proc: process (
    s2_lt_val1, s2_lt_sel1, s2_le_data1, s2_lo_data1, s2_st_data1
  ) is
  begin
    for byte in 0 to C_BYTES-1 loop
      if s2_lt_val1 = '0' then
        s2_cp_data1(byte) <= s2_st_data1(byte);
      elsif s2_lt_sel1(byte) = '0' then
        s2_cp_data1(byte) <= s2_le_data1(byte);
      else
        s2_cp_data1(byte) <= s2_lo_data1(byte);
      end if;
    end loop;
  end process;

  -- cm1 main multiplexer/rotator.
  s2_mux_data1_proc: process (
    s2_li_data1, s2_cp_data1, s2_rol_sel1, s2_mux_sel1
  ) is
    variable idx : unsigned(C_IDX-1 downto 0);
  begin
    for byte in 0 to C_BYTES-1 loop
      idx := s2_rol_sel1(byte) + byte;
      if s2_mux_sel1(byte) = '0' then
        s2_mux_data1(byte) <= s2_li_data1(to_integer(idx));
      else
        s2_mux_data1(byte) <= s2_cp_data1(to_integer(idx));
      end if;
    end loop;
  end process;

  -- Per-lane merge: cm1's lanes take its rotator output, the rest take cm0's.
  s2_mux_merge_proc: process (s2_cm1_lane, s2_mux_data0, s2_mux_data1) is
  begin
    for byte in 0 to C_BYTES-1 loop
      if s2_cm1_lane(byte) = '1' then
        s2_mux_data(byte) <= s2_mux_data1(byte);
      else
        s2_mux_data(byte) <= s2_mux_data0(byte);
      end if;
    end loop;
  end process;

  -- Stage 2-3 registers: holding register + output line. Byte-identical to
  -- vhsnunzip_pipeline; it consumes the merged strobes/mux data above.
  s2_reg_proc: process (clk) is
  begin
    if rising_edge(clk) then

      s3_cnt <= s2_cnt;

      s3_out_push <= '0';
      s3_out_last <= '0';
      s3_out_cnt <= to_unsigned(C_BYTES, C_CNT);
      s3_last_pend <= '0';

      for byte in 0 to C_BYTES-1 loop
        if s2_valid = '1' and s2_int_strb(byte) = '1' then
          s3_hold_data(byte) <= s2_mux_data(byte);
        end if;
        if s2_ext_strb(byte) = '1' then
          s3_out_data(byte) <= s2_mux_data(byte);
        else
          s3_out_data(byte) <= s3_hold_data(byte);
        end if;
      end loop;

      if s3_last_pend = '1' then
        s3_out_push <= '1';
        s3_out_last <= '1';
        s3_out_cnt <= resize(s3_cnt, C_CNT);
      elsif s2_valid = '1' then
        if s2_last = '1' then
          if s2_int_strb(0) = '1' and s2_ext_strb(0) = '0' then
            s3_out_push <= '1';
            s3_last_pend <= '1';
          else
            s3_out_push <= '1';
            s3_out_last <= '1';
            if s2_ext_strb(C_BYTES-1) = '0' then
              s3_out_cnt <= resize(s2_cnt, C_CNT);
            end if;
          end if;
        elsif s2_ext_strb(C_BYTES-1) = '1' then
          s3_out_push <= '1';
        end if;
      end if;

      if reset = '1' then
        s3_out_push <= '0';
        s3_last_pend <= '0';
      end if;

    end if;
  end process;

  -- Decompressed data output FIFO.
  s3_out_ctrl(0) <= s3_out_last;
  s3_out_ctrl(C_CNT downto 1) <= std_logic_vector(s3_out_cnt);

  de_fifo_inst: vhsnunzip_fifo
    generic map (
      DATA_WIDTH  => C_BYTES,
      CTRL_WIDTH  => C_CNT+1
    )
    port map (
      clk         => clk,
      reset       => reset,
      wr_valid    => s3_out_push,
      wr_data     => s3_out_data,
      wr_ctrl     => s3_out_ctrl,
      rd_valid    => de.valid,
      rd_ready    => de_ready,
      rd_data     => de.data,
      rd_ctrl     => de_ctrl,
      level       => de_level_s
    );

  de.last <= de_ctrl(0);
  de.cnt <= unsigned(de_ctrl(C_CNT downto 1));
  de_level <= de_level_s;

  backpres <= (de_level_s(4) or de_level_s(3) or de_level_s(2)) and not de_level_s(5);

end behavior;
