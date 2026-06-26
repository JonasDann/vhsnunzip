library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.vhsnunzip_utils_pkg.all;
use work.vhsnunzip_int_pkg.all;

-- Speculative dual-issue command generator stage 2.
--
-- Slot 0 is the single-issue address generator (vhsnunzip_cmd_gen_2): it splits
-- long literals over multiple cycles and carries the running decompression
-- offset and long-term line pointer. Slot 1 generates a second command in the
-- same cycle, from slot 0's *advanced* offset/line-pointer, but only when the
-- lead partial command retires fully this cycle (no carried literal state) and
-- the next partial command is itself a single, self-contained command. The
-- flattened output (cm0 then cm1 where present) is identical to the single-issue
-- command sequence, which the standalone testbench checks against cm.tv.
entity vhsnunzip_cmd_gen_2_dual is
  generic (
    LONG_CHUNKS : boolean := true
  );
  port (
    clk         : in  std_logic;
    reset       : in  std_logic;

    c1          : in  dual_partial_command_stream;
    c1_ready    : out std_logic;

    lt_off_ld   : in  std_logic := '1';
    lt_off      : in  unsigned(C_AW downto 0) := (others => '0');

    cm          : out dual_command_stream;
    cm_ready    : in  std_logic
  );
end vhsnunzip_cmd_gen_2_dual;

architecture behavior of vhsnunzip_cmd_gen_2_dual is

  -- Whether a partial command retires in exactly one command: a copy-only or
  -- empty command, or one whose literal fits the remaining budget and whose
  -- literal data starts in the current line (li_off < C_BYTES). Last commands are
  -- never co-issued, to keep the end-of-chunk stall handling on slot 0.
  function simple_cmd(c : partial_command_stream) return boolean is
    variable budget : unsigned(C_CNT-1 downto 0);
  begin
    if c.last = '1' then
      return false;
    end if;
    budget := unsigned(c.cp_len(C_CNT-1 downto 0)) xor to_unsigned(C_BYTES-1, C_CNT);
    if c.li_val = '0' then
      return true;
    elsif c.li_off(C_IDX) = '1' then
      return false;
    elsif c.li_len < budget then     -- li_len + 1 <= budget
      return true;
    else
      return false;
    end if;
  end function;

  -- Generate the single command for a simple partial command, advancing off and
  -- lt_ptr. This mirrors the single-issue cmd_gen_2 body for a command that
  -- retires in one cycle (no copy/literal carry).
  procedure gen_simple(
    c      : in    partial_command_stream;
    off    : inout unsigned(C_CNT-1 downto 0);
    lt_ptr : inout unsigned(C_AW downto 0);
    cmd    : out   command_stream
  ) is
    variable cp_rel : signed(16 downto 0);
    variable cp_lt  : unsigned(C_AW downto 0);
    variable len    : unsigned(C_CNT-1 downto 0);
  begin
    cmd.valid  := '1';
    cmd.cp_rle := c.cp_rle;

    cp_rel := signed(resize(off, 17)) - signed(resize(c.cp_off, 17));
    cmd.st_addr := not unsigned(cp_rel(C_IDX+4 downto C_IDX));
    cp_lt := lt_ptr + unsigned(cp_rel(C_IDX+C_AW downto C_IDX));
    cmd.lt_swap := cp_lt(0);
    cmd.lt_adev := cp_lt(C_AW downto 1) + cp_lt(0 downto 0);
    cmd.lt_adod := cp_lt(C_AW downto 1);
    if cp_rel(16 downto C_IDX) < -31 then
      cmd.lt_val := not c.cp_len(C_CNT-1);
    else
      cmd.lt_val := '0';
    end if;

    if c.cp_rle = '1' then
      cmd.cp_rol := resize(unsigned(cp_rel(C_IDX-1 downto 0)), C_WIN);
    else
      cmd.cp_rol := unsigned(cp_rel(C_IDX-1 downto 0)) - off;
    end if;

    off := off + unsigned(c.cp_len) + 1;
    cmd.cp_end := off;

    if c.li_val = '1' then
      len := unsigned(c.li_len(C_CNT-1 downto 0)) + 1;
    else
      len := (others => '0');
    end if;

    cmd.li_rol := c.li_off - off;
    off := off + len;
    cmd.li_end := off;

    if off(C_IDX) = '1' then
      lt_ptr := lt_ptr + 1;
    end if;
    off(C_IDX) := '0';

    cmd.ld_pop := c.ld_pop;
    cmd.last   := c.last;
  end procedure;

  function li_high_fn return natural is
  begin
    if LONG_CHUNKS then
      return 32;
    else
      return 16;
    end if;
  end function;

begin
  proc: process (clk) is

    -- Input holding registers: lead (slot 0) and lookahead (slot 1 candidate).
    variable c1_0h  : partial_command_stream := PARTIAL_COMMAND_STREAM_INIT;
    variable c1_1h  : partial_command_stream := PARTIAL_COMMAND_STREAM_INIT;

    -- Running decompression state (shared; slot 0 owns the multi-cycle state).
    variable lt_val : std_logic;
    variable lt_ptr : unsigned(C_AW downto 0);
    variable c1_pend: std_logic;
    variable cp_len : signed(C_CNT-1 downto 0) := (others => '1');
    variable li_len : signed(li_high_fn downto 0) := (others => '1');
    variable li_off : unsigned(C_WIN-1 downto 0);
    variable off    : unsigned(C_CNT-1 downto 0);
    variable budget : unsigned(C_CNT-1 downto 0);
    variable len    : unsigned(C_CNT-1 downto 0);

    variable cp_rel : signed(16 downto 0);
    variable cp_lt  : unsigned(C_AW downto 0);
    variable advance: boolean;

    -- Contained-fold gate state. off_in is slot 0's running offset before this
    -- command, so b0 = cm0.li_end - off_in is the number of bytes slot 0 writes
    -- this cycle. The slot-1 candidate is generated into temporaries first, so a
    -- non-foldable pair can still be declined without committing off/lt_ptr.
    variable off_in : unsigned(C_CNT-1 downto 0);
    variable off_try: unsigned(C_CNT-1 downto 0);
    variable lt_try : unsigned(C_AW downto 0);
    variable cm1_try: command_stream;
    variable b0     : unsigned(C_CNT downto 0);
    variable b1     : unsigned(C_CNT downto 0);

    -- Output holding register.
    variable cmh    : dual_command_stream := DUAL_COMMAND_STREAM_INIT;

    variable stall  : std_logic;

  begin
    if rising_edge(clk) then

      -- Insert a stall cycle after the last transfer of a chunk (slot 0 only,
      -- as last commands are never co-issued).
      stall := cmh.cm0.valid and cm_ready and cmh.cm0.last;

      -- Invalidate the output register if it was shifted out.
      if cm_ready = '1' then
        cmh.cm0.valid := '0';
        cmh.cm1.valid := '0';
      end if;

      -- Latch a fresh dual transfer when both input slots are free.
      if c1_0h.valid = '0' and c1_1h.valid = '0' and lt_val = '1' then
        c1_0h := c1.c1_0;
        c1_1h := c1.c1_1;
        if c1.c1_0.valid = '1' then
          c1_pend := (not c1.c1_0.cp_len(C_CNT-1)) or c1.c1_0.li_val;
        end if;
      end if;

      -- Process slot 0 (single-issue cmd_gen_2 on the lead command).
      if c1_0h.valid = '1' and cmh.cm0.valid = '0' and stall = '0' then
        cmh.cm0.valid := '1';

        -- Capture slot 0's offset before it advances, to size b0 below.
        off_in := off;

        if li_len(li_len'high) = '1' and c1_pend = '1' then
          cp_len := c1_0h.cp_len;
          if c1_0h.li_val = '1' then
            li_len := signed(resize(c1_0h.li_len, li_len'length));
          end if;
          li_off := c1_0h.li_off;
          c1_pend := '0';
        end if;

        cp_rel := signed(resize(off, 17)) - signed(resize(c1_0h.cp_off, 17));
        cmh.cm0.st_addr := not unsigned(cp_rel(C_IDX+4 downto C_IDX));
        cp_lt := lt_ptr + unsigned(cp_rel(C_IDX+C_AW downto C_IDX));
        cmh.cm0.lt_swap := cp_lt(0);
        cmh.cm0.lt_adev := cp_lt(C_AW downto 1) + cp_lt(0 downto 0);
        cmh.cm0.lt_adod := cp_lt(C_AW downto 1);
        if cp_rel(16 downto C_IDX) < -31 then
          cmh.cm0.lt_val := not cp_len(C_CNT-1);
        else
          cmh.cm0.lt_val := '0';
        end if;

        cmh.cm0.cp_rle := c1_0h.cp_rle;
        if c1_0h.cp_rle = '1' then
          cmh.cm0.cp_rol := resize(unsigned(cp_rel(C_IDX-1 downto 0)), C_WIN);
        else
          cmh.cm0.cp_rol := unsigned(cp_rel(C_IDX-1 downto 0)) - off;
        end if;

        budget := unsigned(cp_len(C_CNT-1 downto 0)) xor to_unsigned(C_BYTES-1, C_CNT);
        off := off + unsigned(cp_len) + 1;
        cp_len := (others => '1');
        cmh.cm0.cp_end := off;

        if li_len < signed(resize(budget, li_len'length)) then
          len := unsigned(li_len(C_CNT-1 downto 0)) + 1;
        else
          len := budget;
        end if;
        if li_off(C_IDX) = '1' then
          len := (others => '0');
        end if;

        cmh.cm0.li_rol := li_off - off;
        off := off + len;
        li_off := li_off + len;
        li_len := li_len - signed(resize(len, li_len'length));
        cmh.cm0.li_end := off;

        -- Bytes slot 0 writes this cycle (li_end is captured before the wrap,
        -- so this is the true 0..C_BYTES count regardless of line crossing).
        b0 := resize(cmh.cm0.li_end, b0'length) - resize(off_in, b0'length);

        if off(C_IDX) = '1' then
          lt_ptr := lt_ptr + 1;
        end if;
        off(C_IDX) := '0';

        advance := true;
        if c1_pend = '1' then
          advance := false;
        end if;
        if li_len(li_len'high) = '0' and li_off < C_BYTES then
          advance := false;
        end if;
        if c1_0h.last = '1' and li_len(li_len'high) = '0' then
          advance := false;
        end if;

        if advance then
          cmh.cm0.ld_pop := c1_0h.ld_pop;
          cmh.cm0.last   := c1_0h.last;
          li_off := li_off - C_BYTES;
          if c1_0h.last = '1' then
            lt_val := '0';
            off := (others => '0');
          end if;

          -- Slot 1: co-issue the lookahead when the lead retired fully (literal
          -- done) this cycle, it isn't the chunk's last command, and the
          -- lookahead is itself a single self-contained command.
          if li_len(li_len'high) = '1' and c1_0h.last = '0'
             and c1_1h.valid = '1' and simple_cmd(c1_1h) then

            -- Generate the slot-1 candidate into temporaries, so the running
            -- offset/line pointer can be left untouched if we decline.
            off_try := off;
            lt_try  := lt_ptr;
            gen_simple(c1_1h, off_try, lt_try, cm1_try);
            b1 := resize(cm1_try.li_end, b1'length) - resize(off, b1'length);

            -- Contained-fold gate: the pair must produce at most one line of
            -- output (b0 + b1 <= C_BYTES, so at most one line completes and each
            -- short-term lane is written at most once this cycle -- the cm0 and
            -- cm1 lane ranges are then provably disjoint). Slot 1 may now read
            -- long-term: the datapath mirrors the history URAM into a second
            -- read bank dedicated to cm1 (see vhsnunzip_unbuffered), so cm0 and
            -- cm1 have independent long-term read ports.
            if (b0 + b1 <= C_BYTES) then
              cmh.cm1 := cm1_try;
              off     := off_try;
              lt_ptr  := lt_try;
              c1_0h.valid := '0';
              c1_1h.valid := '0';
            else
              -- Pair not foldable; defer the lookahead to its own cycle by
              -- shifting it into the lead slot (off/lt_ptr stay as slot 0 left
              -- them; the candidate's temporaries are discarded).
              cmh.cm1.valid := '0';
              c1_0h := c1_1h;
              c1_pend := (not c1_1h.cp_len(C_CNT-1)) or c1_1h.li_val;
              c1_1h.valid := '0';
            end if;

          else
            cmh.cm1.valid := '0';
            -- Cannot co-issue; shift the lookahead into the lead slot, keeping
            -- any carried literal state (deferred-literal continuation).
            c1_0h := c1_1h;
            if c1_1h.valid = '1' then
              c1_pend := (not c1_1h.cp_len(C_CNT-1)) or c1_1h.li_val;
            end if;
            c1_1h.valid := '0';
          end if;
        else
          cmh.cm0.ld_pop := '0';
          cmh.cm0.last   := '0';
          cmh.cm1.valid  := '0';
        end if;

      end if;

      -- Load the long-term memory pointer when we get it.
      if lt_val = '0' and lt_off_ld = '1' then
        lt_ptr := lt_off;
        lt_val := '1';
      end if;

      -- Handle reset.
      if reset = '1' then
        c1_0h.valid := '0';
        c1_1h.valid := '0';
        cmh.cm0.valid := '0';
        cmh.cm1.valid := '0';
        lt_val := '0';
        c1_pend := '0';
        cp_len := (others => '1');
        li_len := (others => '1');
        off := (others => '0');
      end if;

      -- Assign outputs.
      if lt_val = '1' and c1_0h.valid = '0' and c1_1h.valid = '0' then
        c1_ready <= '1';
      else
        c1_ready <= '0';
      end if;
      cm <= cmh;

    end if;
  end process;
end behavior;
