library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.vhsnunzip_utils_pkg.all;
use work.vhsnunzip_int_pkg.all;

-- pragma vhdeps ignore package vcomponents
library unisim;
use unisim.vcomponents.all;

-- pragma vhdeps ignore package vcomponents
library unimacro;
use unimacro.vcomponents.all;

-- Primitive instantiation of a Xilinx ultra or collection of block RAMs, with
-- two R/W access ports and a total read latency of exactly 3 cycles.
--
-- This is parametrised by the datapath lane width C_BYTES:
--  - "ultra": one 72-bit UltraRAM per 8 bytes of line (C_BYTES/8 URAMs per
--    port); the 8-bit control word is stored in the spare bits of bank 0.
--    C_BYTES must be a multiple of 8.
--  - "block": one 9-bit-wide 36k block RAM per byte lane (C_BYTES per port);
--    the control word (8 bits) lives in the parity bit of the low 8 lanes.
-- At C_BYTES = 8 this is identical to the original 8-byte core.
entity vhsnunzip_ram is
  generic (

    -- Select "ultra" to instantiate UltraRAM blocks, or "block" to use
    -- 36kib block RAMs.
    RAM_STYLE   : string := "ultra"

  );
  port (
    clk         : in  std_logic;
    reset       : in  std_logic;

    -- Access port A.
    a_cmd       : in  ram_command;
    a_resp      : out ram_response;

    -- Access port B.
    b_cmd       : in  ram_command;
    b_resp      : out ram_response

  );
end vhsnunzip_ram;

architecture behavior of vhsnunzip_ram is
begin

  -- Implementation for ultra RAMs: C_BYTES/8 UltraRAMs per port, side by side.
  uram_gen: if RAM_STYLE = "ultra" generate
    -- Number of 72-bit UltraRAMs needed per port to hold C_BYTES data bytes.
    -- Each holds 8 data bytes (64 bits); the 8-bit control word fits in the
    -- spare bits of bank 0.
    constant NU : natural := (C_BYTES + 7) / 8;
    type slv72_array is array (natural range <>) of std_logic_vector(71 downto 0);

    signal a_addr     : std_logic_vector(22 downto 0);
    signal a_wdat     : slv72_array(0 to NU-1);
    signal a_rdat     : slv72_array(0 to NU-1);
    signal a_rval_r   : std_logic;
    signal a_rval_rr  : std_logic;
    signal a_rval_rrr : std_logic;

    signal b_addr     : std_logic_vector(22 downto 0);
    signal b_wdat     : slv72_array(0 to NU-1);
    signal b_rdat     : slv72_array(0 to NU-1);
    signal b_rval_r   : std_logic;
    signal b_rval_rr  : std_logic;
    signal b_rval_rrr : std_logic;
  begin

    a_addr <= std_logic_vector(resize(a_cmd.addr, 23));
    b_addr <= std_logic_vector(resize(b_cmd.addr, 23));

    bank_gen: for u in 0 to NU-1 generate
    begin

      -- Pack the C_BYTES-wide write data into the per-bank 72-bit words. Each
      -- bank takes 8 data bytes; bank 0 additionally carries the control word.
      a_pack_proc: process (a_cmd) is
      begin
        a_wdat(u) <= (others => '0');
        for byte in 0 to 7 loop
          if u*8 + byte < C_BYTES then
            a_wdat(u)(byte*8+7 downto byte*8) <= a_cmd.wdat(u*8 + byte);
          end if;
        end loop;
        if u = 0 then
          a_wdat(u)(71 downto 64) <= a_cmd.wctrl;
        end if;
      end process;

      b_pack_proc: process (b_cmd) is
      begin
        b_wdat(u) <= (others => '0');
        for byte in 0 to 7 loop
          if u*8 + byte < C_BYTES then
            b_wdat(u)(byte*8+7 downto byte*8) <= b_cmd.wdat(u*8 + byte);
          end if;
        end loop;
        if u = 0 then
          b_wdat(u)(71 downto 64) <= b_cmd.wctrl;
        end if;
      end process;

      -- pragma vhdeps ignore component uram288_base
      uram_inst : uram288_base
        generic map (
          IREG_PRE_A  => "TRUE",
          IREG_PRE_B  => "TRUE",
          OREG_A      => "TRUE",
          OREG_B      => "TRUE"
        )
        port map (
          clk         => clk,
          rst_a       => '0',
          rst_b       => '0',
          sleep       => '0',

          -- Port A interface.
          en_a        => a_cmd.valid,
          addr_a      => a_addr,
          rdb_wr_a    => a_cmd.wren,
          bwe_a       => "111111111",
          din_a       => a_wdat(u),
          dout_a      => a_rdat(u),

          -- Port B interface.
          en_b        => b_cmd.valid,
          addr_b      => b_addr,
          rdb_wr_b    => b_cmd.wren,
          bwe_b       => "111111111",
          din_b       => b_wdat(u),
          dout_b      => b_rdat(u),

          -- Port A control bits.
          oreg_ce_a         => '1',
          oreg_ecc_ce_a     => '1',
          inject_dbiterr_a  => '0',
          inject_sbiterr_a  => '0',

          -- Port B control bits.
          oreg_ce_b         => '1',
          oreg_ecc_ce_b     => '1',
          inject_dbiterr_b  => '0',
          inject_sbiterr_b  => '0'
        );

    end generate;

    a_resp_connect_proc: process (a_rdat, a_rval_rr, a_rval_rrr) is
    begin
      for byte in 0 to C_BYTES-1 loop
        a_resp.rdat(byte) <= a_rdat(byte/8)((byte mod 8)*8+7 downto (byte mod 8)*8);
      end loop;
      a_resp.rctrl <= a_rdat(0)(71 downto 64);
      a_resp.valid <= a_rval_rrr;
      a_resp.valid_next <= a_rval_rr;
    end process;

    b_resp_connect_proc: process (b_rdat, b_rval_rr, b_rval_rrr) is
    begin
      for byte in 0 to C_BYTES-1 loop
        b_resp.rdat(byte) <= b_rdat(byte/8)((byte mod 8)*8+7 downto (byte mod 8)*8);
      end loop;
      b_resp.rctrl <= b_rdat(0)(71 downto 64);
      b_resp.valid <= b_rval_rrr;
      b_resp.valid_next <= b_rval_rr;
    end process;

    resp_valid_proc: process (clk) is
    begin
      if rising_edge(clk) then
        a_rval_r <= a_cmd.valid and not a_cmd.wren;
        b_rval_r <= b_cmd.valid and not b_cmd.wren;
        a_rval_rr <= a_rval_r;
        b_rval_rr <= b_rval_r;
        a_rval_rrr <= a_rval_rr;
        b_rval_rrr <= b_rval_rr;
        if reset = '1' then
          a_rval_r <= '0';
          b_rval_r <= '0';
          a_rval_rr <= '0';
          b_rval_rr <= '0';
          a_rval_rrr <= '0';
          b_rval_rrr <= '0';
        end if;
      end if;
    end process;

  end generate;

  bram_gen: if RAM_STYLE = "block" generate
    type data_array is array (natural range <>) of std_logic_vector(8 downto 0);

    signal a_ena      : std_logic;
    signal a_addr     : std_logic_vector(11 downto 0);
    signal a_we       : std_logic_vector(0 downto 0);
    signal a_rval_r   : std_logic;
    signal a_rval_rr  : std_logic;
    signal a_rval_rrr : std_logic;
    signal a_wdat     : data_array(0 to C_BYTES-1);
    signal a_rdat     : data_array(0 to C_BYTES-1);

    signal b_ena      : std_logic;
    signal b_addr     : std_logic_vector(11 downto 0);
    signal b_we       : std_logic_vector(0 downto 0);
    signal b_rval_r   : std_logic;
    signal b_rval_rr  : std_logic;
    signal b_rval_rrr : std_logic;
    signal b_wdat     : data_array(0 to C_BYTES-1);
    signal b_rdat     : data_array(0 to C_BYTES-1);

  begin

    a_cmd_connect_proc: process (clk) is
    begin
      if rising_edge(clk) then
        a_ena <= a_cmd.valid;
        a_addr <= std_logic_vector(resize(a_cmd.addr, 12));
        a_we <= (others => a_cmd.wren);
        for byte in 0 to C_BYTES-1 loop
          a_wdat(byte)(7 downto 0) <= a_cmd.wdat(byte);
          -- The control word is only 8 bits wide; carry it in the parity bit
          -- of the low 8 byte lanes.
          if byte < 8 then
            a_wdat(byte)(8) <= a_cmd.wctrl(byte);
          else
            a_wdat(byte)(8) <= '0';
          end if;
        end loop;
      end if;
    end process;

    b_cmd_connect_proc: process (clk) is
    begin
      if rising_edge(clk) then
        b_ena <= b_cmd.valid;
        b_addr <= std_logic_vector(resize(b_cmd.addr, 12));
        b_we <= (others => b_cmd.wren);
        for byte in 0 to C_BYTES-1 loop
          b_wdat(byte)(7 downto 0) <= b_cmd.wdat(byte);
          if byte < 8 then
            b_wdat(byte)(8) <= b_cmd.wctrl(byte);
          else
            b_wdat(byte)(8) <= '0';
          end if;
        end loop;
      end if;
    end process;

    byte_gen: for byte in 0 to C_BYTES-1 generate
    begin
      -- pragma vhdeps ignore component bram_tdp_macro
      bram: bram_tdp_macro
        generic map (
          BRAM_SIZE     => "36Kb",
          DOA_REG       => 1,
          DOB_REG       => 1,
          READ_WIDTH_A  => 9,
          READ_WIDTH_B  => 9,
          WRITE_WIDTH_A => 9,
          WRITE_WIDTH_B => 9
        )
        port map (
          clka          => clk,
          clkb          => clk,
          rsta          => '0',
          rstb          => '0',
          regcea        => '1',
          regceb        => '1',

          ena           => a_ena,
          addra         => a_addr,
          wea           => a_we,
          dia           => a_wdat(byte),
          doa           => a_rdat(byte),

          enb           => b_ena,
          addrb         => b_addr,
          web           => b_we,
          dib           => b_wdat(byte),
          dob           => b_rdat(byte)
        );
    end generate;

    a_resp_connect_proc: process (a_rdat, a_rval_rr, a_rval_rrr) is
    begin
      for byte in 0 to C_BYTES-1 loop
        a_resp.rdat(byte) <= a_rdat(byte)(7 downto 0);
      end loop;
      -- rctrl is 8 bits; only the low 8 byte lanes carry it.
      for byte in 0 to 7 loop
        a_resp.rctrl(byte) <= a_rdat(byte)(8);
      end loop;
      a_resp.valid <= a_rval_rrr;
      a_resp.valid_next <= a_rval_rr;
    end process;

    b_resp_connect_proc: process (b_rdat, b_rval_rr, b_rval_rrr) is
    begin
      for byte in 0 to C_BYTES-1 loop
        b_resp.rdat(byte) <= b_rdat(byte)(7 downto 0);
      end loop;
      for byte in 0 to 7 loop
        b_resp.rctrl(byte) <= b_rdat(byte)(8);
      end loop;
      b_resp.valid <= b_rval_rrr;
      b_resp.valid_next <= b_rval_rr;
    end process;

    resp_valid_proc: process (clk) is
    begin
      if rising_edge(clk) then
        a_rval_r <= a_cmd.valid and not a_cmd.wren;
        b_rval_r <= b_cmd.valid and not b_cmd.wren;
        a_rval_rr <= a_rval_r;
        b_rval_rr <= b_rval_r;
        a_rval_rrr <= a_rval_rr;
        b_rval_rrr <= b_rval_rr;
        if reset = '1' then
          a_rval_r <= '0';
          b_rval_r <= '0';
          a_rval_rr <= '0';
          b_rval_rr <= '0';
          a_rval_rrr <= '0';
          b_rval_rrr <= '0';
        end if;
      end if;
    end process;

  end generate;

end behavior;
