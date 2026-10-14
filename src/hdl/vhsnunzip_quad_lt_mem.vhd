library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.vhsnunzip_int_pkg.all;
use work.vhsnunzip_quad_pkg.all;

-- Long-term history for the quad datapath: four copies of the 64 KiB history
-- (2048 lines of 32 bytes each), one per read bus, so the four reads of a
-- group can be any four lines. Port A of every copy writes the completed
-- output lines; port B serves one read bus.
entity vhsnunzip_quad_lt_mem is
  port (
    clk         : in  std_logic;
    reset       : in  std_logic;

    -- Reads (bus 0 .. 3); the data is valid four cycles later.
    rd_v        : in  std_logic_vector(3 downto 0);
    rd_a        : in  lt_addr_array(0 to 3);
    rdat        : out byte_matrix(0 to 3);

    -- Completed output lines.
    wr_valid    : in  std_logic;
    wr_line     : in  unsigned(10 downto 0);
    wr_data     : in  byte_array(0 to QB-1)
  );
end vhsnunzip_quad_lt_mem;

architecture behavior of vhsnunzip_quad_lt_mem is
  -- Writes are registered once before they fan out to the four copies.
  signal we      : std_logic := '0';
  signal wa      : unsigned(10 downto 0);
  signal wd      : std_logic_vector(255 downto 0);
begin

  wr_proc: process (clk) is
  begin
    if rising_edge(clk) then
      we <= wr_valid;
      wa <= wr_line;
      for i in 0 to QB-1 loop
        wd(8*i+7 downto 8*i) <= wr_data(i);
      end loop;
      if reset = '1' then
        we <= '0';
      end if;
    end if;
  end process;

  copy_gen: for b in 0 to 3 generate
    signal b_dout  : std_logic_vector(255 downto 0);
  begin

    ram_inst: entity work.vhsnunzip_quad_uram
      port map (
        clk     => clk,
        a_we    => we,
        a_addr  => wa,
        a_din   => wd,
        b_en    => rd_v(b),
        b_addr  => rd_a(b),
        b_dout  => b_dout
      );

    out_proc: process (b_dout) is
    begin
      for i in 0 to QB-1 loop
        rdat(b)(i) <= b_dout(8*i+7 downto 8*i);
      end loop;
    end process;

  end generate;

end behavior;
