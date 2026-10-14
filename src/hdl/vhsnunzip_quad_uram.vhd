library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- 2048 x 256-bit memory with one write port (A) and one read port (B), four
-- cycles of read latency. Written in the Vivado UltraRAM inference style
-- (4 x URAM288).
entity vhsnunzip_quad_uram is
  port (
    clk         : in  std_logic;
    a_we        : in  std_logic;
    a_addr      : in  unsigned(10 downto 0);
    a_din       : in  std_logic_vector(255 downto 0);
    b_en        : in  std_logic;
    b_addr      : in  unsigned(10 downto 0);
    b_dout      : out std_logic_vector(255 downto 0)
  );
end vhsnunzip_quad_uram;

architecture behavior of vhsnunzip_quad_uram is
  type mem_type is array (0 to 2047) of std_logic_vector(255 downto 0);
  shared variable mem : mem_type;
  attribute ram_style : string;
  attribute ram_style of mem : variable is "ultra";
  signal b_q, b_p, b_p2 : std_logic_vector(255 downto 0);
begin

  a_proc: process (clk) is
  begin
    if rising_edge(clk) then
      if a_we = '1' then
        mem(to_integer(a_addr)) := a_din;
      end if;
    end if;
  end process;

  b_proc: process (clk) is
  begin
    if rising_edge(clk) then
      if b_en = '1' then
        b_q <= mem(to_integer(b_addr));
      end if;
      b_p <= b_q;
      b_p2 <= b_p;
      b_dout <= b_p2;
    end if;
  end process;

end behavior;
