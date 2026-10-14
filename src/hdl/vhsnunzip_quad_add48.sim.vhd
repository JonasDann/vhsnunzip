library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- 48-bit adder with registered inputs and output (two cycles of latency).
-- Simulation version.
entity vhsnunzip_quad_add48 is
  port (
    clk         : in  std_logic;
    a           : in  std_logic_vector(47 downto 0);
    c           : in  std_logic_vector(47 downto 0);
    p           : out std_logic_vector(47 downto 0)
  );
end vhsnunzip_quad_add48;

architecture behavior of vhsnunzip_quad_add48 is
  signal a_r : unsigned(47 downto 0);
  signal c_r : unsigned(47 downto 0);
begin
  reg_proc: process (clk) is
  begin
    if rising_edge(clk) then
      a_r <= unsigned(a);
      c_r <= unsigned(c);
      p <= std_logic_vector(a_r + c_r);
    end if;
  end process;
end behavior;
