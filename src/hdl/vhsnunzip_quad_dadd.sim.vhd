library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Registered adder p = a + c + ci, lt32 = (p < 32). Simulation version.
entity vhsnunzip_quad_dadd is
  port (
    clk         : in  std_logic;
    ce          : in  std_logic;
    a           : in  unsigned(16 downto 0);
    c           : in  unsigned(5 downto 0);
    ci          : in  std_logic;
    p           : out unsigned(17 downto 0);
    lt32        : out std_logic
  );
end vhsnunzip_quad_dadd;

architecture behavior of vhsnunzip_quad_dadd is
begin
  reg_proc: process (clk) is
    variable s : unsigned(17 downto 0);
  begin
    if rising_edge(clk) then
      if ce = '1' then
        s := resize(a, 18) + resize(c, 18);
        if ci = '1' then
          s := s + 1;
        end if;
        p <= s;
        if s(17 downto 5) = 0 then
          lt32 <= '1';
        else
          lt32 <= '0';
        end if;
      end if;
    end if;
  end process;
end behavior;
