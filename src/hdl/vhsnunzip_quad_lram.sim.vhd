library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- 32-deep LUTRAM with one write address and N independent 2-bit read
-- ports. Simulation version.
entity vhsnunzip_quad_lram is
  generic (
    N           : natural
  );
  port (
    clk         : in  std_logic;
    we          : in  std_logic;
    wa          : in  std_logic_vector(4 downto 0);
    wd          : in  std_logic_vector(2*N-1 downto 0);
    ra          : in  std_logic_vector(5*N-1 downto 0);
    rd          : out std_logic_vector(2*N-1 downto 0)
  );
end vhsnunzip_quad_lram;

architecture behavior of vhsnunzip_quad_lram is
  type mem_type is array (0 to 31) of std_logic_vector(2*N-1 downto 0);
  signal mem : mem_type := (others => (others => '0'));
begin
  wr_proc: process (clk) is
  begin
    if rising_edge(clk) then
      if we = '1' then
        mem(to_integer(unsigned(wa))) <= wd;
      end if;
    end if;
  end process;
  rd_gen: for i in 0 to N-1 generate
    rd(2*i+1 downto 2*i) <= mem(to_integer(unsigned(ra(5*i+4 downto 5*i))))(2*i+1 downto 2*i);
  end generate;
end behavior;
