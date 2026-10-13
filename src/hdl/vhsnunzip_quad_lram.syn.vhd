library ieee;
use ieee.std_logic_1164.all;

library unisim;
use unisim.vcomponents.all;

-- 32-deep LUTRAM with one write address and N independent 2-bit read
-- ports ("port LUTs"), each with its own write data. Read ports are packed
-- seven per RAM32M16, and a remainder of up to three into a RAM32M; the
-- port at the write address (H / D) is unused.
-- Synthesis version (RAM32M16 / RAM32M primitives).
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
  -- Full RAM32M16s, and whether the rest goes into a RAM32M.
  constant NF : natural := N / 7;
  constant NS : natural := N mod 7;
  function nr_of(nf, ns : natural) return natural is
  begin
    if ns > 3 then return nf + 1; else return nf; end if;
  end function;
  constant NR : natural := nr_of(NF, NS);
  signal wdp  : std_logic_vector(14*NR+5 downto 0) := (others => '0');
  signal rap  : std_logic_vector(35*NR+14 downto 0) := (others => '0');
  signal rdp  : std_logic_vector(14*NR+5 downto 0);
begin
  wdp(2*N-1 downto 0) <= wd;
  rap(5*N-1 downto 0) <= ra;
  rd <= rdp(2*N-1 downto 0);

  small_gen: if NS > 0 and NS <= 3 generate
    ram_inst: RAM32M
      port map (
        DOA   => rdp(14*NR+1 downto 14*NR),
        DOB   => rdp(14*NR+3 downto 14*NR+2),
        DOC   => rdp(14*NR+5 downto 14*NR+4),
        DOD   => open,
        ADDRA => rap(35*NR+4 downto 35*NR),
        ADDRB => rap(35*NR+9 downto 35*NR+5),
        ADDRC => rap(35*NR+14 downto 35*NR+10),
        ADDRD => wa,
        DIA   => wdp(14*NR+1 downto 14*NR),
        DIB   => wdp(14*NR+3 downto 14*NR+2),
        DIC   => wdp(14*NR+5 downto 14*NR+4),
        DID   => "00",
        WCLK  => clk,
        WE    => we
      );
  end generate;

  ram_gen: for i in 0 to NR-1 generate
    ram_inst: RAM32M16
      port map (
        DOA   => rdp(14*i+1 downto 14*i),
        DOB   => rdp(14*i+3 downto 14*i+2),
        DOC   => rdp(14*i+5 downto 14*i+4),
        DOD   => rdp(14*i+7 downto 14*i+6),
        DOE   => rdp(14*i+9 downto 14*i+8),
        DOF   => rdp(14*i+11 downto 14*i+10),
        DOG   => rdp(14*i+13 downto 14*i+12),
        DOH   => open,
        ADDRA => rap(35*i+4 downto 35*i),
        ADDRB => rap(35*i+9 downto 35*i+5),
        ADDRC => rap(35*i+14 downto 35*i+10),
        ADDRD => rap(35*i+19 downto 35*i+15),
        ADDRE => rap(35*i+24 downto 35*i+20),
        ADDRF => rap(35*i+29 downto 35*i+25),
        ADDRG => rap(35*i+34 downto 35*i+30),
        ADDRH => wa,
        DIA   => wdp(14*i+1 downto 14*i),
        DIB   => wdp(14*i+3 downto 14*i+2),
        DIC   => wdp(14*i+5 downto 14*i+4),
        DID   => wdp(14*i+7 downto 14*i+6),
        DIE   => wdp(14*i+9 downto 14*i+8),
        DIF   => wdp(14*i+11 downto 14*i+10),
        DIG   => wdp(14*i+13 downto 14*i+12),
        DIH   => "00",
        WCLK  => clk,
        WE    => we
      );
  end generate;
end behavior;
