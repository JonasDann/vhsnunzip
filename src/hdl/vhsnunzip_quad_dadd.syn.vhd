library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library unisim;
use unisim.vcomponents.all;

-- Registered adder p = a + c + ci in a DSP slice (ALU only, P register with
-- clock enable ce), plus lt32 = (p < 32) from the pattern detector.
-- Synthesis version (DSP48E2 primitive).
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
  signal b48  : std_logic_vector(17 downto 0);
  signal c48  : std_logic_vector(47 downto 0);
  signal p48  : std_logic_vector(47 downto 0);
begin
  b48 <= '0' & std_logic_vector(a);
  c48 <= std_logic_vector(resize(c, 48));
  p <= unsigned(p48(17 downto 0));

  dsp_inst: DSP48E2
    generic map (
      USE_MULT            => "NONE",
      USE_SIMD            => "ONE48",
      USE_PATTERN_DETECT  => "PATDET",
      SEL_PATTERN         => "PATTERN",
      SEL_MASK            => "MASK",
      PATTERN             => X"000000000000",
      MASK                => X"FFFFFFFC001F",   -- compare p(17 downto 5) with 0
      AREG                => 0,
      ACASCREG            => 0,
      BREG                => 0,
      BCASCREG            => 0,
      CREG                => 0,
      DREG                => 0,
      ADREG               => 0,
      MREG                => 0,
      PREG                => 1,
      OPMODEREG           => 0,
      ALUMODEREG          => 0,
      INMODEREG           => 0,
      CARRYINREG          => 0,
      CARRYINSELREG       => 0
    )
    port map (
      CLK           => clk,
      A             => (others => '0'),
      B             => b48,
      C             => c48,
      D             => (others => '0'),
      CARRYIN       => ci,
      OPMODE        => "000110011",     -- W = 0, Z = C, Y = 0, X = A:B
      ALUMODE       => "0000",          -- Z + W + X + Y + CIN
      INMODE        => "00000",
      CARRYINSEL    => "000",
      ACIN          => (others => '0'),
      BCIN          => (others => '0'),
      PCIN          => (others => '0'),
      CARRYCASCIN   => '0',
      MULTSIGNIN    => '0',
      CEA1          => '0',
      CEA2          => '0',
      CEAD          => '0',
      CEALUMODE     => '0',
      CEB1          => '0',
      CEB2          => '0',
      CEC           => '0',
      CECARRYIN     => '0',
      CECTRL        => '0',
      CED           => '0',
      CEINMODE      => '0',
      CEM           => '0',
      CEP           => ce,
      RSTA          => '0',
      RSTALLCARRYIN => '0',
      RSTALUMODE    => '0',
      RSTB          => '0',
      RSTC          => '0',
      RSTCTRL       => '0',
      RSTD          => '0',
      RSTINMODE     => '0',
      RSTM          => '0',
      RSTP          => '0',
      P             => p48,
      PATTERNDETECT => lt32,
      ACOUT         => open,
      BCOUT         => open,
      PCOUT         => open,
      CARRYCASCOUT  => open,
      MULTSIGNOUT   => open,
      CARRYOUT      => open,
      OVERFLOW      => open,
      UNDERFLOW     => open,
      PATTERNBDETECT => open,
      XOROUT        => open
    );
end behavior;
