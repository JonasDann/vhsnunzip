library ieee;
use ieee.std_logic_1164.all;

library unisim;
use unisim.vcomponents.all;

-- 48-bit adder with registered inputs and output (two cycles of latency):
-- one DSP48E2 (A:B + C, AREG = BREG = CREG = PREG = 1).
-- Synthesis version.
entity vhsnunzip_quad_add48 is
  port (
    clk         : in  std_logic;
    a           : in  std_logic_vector(47 downto 0);
    c           : in  std_logic_vector(47 downto 0);
    p           : out std_logic_vector(47 downto 0)
  );
end vhsnunzip_quad_add48;

architecture behavior of vhsnunzip_quad_add48 is
begin
  dsp_inst: DSP48E2
    generic map (
      USE_MULT      => "NONE",
      USE_SIMD      => "ONE48",
      AREG          => 1,
      ACASCREG      => 1,
      BREG          => 1,
      BCASCREG      => 1,
      CREG          => 1,
      DREG          => 0,
      ADREG         => 0,
      MREG          => 0,
      PREG          => 1,
      OPMODEREG     => 0,
      ALUMODEREG    => 0,
      INMODEREG     => 0,
      CARRYINREG    => 0,
      CARRYINSELREG => 0
    )
    port map (
      -- X = A:B, Y = 0, Z = C, W = 0; P = Z + X.
      OPMODE        => "000110011",
      ALUMODE       => "0000",
      INMODE        => "00000",
      CARRYINSEL    => "000",
      CARRYIN       => '0',
      A             => a(47 downto 18),
      B             => a(17 downto 0),
      C             => c,
      D             => (others => '0'),
      ACIN          => (others => '0'),
      BCIN          => (others => '0'),
      PCIN          => (others => '0'),
      CARRYCASCIN   => '0',
      MULTSIGNIN    => '0',
      CLK           => clk,
      CEA1          => '1',
      CEA2          => '1',
      CEB1          => '1',
      CEB2          => '1',
      CEC           => '1',
      CED           => '0',
      CEAD          => '0',
      CEM           => '0',
      CEP           => '1',
      CECTRL        => '0',
      CEALUMODE     => '0',
      CEINMODE      => '0',
      CECARRYIN     => '0',
      RSTA          => '0',
      RSTB          => '0',
      RSTC          => '0',
      RSTD          => '0',
      RSTM          => '0',
      RSTP          => '0',
      RSTCTRL       => '0',
      RSTALLCARRYIN => '0',
      RSTALUMODE    => '0',
      RSTINMODE     => '0',
      P             => p,
      ACOUT         => open,
      BCOUT         => open,
      PCOUT         => open,
      CARRYCASCOUT  => open,
      MULTSIGNOUT   => open,
      CARRYOUT      => open,
      OVERFLOW      => open,
      UNDERFLOW     => open,
      PATTERNDETECT => open,
      PATTERNBDETECT => open,
      XOROUT        => open
    );
end behavior;
