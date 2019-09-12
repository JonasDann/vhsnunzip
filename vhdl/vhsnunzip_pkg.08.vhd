library ieee;
use ieee.std_logic_1164.all;

-- Package containing toplevel component declarations for vhsnunzip.
package vhsnunzip_pkg is

  -- Streaming toplevel for vhsnunzip. This version of the decompressor doesn't
  -- include any large-scale input and output stream buffering, so the streams
  -- are limited to the speed of the decompression engine.
  component vhsnunzip_streaming is
    generic (
      RAM_STYLE   : string := "URAM"
    );
    port (
      clk         : in  std_logic;
      reset       : in  std_logic;
      co_valid    : in  std_logic;
      co_ready    : out std_logic;
      co_data     : in  std_logic_vector(63 downto 0);
      co_cnt      : in  std_logic_vector(2 downto 0);
      co_last     : in  std_logic;
      de_valid    : out std_logic;
      de_ready    : in  std_logic;
      de_data     : out std_logic_vector(63 downto 0);
      de_cnt      : out std_logic_vector(3 downto 0);
      de_dvalid   : out std_logic;
      de_last     : out std_logic
    );
  end component;

  -- Buffered toplevel for a single vhsnunzip core. This version of the
  -- decompressor uses the RAMs needed for long-term decompression history
  -- storage for input/output FIFOs as well. This allows the data to be pumped
  -- in using a much wider bus (32-byte) and without stalling, but total
  -- decompression time may be longer due to memory bandwidth starvation.
  component vhsnunzip_buffered is
    generic (
      RAM_STYLE   : string := "URAM"
    );
    port (
      clk         : in  std_logic;
      reset       : in  std_logic;
      in_valid    : in  std_logic;
      in_ready    : out std_logic;
      in_data     : in  std_logic_vector(255 downto 0);
      in_cnt      : in  std_logic_vector(4 downto 0);
      in_last     : in  std_logic;
      out_valid   : out std_logic;
      out_ready   : in  std_logic;
      out_data    : out std_logic_vector(255 downto 0);
      out_cnt     : out std_logic_vector(4 downto 0);
      out_last    : out std_logic
    );
  end component;

end package vhsnunzip_pkg;
