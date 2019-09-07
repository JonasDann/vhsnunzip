library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package vhsnunzip_pkg is

  -- Generic array of bytes.
  type byte_array is array (natural range <>) of std_logic_vector(7 downto 0);

  -- Payload of the compressed data stream.
  type compressed_stream_payload is record

    -- Compressed data line.
    data      : byte_array(0 to 7);

    -- Asserted to mark the last line of a chunk.
    last      : std_logic;

    -- Index of the last valid byte. Assumed to be 7 when last is not asserted.
    endi      : std_logic_vector(2 downto 0);

  end record;

  -- Snappy element information and literal data stream.
  type element_stream_payload is record

    -- Whether the copy element info is valid.
    cp_valid  : std_logic;

    -- The byte offset for the copy as encoded by the element header.
    cp_offs   : std_logic_vector(15 downto 0);

    -- The DIMINISHED-ONE length of the copy as encoded by the element header.
    cp_len    : std_logic_vector(5 downto 0);

    -- Whether the literal element info is valid.
    li_valid  : std_logic;

    -- The starting byte offset within the current li_data for the literal.
    li_offs   : std_logic_vector(3 downto 0);

    -- The DIMINISHED-ONE length of the literal as encoded by the element
    -- header.
    li_len    : std_logic_vector(15 downto 0);

    -- Literal data line pair.
    li_data   : byte_array(0 to 15);

    -- Indicator for first set of elements in chunk.
    first     : std_logic;

    -- Indicator for last set of elements/literal data in chunk.
    last      : std_logic;

  end record;

  -- Returns 'U' during simulation, but '0' during synthesis.
  function undef_fn return std_logic is
    variable undef: std_logic := '0';
  begin
    -- pragma translate_off
    undef := 'U';
    -- pragma translate_on
    return undef;
  end function;
  constant UNDEF    : std_logic := undef_fn;

  component vhsnunzip_srl is
    generic (
      WIDTH       : natural := 8;
      DEPTH_LOG2  : natural := 5
    );
    port (
      clk         : in  std_logic;
      wr_ena      : in  std_logic;
      wr_data     : in  std_logic_vector(WIDTH-1 downto 0);
      rd_addr     : in  std_logic_vector(DEPTH_LOG2-1 downto 0) := (others => '0');
      rd_data     : out std_logic_vector(WIDTH-1 downto 0)
    );
  end component;

  component vhsnunzip_fifo is
    generic (
      DATA_WIDTH  : natural := 0;
      CTRL_WIDTH  : natural := 0;
      DEPTH_LOG2  : natural := 5
    );
    port (
      clk         : in  std_logic;
      reset       : in  std_logic;
      wr_valid    : in  std_logic;
      wr_ready    : out std_logic;
      wr_data     : in  byte_array(DATA_WIDTH-1 downto 0) := (others => X"00");
      wr_ctrl     : in  std_logic_vector(CTRL_WIDTH-1 downto 0) := (others => '0');
      rd_valid    : out std_logic;
      rd_ready    : in  std_logic;
      rd_data     : out byte_array(DATA_WIDTH-1 downto 0);
      rd_ctrl     : out std_logic_vector(CTRL_WIDTH-1 downto 0);
      level       : out std_logic_vector(DEPTH_LOG2 downto 0);
      empty       : out std_logic;
      full        : out std_logic
    );
  end component;

end package vhsnunzip_pkg;
