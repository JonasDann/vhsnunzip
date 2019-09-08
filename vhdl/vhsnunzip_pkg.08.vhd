library std;
use std.textio.all;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_textio.all;

package vhsnunzip_pkg is

  -- Returns 'U' during simulation, but '0' during synthesis.
  function undef_fn return std_logic;
  constant UNDEF    : std_logic := undef_fn;

  -- Generic array of bytes.
  type byte_array is array (natural range <>) of std_logic_vector(7 downto 0);

  -- Behavioral description of a shift register lookup.
  component vhsnunzip_srl is
    generic (
      WIDTH       : natural := 8;
      DEPTH_LOG2  : natural := 5
    );
    port (
      clk         : in  std_logic;
      wr_ena      : in  std_logic;
      wr_data     : in  std_logic_vector(WIDTH-1 downto 0);
      rd_addr     : in  unsigned(DEPTH_LOG2-1 downto 0) := (others => '0');
      rd_data     : out std_logic_vector(WIDTH-1 downto 0)
    );
  end component;

  -- FIFO component based on vhsnunzip_srl.
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
      level       : out unsigned(DEPTH_LOG2 downto 0);
      empty       : out std_logic;
      full        : out std_logic
    );
  end component;

  -- Payload of the compressed data stream from the memory to the decoder. This
  -- passes through an SRL-based FIFO for buffering.
  type compressed_stream_single is record

    -- Stream valid signal.
    valid     : std_logic;

    -- Compressed data line.
    data      : byte_array(0 to 7);

    -- Asserted to mark the last line of a chunk. When asserted, endi indicates
    -- the index of the last valid byte. endi must be 7 otherwise.
    last      : std_logic;
    endi      : unsigned(2 downto 0);

  end record;

  constant COMPRESSED_STREAM_SINGLE_INIT : compressed_stream_single := (
    valid     => '0',
    data      => (others => (others => UNDEF)),
    last      => UNDEF,
    endi      => (others => UNDEF)
  );

  procedure stream_des(l: inout line; value: out compressed_stream_single; to_x: boolean);

  -- Preprocessed compressed data stream, including information to skip over
  -- the uncompressed length field, and including a second "lookahead" line to
  -- ensure that we never have to stall in the middle of decoding an element.
  type compressed_stream_double is record

    -- Stream valid signal.
    valid     : std_logic;

    -- Two lines of compressed data. Reading elements that *start* after the
    -- first line is not legal, because not all of the lookahead line may be
    -- valid. However, if an element starts at byte 7, as much of the second
    -- line as is needed to encode the element should be valid, assuming that
    -- the input is valid snappy data.
    data      : byte_array(0 to 15);

    -- Asserted to mark the first line of a chunk. When asserted, start
    -- indicates the byte index of the first element; start should be ignored
    -- otherwise.
    first     : std_logic;
    start     : unsigned(1 downto 0);

    -- Asserted to mark the last line of a chunk. When asserted, endi indicates
    -- the index of the last valid byte. endi must be 7 otherwise.
    last      : std_logic;
    endi      : unsigned(2 downto 0);

  end record;

  constant COMPRESSED_STREAM_DOUBLE_INIT : compressed_stream_double := (
    valid     => '0',
    data      => (others => (others => UNDEF)),
    first     => UNDEF,
    start     => (others => UNDEF),
    last      => UNDEF,
    endi      => (others => UNDEF)
  );

  procedure stream_des(l: inout line; value: out compressed_stream_double; to_x: boolean);

  -- Compressed data stream preprocessor.
  component vhsnunzip_pre_decoder is
    port (
      clk         : in  std_logic;
      reset       : in  std_logic;
      cs          : in  compressed_stream_single;
      cs_ready    : out std_logic;
      cd          : out compressed_stream_double;
      cd_ready    : in  std_logic
    );
  end component;

  -- Snappy element information and literal data stream from the decoder to
  -- the command generator. Each transfer in this stream encompasses (in this
  -- order) zero or one copy elements, zero or one literal headers, and
  -- optionally literal data. After a transfer with li_valid set, transfers
  -- with cp_valid and li_valid low will follow until all literal data bytes
  -- have been in the first 8 bytes of li_data (for short literals that start
  -- at a low offset, there may be zero such transfers).
  type element_stream is record

    -- Stream valid signal.
    valid     : std_logic;

    -- Copy element information. cp_offs is the byte offset and cp_len is the
    -- length as encoded by the element header. cp_len is stored
    -- DIMINISHED-ONE, just like the value in the Snappy header (this saves a
    -- bit).
    cp_val    : std_logic;
    cp_off    : unsigned(15 downto 0);
    cp_len    : unsigned(5 downto 0);

    -- Literal element information. li_offs is the starting byte offset within
    -- li_data for the literal; li_len encodes the literal length. li_len is
    -- stored DIMINISHED-ONE, just like the value in the Snappy header (this
    -- saves a bit).
    li_val    : std_logic;
    li_off    : unsigned(3 downto 0);
    li_len    : unsigned(15 downto 0);

    -- Indicates that the literal data FIFO should be popped after this stream
    -- transfer has been handled.
    ld_pop    : std_logic;

    -- Indicator for last set of elements/literal data in chunk.
    last      : std_logic;

  end record;

  procedure stream_des(l: inout line; value: out element_stream; to_x: boolean);

  constant ELEMENT_STREAM_INIT : element_stream := (
    valid     => '0',
    cp_val    => UNDEF,
    cp_off    => (others => UNDEF),
    cp_len    => (others => UNDEF),
    li_val    => UNDEF,
    li_off    => (others => UNDEF),
    li_len    => (others => UNDEF),
    ld_pop    => UNDEF,
    last      => UNDEF
  );

  -- Snappy element decoder.
  component vhsnunzip_decoder is
    port (
      clk         : in  std_logic;
      reset       : in  std_logic;
      cd          : in  compressed_stream_double;
      cd_ready    : out std_logic;
      el          : out element_stream;
      el_ready    : in  std_logic
    );
  end component;

  -- Command stream for the datapath.
  type command_stream is record

    -- Stream valid signal.
    valid     : std_logic;

    -- Whether long-term memory should be read.
    lt_val    : std_logic;

    -- Absolute first linepair indices for long-term memory read (0 = first
    -- linepair that was written, 1 = second linepair that was written, etc.).
    -- Both an even and an odd line must be read, with independent addresses.
    -- They'll always be next to each other, but the even line index may be
    -- one later to read a "misaligned" line pair.
    lt_adev   : unsigned(11 downto 0);
    lt_adod   : unsigned(11 downto 0);

    -- When low, lt_adev is the address for the low line, and lt_adod is the
    -- address for the high line. When high, this is swapped.
    lt_swap   : std_logic;

    -- Relative first line index for short-term memory read (-1 = line we're
    -- currently writing; positive = further back). For each of the 8 bytes,
    -- either the given line index must be read, or the subsequent line,
    -- depending on the rotation. This is computed by the datapath to reduce
    -- FIFO usage.
    st_addr   : unsigned(4 downto 0);

    -- Desired rotation for normal copies, or byte index for run-length copies.
    -- That is:
    --
    --  - cp_rle = '0': the copy mux result should be the first 8 bytes of
    --    linepair <<> cp_rol (where <<> denotes rotate-left).
    --  - cp_rle = '1': the copy mux result should be linepair(cp_rol(2..0))
    --    for each byte.
    --
    -- The linepair is:
    --
    --  - lt_val = '1', lt_swp = '0': lt_even & lt_odd
    --  - lt_val = '1', lt_swp = '1': lt_odd & lt_odd
    --  - lt_val = '0': short_term(st_addr) & short_term(st_addr+1)
    --
    -- For both long-term and short-term copies, an 8:8 rotator is sufficient,
    -- if:
    --
    --  - the short-term read address is determined on a byte-by-byte basis
    --    based on cp_rol;
    --  - the effect of lt_swap is inverted on a byte-by-byte basis based on
    --    cp_rol.
    --
    cp_rol    : unsigned(3 downto 0);

    -- Run-length encoding acceleration flag for rotations. When set, the
    -- constant (0, 1, 2, 3, 4, 5, 6, 7) should be added to cp_rol before the
    -- main rotation is applied. Carry into bit 3 can be ignored; the high
    -- line is never actually used in the main rotator because of this number
    -- and the fact that the decoder only outputs rotations between 0 and 7
    -- when this is asserted. This all sounds a bit arcane, but what ultimately
    -- happens because of this is simple: cp_rol is reduced to a byte index
    -- within the two lines.
    cp_rle    : std_logic;

    -- This index indicates the last valid *copy* byte provided by this command
    -- + one. Bytes between cp_endi and endi are literal bytes. The copy
    -- selection signals can be decoded from this in the same way that the 
    -- byte strobe signals are determined from endi.
    cp_end    : unsigned(3 downto 0);

    -- Rotation for literals. The direction is rotate-left. The MSB should be
    -- handled by offsetting the SRL literal read by one line on a byte-by-byte
    -- basis, in the same way that the short-term memory read handles this. The
    -- remaining 3 LSBs must be handled by the main 8:8 rotator.
    li_rol    : unsigned(3 downto 0);

    -- Index of the last valid byte provided by this command + one. The byte
    -- strobe signals can be derived from this thermometer-code style, ignoring
    -- any bytes that were already written. Overflow past the current line
    -- (endi > 8) should be written to a holding register, as the beginning for
    -- the next line. The MSB therefore indicates that an aligned line of
    -- decompressed data is complete.
    li_end    : unsigned(3 downto 0);

    -- Indicates that the literal data FIFO should be popped after this command
    -- has been handled.
    ld_pop    : std_logic;

    -- Set to mark the last command for a chunk.
    last      : std_logic;

  end record;

  constant COMMAND_STREAM_INIT : command_stream := (
    valid     => '0',
    lt_val    => UNDEF,
    lt_adev   => (others => UNDEF),
    lt_adod   => (others => UNDEF),
    lt_swap   => UNDEF,
    st_addr   => (others => UNDEF),
    cp_rol    => (others => UNDEF),
    cp_rle    => UNDEF,
    cp_end    => (others => UNDEF),
    li_rol    => (others => UNDEF),
    li_end    => (others => UNDEF),
    ld_pop    => UNDEF,
    last      => UNDEF
  );

end package vhsnunzip_pkg;

package body vhsnunzip_pkg is

  function undef_fn return std_logic is
    variable retval : std_logic := '0';
  begin
    -- pragma translate_off
    retval := 'U';
    -- pragma translate_on
    return retval;
  end function;

  procedure stream_des(l: inout line; value: out compressed_stream_single; to_x: boolean) is
  begin
    for i in value.data'range loop
      read(l, value.data(i));
      if to_x then
        value.data(i) := to_x01(value.data(i));
      end if;
    end loop;
    read(l, value.last);
    if to_x then
      value.last := to_x01(value.last);
    end if;
    read(l, value.endi);
    if to_x then
      value.endi := to_x01(value.endi);
    end if;
    value.valid := '1';
  end procedure;

  procedure stream_des(l: inout line; value: out compressed_stream_double; to_x: boolean) is
  begin
    for i in value.data'range loop
      read(l, value.data(i));
      if to_x then
        value.data(i) := to_x01(value.data(i));
      end if;
    end loop;
    read(l, value.first);
    if to_x then
      value.first := to_x01(value.first);
    end if;
    read(l, value.start);
    if to_x then
      value.start := to_x01(value.start);
    end if;
    read(l, value.last);
    if to_x then
      value.last := to_x01(value.last);
    end if;
    read(l, value.endi);
    if to_x then
      value.endi := to_x01(value.endi);
    end if;
    value.valid := '1';
  end procedure;

  procedure stream_des(l: inout line; value: out element_stream; to_x: boolean) is
  begin
    read(l, value.cp_val);
    if to_x then
      value.cp_val := to_x01(value.cp_val);
    end if;
    read(l, value.cp_off);
    if to_x then
      value.cp_off := to_x01(value.cp_off);
    end if;
    read(l, value.cp_len);
    if to_x then
      value.cp_len := to_x01(value.cp_len);
    end if;
    read(l, value.li_val);
    if to_x then
      value.li_val := to_x01(value.li_val);
    end if;
    read(l, value.li_off);
    if to_x then
      value.li_off := to_x01(value.li_off);
    end if;
    read(l, value.li_len);
    if to_x then
      value.li_len := to_x01(value.li_len);
    end if;
    read(l, value.ld_pop);
    if to_x then
      value.ld_pop := to_x01(value.ld_pop);
    end if;
    read(l, value.last);
    if to_x then
      value.last := to_x01(value.last);
    end if;
    value.valid := '1';
  end procedure;

end package body vhsnunzip_pkg;
