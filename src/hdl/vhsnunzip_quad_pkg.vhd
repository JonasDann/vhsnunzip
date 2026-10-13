library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.vhsnunzip_int_pkg.all;

-- Declarations for the 32-byte, 4-slot ("quad", R2-lite) datapath. This core
-- is independent of C_BYTES: its line width is fixed at QB = 32 bytes.
--
-- Pipeline overview:
--
--   input FIFO -> decoder (4 elements/cycle, two-line window)
--              -> element FIFO
--              -> group former (G2): up to 4 slots and 32 output bytes per cycle
--              -> R0 (long-term reads are issued here)
--              -> G3 (copy-source resolution)
--              -> S1a/S1b (per-lane control)
--              -> S2a (memory reads, rotators, merge)
--              -> S2b (recent-history/replicate/stride fill, history write)
--              -> output FIFO (+ long-term write queues)
--
-- An "element" is one Snappy copy plus the literal directly following it, a
-- literal alone, or the end-of-chunk marker.
package vhsnunzip_quad_pkg is

  -- Line width in bytes, lane index width.
  constant QB  : natural := 32;
  constant QI  : natural := 5;

  -- Number of element slots per cycle.
  constant QK  : natural := 4;

  -- Simulation trace output (cycles 0 .. QDEBUG-1).
  constant QDEBUG : natural := 0;

  -- Decoded element.
  type qel_type is record
    cp    : unsigned(6 downto 0);   -- copy length (0 = no copy, 1..64)
    off   : unsigned(15 downto 0);  -- copy offset
    li    : unsigned(16 downto 0);  -- literal length (0 = no literal)
    q     : unsigned(19 downto 0);  -- compressed-stream position of the literal data
    last  : std_logic;              -- end-of-chunk marker (cp = li = 0)
    tot   : unsigned(5 downto 0);   -- min(cp + li, 63)
    pw2   : std_logic;              -- off in {1, 2, 4, 8, 16}
    s4    : std_logic;              -- off = 4
    s8    : std_logic;              -- off = 8
  end record;

  constant QEL_ZERO : qel_type := (
    cp => (others => '0'), off => (others => '0'), li => (others => '0'),
    q => (others => '0'), last => '0', tot => (others => '0'),
    pw2 => '0', s4 => '0', s8 => '0');

  type qel_array is array (natural range <>) of qel_type;

  constant QEL_BITS : natural := 7 + 16 + 17 + 20 + 1 + 6 + 3;

  function qel_pack(e : qel_type) return std_logic_vector;
  function qel_unpack(v : std_logic_vector(QEL_BITS-1 downto 0)) return qel_type;

  -- Fill in the derived fields of an element.
  function qel_derive(e : qel_type) return qel_type;

  -- Copy kinds of a slot.
  constant CK_NONE   : std_logic_vector(2 downto 0) := "000";
  constant CK_HIST   : std_logic_vector(2 downto 0) := "001";  -- source before the group (ST, LT or recent)
  constant CK_CSR    : std_logic_vector(2 downto 0) := "010";  -- source inside the group (resolved in G3)
  constant CK_STRIDE : std_logic_vector(2 downto 0) := "011";  -- out[x] = out[x - off], off in {4, 8}
  constant CK_REP    : std_logic_vector(2 downto 0) := "100";  -- slot 0, power-of-two self-overlapping copy

  -- One slot of a group, as produced by the group former.
  type qslot_type is record
    act   : std_logic;              -- slot is used
    pos   : unsigned(5 downto 0);   -- start offset of the slot within the group (0..31)
    c     : unsigned(5 downto 0);   -- copy piece length (0..32)
    l     : unsigned(5 downto 0);   -- literal piece length (0..32)
    ck    : std_logic_vector(2 downto 0);
    off   : unsigned(15 downto 0);
    src   : unsigned(15 downto 0);  -- copy source (start - off)
    q     : unsigned(15 downto 0);  -- compressed position of the literal piece
    lt    : std_logic;              -- copy source in long-term memory (CK_HIST only)
    csrj  : unsigned(1 downto 0);   -- CSR: slot containing the source
    csrl  : std_logic;              -- CSR: source in that slot's literal part
  end record;

  constant QSLOT_ZERO : qslot_type := (
    act => '0', pos => (others => '0'), c => (others => '0'), l => (others => '0'),
    ck => CK_NONE, off => (others => '0'), src => (others => '0'), q => (others => '0'), lt => '0',
    csrj => (others => '0'), csrl => '0');

  type qslot_array is array (natural range <>) of qslot_type;

  -- Long-term read port bundle: index 0 = even bank port B, 1 = even bank
  -- port A, 2 = odd bank port B, 3 = odd bank port A.
  type lt_addr_array is array (natural range <>) of unsigned(9 downto 0);

  -- A group: what the datapath does in one cycle.
  type qgroup_type is record
    valid : std_logic;
    p0    : unsigned(15 downto 0);  -- output position of the first byte
    n     : unsigned(5 downto 0);   -- number of output bytes (0..32)
    last  : std_logic;              -- last group of the chunk
    lnext : unsigned(15 downto 0);  -- next literal byte still needed after this group
    rd_v  : std_logic_vector(3 downto 0);
    rd_a  : lt_addr_array(0 to 3);
    s     : qslot_array(0 to QK-1);
  end record;

  constant QGROUP_ZERO : qgroup_type := (
    valid => '0', p0 => (others => '0'), n => (others => '0'), last => '0',
    lnext => (others => '0'), rd_v => (others => '0'), rd_a => (others => (others => '0')),
    s => (others => QSLOT_ZERO));

  -- Resolved copy source of a slot (G3 output).
  constant RK_NONE   : std_logic_vector(2 downto 0) := "000";
  constant RK_ST     : std_logic_vector(2 downto 0) := "001";  -- history, short-term memory (or recent)
  constant RK_LT     : std_logic_vector(2 downto 0) := "010";  -- history, long-term memory
  constant RK_LIT    : std_logic_vector(2 downto 0) := "011";  -- literal vector of slot rsl
  constant RK_STRIDE : std_logic_vector(2 downto 0) := "100";
  constant RK_REP    : std_logic_vector(2 downto 0) := "101";

  type qres_type is record
    rk    : std_logic_vector(2 downto 0);
    src   : unsigned(15 downto 0);  -- history position, or compressed position for RK_LIT
    rsl   : unsigned(1 downto 0);   -- RK_LIT: slot whose literal vector holds the source
    per   : unsigned(4 downto 0);   -- RK_REP: period (power of two <= 16); src = p0 - per
    sh    : unsigned(4 downto 0);   -- RK_REP: phase of the first byte within the period
  end record;

  type qres_array is array (natural range <>) of qres_type;

  type u5_array is array (natural range <>) of unsigned(4 downto 0);
  type u2_array is array (natural range <>) of unsigned(1 downto 0);
  type u3_array is array (natural range <>) of unsigned(2 downto 0);
  type u4_array is array (natural range <>) of unsigned(3 downto 0);
  type u6_array is array (natural range <>) of unsigned(5 downto 0);
  type u7_array is array (natural range <>) of unsigned(6 downto 0);
  type u16_array is array (natural range <>) of unsigned(15 downto 0);
  type ck_array is array (natural range <>) of std_logic_vector(2 downto 0);
  type u5_matrix is array (natural range <>) of u5_array(0 to QB-1);
  type u2_matrix is array (natural range <>) of u2_array(0 to QB-1);
  type u3_matrix is array (natural range <>) of u3_array(0 to QB-1);
  type byte_matrix is array (natural range <>) of byte_array(0 to QB-1);

  -- Rotate left: result(x) = v((x + r) mod QB).
  function rotl(v : byte_array(0 to QB-1); r : unsigned(QI-1 downto 0)) return byte_array;

  component vhsnunzip_quad_decoder is
    port (
      clk         : in  std_logic;
      reset       : in  std_logic;
      cs_valid    : in  std_logic;
      cs_ready    : out std_logic;
      cs_data     : in  byte_array(0 to QB-1);
      cs_last     : in  std_logic;
      cs_endi     : in  unsigned(QI-1 downto 0);
      lb_we       : out std_logic;
      lb_line     : out unsigned(QI-1 downto 0);
      lb_data     : out byte_array(0 to QB-1);
      lb_wpos     : out unsigned(19 downto 0);
      cons_pos    : in  unsigned(15 downto 0);
      el          : out qel_array(0 to QK-1);
      el_cnt      : out unsigned(2 downto 0);
      el_space    : in  std_logic
    );
  end component;

  component vhsnunzip_quad_group is
    port (
      clk         : in  std_logic;
      reset       : in  std_logic;
      el          : in  qel_array(0 to QK-1);
      el_cnt      : in  unsigned(2 downto 0);
      el_space    : out std_logic;
      lb_wpos     : in  unsigned(19 downto 0);
      wq_resv     : in  std_logic_vector(1 downto 0);
      stall       : in  std_logic;
      gr          : out qgroup_type
    );
  end component;

  component vhsnunzip_quad_datapath is
    port (
      clk         : in  std_logic;
      reset       : in  std_logic;
      gr          : in  qgroup_type;
      lb_we       : in  std_logic;
      lb_line     : in  unsigned(QI-1 downto 0);
      lb_data     : in  byte_array(0 to QB-1);
      cons_pos    : out unsigned(15 downto 0);
      lt_rdat     : in  byte_matrix(0 to 3);
      ln_valid    : out std_logic;
      ln_data     : out byte_array(0 to QB-1);
      ln_line     : out unsigned(10 downto 0);
      out_valid   : out std_logic;
      out_pvalid  : out std_logic;
      out_pdata   : out byte_array(0 to QB-1);
      out_pcnt    : out unsigned(QI downto 0);
      out_plast   : out std_logic
    );
  end component;

  component vhsnunzip_quad_lt_mem is
    port (
      clk         : in  std_logic;
      reset       : in  std_logic;
      rd_v        : in  std_logic_vector(3 downto 0);
      rd_a        : in  lt_addr_array(0 to 3);
      rdat        : out byte_matrix(0 to 3);
      wr_valid    : in  std_logic;
      wr_line     : in  unsigned(10 downto 0);
      wr_data     : in  byte_array(0 to QB-1);
      wq_resv     : out std_logic_vector(1 downto 0)
    );
  end component;

end package vhsnunzip_quad_pkg;

package body vhsnunzip_quad_pkg is

  function qel_pack(e : qel_type) return std_logic_vector is
  begin
    return std_logic_vector(e.cp) & std_logic_vector(e.off) & std_logic_vector(e.li)
         & std_logic_vector(e.q) & e.last & std_logic_vector(e.tot) & e.pw2 & e.s4 & e.s8;
  end function;

  function qel_unpack(v : std_logic_vector(QEL_BITS-1 downto 0)) return qel_type is
    variable e : qel_type;
    variable i : natural := QEL_BITS;
  begin
    e.cp   := unsigned(v(i-1 downto i-7));  i := i - 7;
    e.off  := unsigned(v(i-1 downto i-16)); i := i - 16;
    e.li   := unsigned(v(i-1 downto i-17)); i := i - 17;
    e.q    := unsigned(v(i-1 downto i-20)); i := i - 20;
    e.last := v(i-1);                       i := i - 1;
    e.tot  := unsigned(v(i-1 downto i-6));  i := i - 6;
    e.pw2  := v(2);
    e.s4   := v(1);
    e.s8   := v(0);
    return e;
  end function;

  function qel_derive(e : qel_type) return qel_type is
    variable r : qel_type := e;
    variable t : unsigned(7 downto 0);
  begin
    if e.li(16 downto 6) /= 0 then
      r.tot := (others => '1');
    else
      t := resize(e.cp, 8) + resize(e.li(5 downto 0), 8);
      if t > 63 then
        r.tot := (others => '1');
      else
        r.tot := t(5 downto 0);
      end if;
    end if;
    r.pw2 := '0';
    r.s4 := '0';
    r.s8 := '0';
    if e.cp /= 0 then
      case to_integer(e.off) is
        when 1 | 2 | 16 => r.pw2 := '1';
        when 4 => r.pw2 := '1'; r.s4 := '1';
        when 8 => r.pw2 := '1'; r.s8 := '1';
        when others => null;
      end case;
    end if;
    return r;
  end function;

  -- Rotation as a log shifter (4:1, 4:1, 2:1 stages), which is much smaller
  -- than a 32:1 multiplexer per output byte.
  function rotl(v : byte_array(0 to QB-1); r : unsigned(QI-1 downto 0)) return byte_array is
    variable a, b, c : byte_array(0 to QB-1);
  begin
    for x in 0 to QB-1 loop
      a(x) := v((x + to_integer(r(1 downto 0))) mod QB);
    end loop;
    for x in 0 to QB-1 loop
      b(x) := a((x + 4 * to_integer(r(3 downto 2))) mod QB);
    end loop;
    for x in 0 to QB-1 loop
      if r(4) = '1' then
        c(x) := b((x + 16) mod QB);
      else
        c(x) := b(x);
      end if;
    end loop;
    return c;
  end function;

end package body vhsnunzip_quad_pkg;
