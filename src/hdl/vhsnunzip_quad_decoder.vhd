library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.vhsnunzip_int_pkg.all;
use work.vhsnunzip_quad_pkg.all;

-- Snappy element decoder for the quad datapath: up to 4 elements per cycle.
--
-- Every compressed line passes through an input pipeline. Stage A decodes an
-- element at each of its 32 byte positions (with lookahead into the next
-- line for headers that cross the line end, and for the first 12 positions
-- of the next line): whether an element can start there, whether it is
-- "small" (total compressed size at most 12 bytes, so the next element start
-- is reachable by a table lookup), its size and its fields. Stage B turns
-- this into the line's walk table: for every position, the step to the next
-- element start, the two-step position s2, and the outcome of emitting one
-- or two elements from there, both for when the line is the first and when
-- it is the second line of the decoder window. Walk tables and fields queue
-- up in front of the decoder window.
--
-- The window covers two lines. The walk tables of the queued lines are kept
-- in a register file of four lines that the walk reads directly (the head
-- may move one cycle late, so three of them are in reach); the loop-carried
-- path is only the walk from one element start to the next:
-- e1 = step(e0), e2 = s2(e0), e3 = step(e2), e4 = s2(e2), i.e. two serial
-- table lookups.
--
-- Element emission rules (these match the throughput model):
--  - continue past an element only if it is small;
--  - a non-small ("big") element may only be emitted in slot 0 or 1; it ends
--    the cycle and the next start is computed from its full size;
--  - the window advances by at most one line per cycle.
entity vhsnunzip_quad_decoder is
  port (
    clk         : in  std_logic;
    reset       : in  std_logic;

    -- Compressed line stream. Lines are normalized; cs_endi is the index of
    -- the last valid byte when cs_last is set.
    cs_valid    : in  std_logic;
    cs_ready    : out std_logic;
    cs_data     : in  byte_array(0 to QB-1);
    cs_last     : in  std_logic;
    cs_endi     : in  unsigned(QI-1 downto 0);

    -- Literal buffer write port. Every compressed line is written here as it
    -- enters the decoder; lb_wpos is the compressed position up to which data
    -- has been written. cons_pos is the oldest literal byte that is still
    -- needed; lines are only accepted while the buffer has room.
    lb_we       : out std_logic;
    lb_line     : out unsigned(QI-1 downto 0);
    lb_data     : out byte_array(0 to QB-1);
    lb_wpos     : out unsigned(19 downto 0);
    cons_pos    : in  unsigned(15 downto 0);

    -- Decoded elements: el(0 .. el_cnt-1) are valid. el_space must be high
    -- for the decoder to emit anything (room for 4 elements downstream).
    el          : out qel_array(0 to QK-1);
    el_cnt      : out unsigned(2 downto 0);
    el_space    : in  std_logic
  );
end vhsnunzip_quad_decoder;

architecture behavior of vhsnunzip_quad_decoder is

  type line_type is record
    data  : byte_array(0 to QB-1);
    last  : std_logic;
    endi  : unsigned(QI-1 downto 0);
    lc    : unsigned(14 downto 0);
  end record;

  constant LINE_ZERO : line_type := (
    data => (others => X"00"), last => '0', endi => (others => '0'),
    lc => (others => '0'));

  -- Line table entry: element starting at this byte of the line.
  type pent_type is record
    ok    : std_logic;              -- element (or end marker) starts here
    sm    : std_logic;              -- small
    endm  : std_logic;              -- end-of-chunk marker
    sz4   : unsigned(3 downto 0);   -- size when small
    hend  : unsigned(5 downto 0);   -- end of the header, relative to the line
    bt    : unsigned(17 downto 0);  -- end of the element (hend + literal
                                    -- length), relative to the line
  end record;

  constant PENT_ZERO : pent_type := (
    ok => '0', sm => '0', endm => '0', sz4 => (others => '0'), hend => (others => '0'),
    bt => (others => '0'));

  type pent_array is array (natural range <>) of pent_type;

  -- Lookahead entry (first positions of the next line).
  type lent_type is record
    ok    : std_logic;
    sm    : std_logic;
    endm  : std_logic;
    sz4   : unsigned(3 downto 0);
  end record;

  type lent_array is array (natural range <>) of lent_type;

  type ptab_type is record
    e     : pent_array(0 to QB-1);
    w     : byte_array(0 to QB+1);      -- the line and two bytes of the next
    la    : lent_array(0 to 11);
    last  : std_logic;
    endi  : unsigned(QI-1 downto 0);
    lc    : unsigned(14 downto 0);
    vlen  : unsigned(2 downto 0);   -- varint length if the line starts a chunk
  end record;

  -- Outcome of the first half of the walk (slots 0, 1) from an element
  -- start: number of elements emitted, whether the walk continues with slot
  -- 2, whether it ends with an end marker, and where the next start comes
  -- from (0: unchanged, 1: bt of slot 0, 2: slot 1 start, 3: bt of slot 1).
  type kinda_type is record
    n     : unsigned(1 downto 0);
    cont  : std_logic;
    endc  : std_logic;
    sel   : unsigned(1 downto 0);
  end record;

  -- Outcome of the second half (slots 2, 3) from the slot 2 start; sel: 0:
  -- slot 2 start, 1: slot 3 start, 2: the start after slot 3.
  type kindb_type is record
    n     : unsigned(1 downto 0);
    endc  : std_logic;
    sel   : unsigned(1 downto 0);
  end record;

  constant KA_ZERO : kinda_type := ("00", '0', '0', "00");
  constant KB_ZERO : kindb_type := ("00", '0', "00");

  -- Indexed by the window line the entry is in (0: first, 1: second).
  type kindb_array is array (0 to 1) of kindb_type;

  -- What the first half of the walk depends on (kinda follows from it after
  -- the lookup): this element starts here (ok), is small (sm), is an end
  -- marker (en); the next one is in the window when this one is in window
  -- line 0 or 1 (fok), small (fsm), an end marker (fen); the start after it
  -- is an end marker at the end of the line (sp).
  type kflags_type is record
    ok    : std_logic;
    sm    : std_logic;
    en    : std_logic;
    fok   : std_logic_vector(0 to 1);
    fsm   : std_logic;
    fen   : std_logic;
    sp    : std_logic;
  end record;

  constant KF_ZERO : kflags_type := ('0', '0', '0', "00", '0', '0', '0');

  function kinda_of(f : kflags_type; wl : integer range 0 to 1) return kinda_type is
    variable ok : std_logic;
  begin
    ok := f.ok;
    if wl = 1 and f.en = '1' then
      ok := '0';
    end if;
    if ok = '0' then
      return KA_ZERO;
    elsif f.sm = '0' then
      return ("01", '0', f.en, "01");
    elsif f.fok(wl) = '0' then
      return ("01", '0', '0', "10");
    elsif f.fsm = '0' then
      return ("10", '0', f.fen, "11");
    elsif wl = 0 and f.sp = '1' then
      -- The start after slot 1 is the end marker at the line end.
      return ("11", '0', '1', "00");
    else
      return ("10", '1', '0', "00");
    end if;
  end function;

  -- Walk table entry. Positions are relative to the entry's line.
  type went_type is record
    kf    : kflags_type;
    kb    : kindb_array;
    step  : unsigned(5 downto 0);   -- next element start (if small)
    s2    : unsigned(5 downto 0);   -- start after that one
    bt    : unsigned(17 downto 0);  -- next element start (any size)
    btge  : std_logic;              -- bt >= 32
  end record;

  constant WENT_ZERO : went_type := (
    kf => KF_ZERO, kb => (others => KB_ZERO),
    step => (others => '0'), s2 => (others => '0'), bt => (others => '0'), btge => '0');

  type went_array is array (natural range <>) of went_type;

  type wline_type is record
    t     : went_array(0 to QB-1);
    last  : std_logic;
    ez    : std_logic;              -- last, and the chunk ends at the line end
    vlen  : unsigned(2 downto 0);
  end record;

  type wline_array is array (natural range <>) of wline_type;

  -- What field extraction needs of a line: its bytes (and two of the next
  -- line) for the copy tag and offset, and per position the header length
  -- (the literal length follows from the element end in the walk table).
  type u4_hl_array is array (natural range <>) of unsigned(3 downto 0);

  type fline_type is record
    w     : byte_array(0 to QB+1);
    hl    : u4_hl_array(0 to QB-1);
    lc    : unsigned(14 downto 0);
  end record;

  type fline_array is array (natural range <>) of fline_type;

  type u6_arr is array (natural range <>) of unsigned(5 downto 0);
  type u4_arr is array (natural range <>) of unsigned(3 downto 0);

  -- Stage A: decodes the elements starting in line x and the first 12
  -- positions of the next line y (yv: valid and in the same chunk).
  function line_table(x : line_type; y : line_type; yv : std_logic) return ptab_type is
    variable w      : byte_array(0 to 63);
    variable e      : integer range 0 to 127;
    type int_array is array (natural range <>) of integer;
    variable islit  : std_logic_vector(0 to 48);
    variable lh     : int_array(0 to 39);
    type u17_array is array (natural range <>) of unsigned(16 downto 0);
    variable lm1    : u17_array(0 to 39);
    variable lacc   : unsigned(16 downto 0);
    variable fld    : unsigned(5 downto 0);
    variable nb     : integer range 0 to 4;
    variable tag    : std_logic_vector(1 downto 0);
    variable clen   : integer range 0 to 5;
    variable m      : integer range 0 to 63;
    variable haslit : std_logic;
    variable hend   : integer range 0 to 63;
    variable lfld   : unsigned(5 downto 0);
    variable ok     : std_logic;
    variable sm     : std_logic;
    variable endm   : std_logic;
    variable sz4    : unsigned(3 downto 0);
    variable r      : ptab_type;
  begin
    -- Bytes of the next line are used even when it isn't part of this
    -- chunk (or not there): only positions past the chunk end see them, and
    -- their entries are never used.
    for i in 0 to QB-1 loop
      w(i) := x.data(i);
      w(QB + i) := y.data(i);
    end loop;

    if x.last = '1' then
      e := to_integer(x.endi) + 1;
    elsif yv = '1' and y.last = '1' then
      e := QB + to_integer(y.endi) + 1;
    else
      e := 127;
    end if;

    -- Literal headers (length - 1).
    for i in 0 to 48 loop
      islit(i) := not w(i)(1) and not w(i)(0);
    end loop;
    for i in 0 to 39 loop
      fld := unsigned(w(i)(7 downto 2));
      if fld < 60 then
        lh(i) := 1;
        lm1(i) := resize(fld, 17);
      else
        nb := to_integer(fld(1 downto 0)) + 1;
        lh(i) := 1 + nb;
        lacc := (others => '0');
        for b in 0 to 2 loop
          if b < nb then
            for k in 0 to 7 loop
              if b*8 + k < 17 then
                lacc(b*8 + k) := w(i + 1 + b)(k);
              end if;
            end loop;
          end if;
        end loop;
        lm1(i) := lacc;
      end if;
    end loop;

    -- Elements (positions 32 .. 43: lookahead into the next line).
    for j in 0 to 43 loop
      tag := w(j)(1 downto 0);
      case tag is
        when "01" => clen := 2;
        when "10" => clen := 3;
        when "11" => clen := 5;
        when others => clen := 0;
      end case;
      m := j + clen;
      if tag = "00" then
        haslit := '1';
      elsif m >= e then
        haslit := '0';
      else
        haslit := islit(m);
      end if;
      -- Small: total size at most 12 bytes, i.e. a one-byte literal header
      -- (fld < 60) with clen + fld + 2 <= 12. Longer non-canonical encodings
      -- of short literals are treated as big, which is just slower.
      lfld := unsigned(w(m)(7 downto 2));
      if j < e then
        ok := '1';
      else
        ok := '0';
      end if;
      if haslit = '1' then
        sz4 := to_unsigned(clen + 2, 4) + lfld(3 downto 0);
        if resize(lfld, 7) + clen <= 10 then
          sm := ok;
        else
          sm := '0';
        end if;
      else
        sz4 := to_unsigned(clen, 4);
        sm := ok;
      end if;
      endm := '0';
      if j = e and (x.last = '1' or (yv = '1' and y.last = '1')) then
        ok := '1';
        sm := '0';
        endm := '1';
      end if;
      if j < QB then
        r.e(j).ok := ok;
        r.e(j).sm := sm;
        r.e(j).endm := endm;
        r.e(j).sz4 := sz4;
        if haslit = '1' then
          hend := m + lh(m);
          r.e(j).bt := to_unsigned(hend, 18) + resize(lm1(m), 18) + 1;
        else
          hend := m;
          r.e(j).bt := to_unsigned(hend, 18);
        end if;
        r.e(j).hend := to_unsigned(hend, 6);
        if endm = '1' then
          r.e(j).hend := to_unsigned(j, 6);
          r.e(j).bt := to_unsigned(j, 18);
        end if;
      else
        r.la(j - QB).ok := ok;
        r.la(j - QB).sm := sm;
        r.la(j - QB).endm := endm;
        r.la(j - QB).sz4 := sz4;
        if x.last = '1' and j > e then
          r.la(j - QB).ok := '0';
        end if;
      end if;
    end loop;

    r.w := w(0 to QB+1);
    r.last := x.last;
    r.endi := x.endi;
    r.lc := x.lc;
    r.vlen := "101";
    for i in 4 downto 0 loop
      if w(i)(7) = '0' then
        r.vlen := to_unsigned(i + 1, 3);
      end if;
    end loop;
    return r;
  end function;

  -- Stage B: walk table of a line. An entry can be emitted in window line
  -- wl: anywhere in the first line, and in the second line unless it is an
  -- end marker (an end marker at a line start belongs to the line before it;
  -- the walk handles it). The element after it is in the window if it starts in
  -- the first two lines (wl = 0) or in the same line (wl = 1); an end marker
  -- there must be at most at the start of the second window line.
  function line_walk(t : ptab_type) return wline_type is
    variable okx  : std_logic_vector(0 to 43);
    variable smx  : std_logic_vector(0 to 43);
    variable enx  : std_logic_vector(0 to 43);
    variable szx  : u4_arr(0 to 43);
    variable stx  : u6_arr(0 to 43);
    variable ok   : std_logic_vector(0 to 1);
    variable fok  : std_logic_vector(0 to 1);
    variable fsm  : std_logic;
    variable fen  : std_logic;
    variable kb   : kindb_type;
    variable r    : wline_type;
  begin
    for j in 0 to 43 loop
      if j < QB then
        okx(j) := t.e(j).ok; smx(j) := t.e(j).sm; enx(j) := t.e(j).endm; szx(j) := t.e(j).sz4;
      else
        okx(j) := t.la(j - QB).ok; smx(j) := t.la(j - QB).sm; enx(j) := t.la(j - QB).endm; szx(j) := t.la(j - QB).sz4;
      end if;
      stx(j) := to_unsigned(j, 6) + resize(szx(j), 6);
    end loop;
    for j in 0 to QB-1 loop
      r.t(j).step := stx(j);
      r.t(j).s2 := stx(j);
      r.t(j).bt := t.e(j).bt;
      if t.e(j).bt(17 downto 5) /= 0 then
        r.t(j).btge := '1';
      else
        r.t(j).btge := '0';
      end if;
      -- The element after this one (if this one is small).
      fok := "00";
      fsm := '0';
      fen := '0';
      for d in 2 to 12 loop
        if szx(j) = d then
          if enx(j + d) = '0' or j + d <= QB then
            fok(0) := okx(j + d);
          end if;
          if enx(j + d) = '0' and j + d < QB then
            fok(1) := okx(j + d);
          end if;
          fsm := smx(j + d);
          fen := enx(j + d);
          r.t(j).s2 := stx(j + d);
        end if;
      end loop;
      ok(0) := okx(j);
      ok(1) := okx(j) and not enx(j);
      r.t(j).kf.ok := okx(j);
      r.t(j).kf.sm := smx(j);
      r.t(j).kf.en := enx(j);
      r.t(j).kf.fok := fok;
      r.t(j).kf.fsm := fsm;
      r.t(j).kf.fen := fen;
      if r.t(j).s2 = QB and okx(QB) = '1' and enx(QB) = '1' then
        r.t(j).kf.sp := '1';
      else
        r.t(j).kf.sp := '0';
      end if;
      for wl in 0 to 1 loop
        -- Slots 2, 3 starting here.
        kb := KB_ZERO;
        if ok(wl) = '0' or (smx(j) = '0' and enx(j) = '0') then
          null;
        elsif smx(j) = '0' then
          kb := ("01", '1', "00");
        elsif fok(wl) = '0' or (fsm = '0' and fen = '0') then
          kb := ("01", '0', "01");
        elsif fsm = '0' then
          kb := ("10", '1', "01");
        else
          kb := ("10", '0', "10");
        end if;
        r.t(j).kb(wl) := kb;
      end loop;
    end loop;
    r.last := t.last;
    if t.last = '1' and t.endi = QB-1 then
      r.ez := '1';
    else
      r.ez := '0';
    end if;
    r.vlen := t.vlen;
    return r;
  end function;

  function line_fields(t : ptab_type) return fline_type is
    variable r : fline_type;
  begin
    r.w := t.w;
    for j in 0 to QB-1 loop
      r.hl(j) := resize(t.e(j).hend - j, 4);
    end loop;
    r.lc := t.lc;
    return r;
  end function;

  -- Input pipeline.
  signal x1, x2   : line_type := LINE_ZERO;
  signal x1v, x2v : std_logic := '0';
  signal pt       : ptab_type;
  signal ptv      : std_logic := '0';
  signal lc_next  : unsigned(14 downto 0) := (others => '0');
  signal wpos     : unsigned(19 downto 0) := (others => '0');
  signal pop      : std_logic;
  signal mv1, mv2 : std_logic;
  signal mvq      : std_logic;

  -- Line file: walk tables of the queued lines hp .. hp+qcnt-1 (line n in
  -- slot n mod 3), read directly by the walk; registers written with a clock
  -- enable, so the only logic is the lookup multiplexers. The fields go to a
  -- queue with the same slots in block RAM, one copy per window line, whose
  -- output registers are the field extraction's line registers. The window
  -- is lines hp + lag + (0, 1) (lag = 1: the window advanced last cycle; the
  -- head moves this cycle).
  signal wf       : wline_array(0 to 2);
  signal hp       : unsigned(1 downto 0) := "00";
  signal qcnt     : unsigned(1 downto 0) := "00";
  signal lag      : std_logic := '0';

  -- Field queue copies: window line 0 needs bytes 0 .. 31, the header
  -- lengths and the line number; window line 1 bytes 0 .. 33 and the header
  -- lengths.
  constant FM0_BITS : natural := 8*QB + 4*QB + 15;
  constant FM1_BITS : natural := 8*(QB+2) + 4*QB;
  type fm0_type is array (0 to 3) of std_logic_vector(FM0_BITS-1 downto 0);
  type fm1_type is array (0 to 3) of std_logic_vector(FM1_BITS-1 downto 0);
  signal fm0      : fm0_type;
  signal fm1      : fm1_type;
  signal fq0      : std_logic_vector(FM0_BITS-1 downto 0);
  signal fq1      : std_logic_vector(FM1_BITS-1 downto 0);

  function fm_pack(f : fline_type; nb : natural; wlc : boolean) return std_logic_vector is
    variable v : std_logic_vector(8*nb + 4*QB + 15 - 1 downto 0);
  begin
    for i in 0 to nb-1 loop
      v(8*i+7 downto 8*i) := f.w(i);
    end loop;
    for i in 0 to QB-1 loop
      v(8*nb+4*i+3 downto 8*nb+4*i) := std_logic_vector(f.hl(i));
    end loop;
    v(8*nb+4*QB+14 downto 8*nb+4*QB) := std_logic_vector(f.lc);
    if wlc then
      return v;
    else
      return v(8*nb+4*QB-1 downto 0);
    end if;
  end function;

  function fm_unpack(v : std_logic_vector; nb : natural; wlc : boolean) return fline_type is
    variable f : fline_type;
  begin
    f.w := (others => X"00");
    for i in 0 to nb-1 loop
      f.w(i) := v(8*i+7 downto 8*i);
    end loop;
    for i in 0 to QB-1 loop
      f.hl(i) := unsigned(v(8*nb+4*i+3 downto 8*nb+4*i));
    end loop;
    f.lc := (others => '0');
    if wlc then
      f.lc := unsigned(v(8*nb+4*QB+14 downto 8*nb+4*QB));
    end if;
    return f;
  end function;

  attribute ram_style : string;
  attribute ram_style of wf : signal is "registers";
  attribute ram_style of fm0, fm1 : signal is "block";

  -- Next element start: rel_raw + 32 * (radd - rsh) relative to the start of
  -- line hp (rel_raw + 32 * radd is relative to the line before the last
  -- head move, rsh says whether the head moved; radd is the line of a table
  -- entry whose line-relative end was taken).
  signal rel_raw  : unsigned(17 downto 0) := (others => '0');
  signal radd     : unsigned(1 downto 0) := "00";
  signal kreg     : unsigned(1 downto 0) := "00";  -- (hp + radd - rsh) mod 3

  -- Slot arithmetic modulo 3 (a, b < 4).
  function m3(a : unsigned(1 downto 0); b : natural) return unsigned is
  begin
    return to_unsigned((to_integer(a) + b) mod 3, 2);
  end function;
  signal rsh      : std_logic := '0';
  signal newchunk : std_logic := '1';

  -- Field extraction stage: element starts relative to the line before the
  -- last head move (fx_sh: whether it moved), the fields of the two window
  -- lines (which start at the line after the head move), and the element
  -- ends from the walk (line-relative).
  type u7_arr is array (natural range <>) of unsigned(6 downto 0);
  signal fx_e     : u7_arr(0 to QK-1);
  signal fx_cnt   : unsigned(2 downto 0) := "000";
  signal fx_endc  : std_logic := '0';
  signal fx_sh    : std_logic := '0';
  signal fx_base  : unsigned(14 downto 0) := (others => '0');
  signal fx_q     : fline_array(0 to 1);
  signal fx_btb   : unsigned(17 downto 0);   -- end of the big element
  signal fx_big   : std_logic_vector(0 to 1); -- slot 0 / slot 1 is big
  signal fx_en0   : unsigned(5 downto 0);
  signal fx_en1   : unsigned(5 downto 0);
  signal fx_en2   : unsigned(5 downto 0);
  signal fx_en3   : unsigned(5 downto 0);

begin

  -----------------------------------------------------------------------------
  -- Input pipeline: FIFO -> x1 -> x2 -> (A) line table -> pt -> (B) walk
  -- table -> queue. All movement decisions depend only on registered state.
  -----------------------------------------------------------------------------
  -- Fields of the window lines, for the field extraction next cycle.
  fm_proc: process (clk) is
    variable wa : integer range 0 to 3;
  begin
    if rising_edge(clk) then
      wa := to_integer(m3(hp, to_integer(qcnt)));
      if mvq = '1' then
        fm0(wa) <= fm_pack(line_fields(pt), QB, true);
        fm1(wa) <= fm_pack(line_fields(pt), QB+2, false);
      end if;
      fq0 <= fm0(to_integer(m3(hp, to_integer(unsigned'("" & lag)))));
      fq1 <= fm1(to_integer(m3(hp, to_integer(unsigned'("" & lag)) + 1)));
    end if;
  end process;
  fx_q(0) <= fm_unpack(fq0, QB, true);
  fx_q(1) <= fm_unpack(fq1, QB+2, false);
  fx_base <= fx_q(0).lc;

  -- A full file takes a new line when the head moves (its slot is free).
  mvq <= ptv when qcnt /= 3 or lag = '1' else '0';
  mv2 <= x2v and (x1v or x2.last) and (not ptv or mvq);

  -- The new line goes to the slot after the last valid one; that slot
  -- doesn't change when the window advances in the same cycle.
  qwr_proc: process (clk) is
  begin
    if rising_edge(clk) then
      if mvq = '1' then
        wf(to_integer(m3(hp, to_integer(qcnt)))) <= line_walk(pt);
      end if;
    end if;
  end process;
  mv1 <= x1v and (not x2v or mv2);
  pop <= cs_valid and (not x1v or mv1) and not reset
     when (lc_next(10 downto 0) - cons_pos(15 downto 5)) < 32 else '0';
  cs_ready <= pop;

  lb_we <= pop;
  lb_line <= lc_next(QI-1 downto 0);
  lb_data <= cs_data;
  lb_wpos <= wpos;

  in_proc: process (clk) is
  begin
    if rising_edge(clk) then
      if mv2 = '1' then
        pt <= line_table(x2, x1, x1v and not x2.last);
        ptv <= '1';
      elsif mvq = '1' then
        ptv <= '0';
      end if;
      if mv1 = '1' then
        x2 <= x1;
        x2v <= '1';
      elsif mv2 = '1' then
        x2v <= '0';
      end if;
      if pop = '1' then
        x1.data <= cs_data;
        x1.last <= cs_last;
        x1.endi <= cs_endi;
        x1.lc <= lc_next;
        x1v <= '1';
        lc_next <= lc_next + 1;
        wpos <= (lc_next + 1) & "00000";
      elsif mv1 = '1' then
        x1v <= '0';
      end if;
      if reset = '1' then
        x1v <= '0';
        x2v <= '0';
        ptv <= '0';
        lc_next <= (others => '0');
        wpos <= (others => '0');
      end if;
    end if;
  end process;

  main_proc: process (clk) is
    variable fl, sl     : integer range 0 to 2;
    variable ready      : std_logic;
    variable run        : std_logic;
    variable relc       : unsigned(17 downto 0);
    variable l0         : unsigned(1 downto 0);
    variable l1, l2     : unsigned(1 downto 0);
    variable w0, w2     : integer range 0 to 1;
    variable s0, s1, s2 : unsigned(1 downto 0);
    variable lv         : std_logic_vector(0 to 2);
    variable lastf      : std_logic;
    variable ezf        : std_logic;
    variable vlf, vls   : unsigned(2 downto 0);
    variable e0, e1, e2 : unsigned(6 downto 0);
    variable e3, e4     : unsigned(6 downto 0);
    variable ta, tb, tg : went_type;
    variable sg         : unsigned(1 downto 0);
    variable pg         : unsigned(4 downto 0);
    variable ka         : kinda_type;
    variable kb         : kindb_type;
    variable rn         : unsigned(17 downto 0);
    variable ra         : unsigned(1 downto 0);
    variable cnt        : unsigned(2 downto 0);
    variable endc       : std_logic;
    variable ncnt       : unsigned(1 downto 0);
    variable lg         : integer range 0 to 1;
    variable ge         : std_logic;
    variable rge        : std_logic;

    -- Position p (relative to line hp) is at least 32 * (lag + 1).
    function ge7(p : unsigned(6 downto 0); lg : integer) return std_logic is
    begin
      if lg = 0 then
        return p(6) or p(5);
      else
        return p(6);
      end if;
    end function;

    -- Table position of the line-relative position p in line l.
    function tpos(l : unsigned(1 downto 0); p : unsigned(5 downto 0)) return unsigned is
    begin
      return (l + ("0" & p(5))) & p(4 downto 0);
    end function;
  begin
    if rising_edge(clk) then

      if lag = '1' then fl := 1; sl := 2; lg := 1; else fl := 0; sl := 1; lg := 0; end if;
      for i in 0 to 2 loop
        if qcnt > i then lv(i) := '1'; else lv(i) := '0'; end if;
      end loop;
      lastf := wf(to_integer(m3(hp, fl))).last;
      ezf := wf(to_integer(m3(hp, fl))).ez;
      vlf := wf(to_integer(m3(hp, fl))).vlen;
      vls := wf(to_integer(m3(hp, sl))).vlen;
      ready := lv(fl) and (lv(sl) or lastf);

      -- Current start, relative to line hp.
      relc := rel_raw + shift_left(resize(radd, 18), 5);
      if rsh = '1' then
        relc := relc - 32;
      end if;
      run := ready and el_space and not newchunk;
      if relc >= to_unsigned(64 + 32 * lg, 18) then
        run := '0';
      end if;
      if relc >= to_unsigned(32 + 32 * lg, 18) then
        rge := '1';
      else
        rge := '0';
      end if;

      -- Element walk: two table lookups. The line of the start is selected
      -- separately, so the head move doesn't add to the lookup.
      l0 := rel_raw(6 downto 5) + radd - ("0" & rsh);
      s0 := m3(kreg, to_integer(rel_raw(6 downto 5)));
      e0 := l0 & rel_raw(4 downto 0);
      if l0(0) = lag then w0 := 0; else w0 := 1; end if;
      ta := wf(to_integer(s0)).t(to_integer(rel_raw(4 downto 0)));
      ka := kinda_of(ta.kf, w0);
      -- The end marker of a chunk that ends exactly at the end of the first
      -- window line is at the start of the second (whose table may not exist).
      if w0 = 1 and rel_raw(4 downto 0) = 0 and ezf = '1' then
        ka := ("01", '0', '1', "01");
      end if;
      e1 := tpos(l0, ta.step);
      e2 := tpos(l0, ta.s2);
      s1 := m3(s0, to_integer(ta.step(5 downto 5)));
      s2 := m3(s0, to_integer(ta.s2(5 downto 5)));
      l1 := e1(6 downto 5);
      l2 := e2(6 downto 5);
      if l2(0) = lag then w2 := 0; else w2 := 1; end if;
      tb := wf(to_integer(s2)).t(to_integer(ta.s2(4 downto 0)));
      kb := tb.kb(w2);
      -- Slot 2 start past the window.
      if (lag = '0' and l2(1) = '1') or (lag = '1' and l2 = "11") then
        kb := KB_ZERO;
      end if;
      e3 := tpos(l2, tb.step);
      e4 := tpos(l2, tb.s2);
      -- The end of the big element of the cycle (at most one: slot 0 if it
      -- isn't small, otherwise slot 1).
      if ta.kf.sm = '0' then
        sg := s0;
        pg := rel_raw(4 downto 0);
      else
        sg := s1;
        pg := ta.step(4 downto 0);
      end if;
      tg := wf(to_integer(sg)).t(to_integer(pg));

      -- Next start, and whether it is past the first window line (ge).
      ra := "00";
      if ka.cont = '0' then
        cnt := '0' & ka.n;
        endc := ka.endc;
        case ka.sel is
          when "00" => rn := relc; ge := rge;
          when "01" => rn := tg.bt; ra := l0; ge := tg.btge or to_unsigned(w0, 1)(0);
          when "10" => rn := resize(e1, 18); ge := ge7(e1, lg);
          when others =>
            rn := tg.bt; ra := l1; ge := tg.btge;
            if l1(0) /= lag then
              ge := '1';
            end if;
        end case;
      else
        cnt := ('0' & kb.n) + 2;
        endc := kb.endc;
        case kb.sel is
          when "00" => rn := resize(e2, 18); ge := ge7(e2, lg);
          when "01" => rn := resize(e3, 18); ge := ge7(e3, lg);
          when others => rn := resize(e4, 18); ge := ge7(e4, lg);
        end case;
      end if;
      if run = '0' then
        cnt := "000";
        endc := '0';
        rn := relc;
        ra := "00";
        ge := rge;
      end if;

      -- Field extraction happens in the next cycle.
      fx_e(0) <= e0;
      fx_e(1) <= e1;
      fx_e(2) <= e2;
      fx_e(3) <= e3;
      fx_cnt <= cnt;
      fx_endc <= endc;
      fx_sh <= lag;
      fx_btb <= tg.bt;
      fx_big(0) <= not ta.kf.sm;
      fx_big(1) <= not ta.kf.fsm;
      fx_en0 <= ta.step;
      fx_en1 <= ta.s2;
      fx_en2 <= tb.step;
      fx_en3 <= tb.s2;

      -- Window advance. All positions here are relative to line hp; the head
      -- moves by lag at the end of this cycle.
      rsh <= lag;
      radd <= "00";
      if endc = '1' then
        -- End of chunk in the first window line: the next chunk starts in
        -- the line after it.
        lag <= '1';
        kreg <= hp;
        if lv(sl) = '1' then
          rel_raw <= to_unsigned(32 * sl, 18) + resize(vls, 18);
          newchunk <= '0';
        else
          rel_raw <= to_unsigned(32 * sl, 18);
          newchunk <= '1';
        end if;
      elsif newchunk = '1' then
        lag <= '0';
        rel_raw <= to_unsigned(32 * fl, 18);
        kreg <= hp;
        if lv(fl) = '1' then
          rel_raw <= to_unsigned(32 * fl, 18) + resize(vlf, 18);
          newchunk <= '0';
        end if;
      elsif ge = '1' and lv(sl) = '1' and lastf = '0' and lv(fl) = '1' then
        lag <= '1';
        rel_raw <= rn;
        radd <= ra;
        kreg <= m3(hp, to_integer(ra));
      else
        lag <= '0';
        rel_raw <= rn;
        radd <= ra;
        kreg <= m3(hp, to_integer(ra));
      end if;

      -- Queue pointers.
      ncnt := qcnt;
      if lag = '1' then
        hp <= m3(hp, 1);
        ncnt := ncnt - 1;
      end if;
      if mvq = '1' then
        ncnt := ncnt + 1;
      end if;
      qcnt <= ncnt;

      if reset = '1' then
        qcnt <= "00";
        hp <= "00";
        kreg <= "00";
        lag <= '0';
        rsh <= '0';
        radd <= "00";
        newchunk <= '1';
        rel_raw <= (others => '0');
        fx_cnt <= "000";
      end if;
    end if;
  end process;

  -- Field extraction for the elements emitted in the previous cycle. They
  -- start in the two window lines. The literal length is the element end
  -- minus the header end (slots 2 and 3 only hold small elements).
  fx_proc: process (clk) is
    variable wb   : byte_array(0 to 2*QB+1);
    variable wh   : u4_hl_array(0 to 2*QB-1);
    variable b0   : std_logic_vector(7 downto 0);
    variable b1   : std_logic_vector(7 downto 0);
    variable b2   : std_logic_vector(7 downto 0);
    variable hend : unsigned(5 downto 0);
    variable wl   : std_logic;
    variable wp   : unsigned(5 downto 0);
    variable base : unsigned(19 downto 0);
    variable li   : unsigned(17 downto 0);
  begin
    if rising_edge(clk) then
      base := fx_base & "00000";
      wb := fx_q(0).w(0 to QB-1) & fx_q(1).w;
      wh := fx_q(0).hl & fx_q(1).hl;
      for k in 0 to QK-1 loop
        -- Position relative to the window.
        wp := fx_e(k)(5 downto 0);
        wp(5) := fx_e(k)(5) xor fx_sh;
        wl := wp(5);
        b0 := wb(to_integer(wp));
        b1 := wb(to_integer(wp) + 1);
        b2 := wb(to_integer(wp) + 2);
        hend := resize(wp(4 downto 0), 6) + wh(to_integer(wp));
        case k is
          -- Small elements are at most 12 bytes, so their literal length
          -- follows modulo 16 from the next start (in any line).
          when 0 =>
            if fx_big(0) = '1' then
              li := fx_btb - resize(hend, 18);
            else
              li := resize((fx_en0 - hend) and "001111", 18);
            end if;
          when 1 =>
            if fx_big(1) = '1' then
              li := fx_btb - resize(hend, 18);
            else
              li := resize((fx_en1 - hend) and "001111", 18);
            end if;
          when 2 => li := resize((fx_en2 - hend) and "001111", 18);
          when others => li := resize((fx_en3 - hend) and "001111", 18);
        end case;
        case b0(1 downto 0) is
          when "01" =>
            el(k).cp <= resize(unsigned(b0(4 downto 2)), 7) + 4;
            el(k).off <= "00000" & unsigned(b0(7 downto 5)) & unsigned(b1);
          when "10" | "11" =>
            el(k).cp <= resize(unsigned(b0(7 downto 2)), 7) + 1;
            el(k).off <= unsigned(b2) & unsigned(b1);
          when others =>
            el(k).cp <= (others => '0');
            el(k).off <= (others => '0');
        end case;
        el(k).li <= li(16 downto 0);
        el(k).q <= base + resize(wl & "00000", 20) + resize(hend, 20);
        el(k).last <= '0';
        el(k).tot <= (others => '0');
        el(k).pw2 <= '0';
        el(k).s4 <= '0';
        el(k).s8 <= '0';
        if fx_endc = '1' and k + 1 = fx_cnt then
          -- An end marker may be at the start of the line after the window.
          el(k).cp <= (others => '0');
          el(k).li <= (others => '0');
          el(k).q <= base + resize(fx_e(k), 20) - resize(fx_sh & "00000", 20);
          el(k).last <= '1';
        end if;
      end loop;
      el_cnt <= fx_cnt;
      if reset = '1' then
        el_cnt <= "000";
      end if;
    end if;
  end process;

  -- pragma translate_off
  dbg_proc: process (clk) is
    variable cyc : natural := 0;
  begin
    if rising_edge(clk) then
      cyc := cyc + 1;
      if cyc < QDEBUG then
        report "DEC c=" & integer'image(cyc) & " rel=" & integer'image(to_integer(rel_raw)) & " radd=" & integer'image(to_integer(radd))
          & " rsh=" & std_logic'image(rsh) & " lag=" & std_logic'image(lag)
          & " nc=" & std_logic'image(newchunk) & " qcnt=" & integer'image(to_integer(qcnt)) & " x1v=" & std_logic'image(x1v)
          & " x2v=" & std_logic'image(x2v) & " ptv=" & std_logic'image(ptv) & " pop=" & std_logic'image(pop) & " lc=" & integer'image(to_integer(lc_next))
          & " el_cnt=" & integer'image(to_integer(el_cnt)) & " sp=" & std_logic'image(el_space)
          & " cons=" & integer'image(to_integer(cons_pos));
      end if;
    end if;
  end process;
  -- pragma translate_on

end behavior;
