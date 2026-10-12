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
    first : std_logic;
    lc    : unsigned(14 downto 0);
  end record;

  constant LINE_ZERO : line_type := (
    data => (others => X"00"), last => '0', endi => (others => '0'),
    first => '0', lc => (others => '0'));

  -- Line table entry: element starting at this byte of the line.
  type pent_type is record
    ok    : std_logic;              -- element (or end marker) starts here
    sm    : std_logic;              -- small
    endm  : std_logic;              -- end-of-chunk marker
    sz4   : unsigned(3 downto 0);   -- size when small
    cp    : unsigned(6 downto 0);
    off   : unsigned(15 downto 0);
    hend  : unsigned(5 downto 0);   -- end of the header, relative to the line
    bt    : unsigned(17 downto 0);  -- end of the element (hend + literal
                                    -- length), relative to the line
  end record;

  constant PENT_ZERO : pent_type := (
    ok => '0', sm => '0', endm => '0', sz4 => (others => '0'),
    cp => (others => '0'), off => (others => '0'), hend => (others => '0'),
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
  type kinda_array is array (0 to 1) of kinda_type;
  type kindb_array is array (0 to 1) of kindb_type;

  -- Walk table entry. Positions are relative to the entry's line.
  type went_type is record
    ka    : kinda_array;
    kb    : kindb_array;
    step  : unsigned(5 downto 0);   -- next element start (if small)
    s2    : unsigned(5 downto 0);   -- start after that one
    bt    : unsigned(17 downto 0);  -- next element start (any size)
    btge  : std_logic;              -- bt >= 32
  end record;

  constant WENT_ZERO : went_type := (
    ka => (others => KA_ZERO), kb => (others => KB_ZERO),
    step => (others => '0'), s2 => (others => '0'), bt => (others => '0'), btge => '0');

  type went_array is array (natural range <>) of went_type;

  type wline_type is record
    t     : went_array(0 to QB-1);
    last  : std_logic;
    ez    : std_logic;              -- last, and the chunk ends at the line end
    vlen  : unsigned(2 downto 0);
  end record;

  type wline_array is array (natural range <>) of wline_type;

  -- Fields of the element at a position (the literal length follows from
  -- the element end in the walk table).
  type fent_type is record
    cp    : unsigned(6 downto 0);
    off   : unsigned(15 downto 0);
    hend  : unsigned(5 downto 0);
  end record;

  type fent_array is array (natural range <>) of fent_type;

  type fline_type is record
    f     : fent_array(0 to QB-1);
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
    for i in 0 to QB-1 loop
      w(i) := x.data(i);
      if yv = '1' then
        w(QB + i) := y.data(i);
      else
        w(QB + i) := X"00";
      end if;
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
        r.e(j).cp := (others => '0');
        r.e(j).off := (others => '0');
        case tag is
          when "01" =>
            r.e(j).cp := resize(unsigned(w(j)(4 downto 2)), 7) + 4;
            r.e(j).off := "00000" & unsigned(w(j)(7 downto 5)) & unsigned(w(j + 1));
          when "10" | "11" =>
            r.e(j).cp := resize(unsigned(w(j)(7 downto 2)), 7) + 1;
            r.e(j).off := unsigned(w(j + 2)) & unsigned(w(j + 1));
          when others =>
            null;
        end case;
        if haslit = '1' then
          hend := m + lh(m);
          r.e(j).bt := to_unsigned(hend, 18) + resize(lm1(m), 18) + 1;
        else
          hend := m;
          r.e(j).bt := to_unsigned(hend, 18);
        end if;
        r.e(j).hend := to_unsigned(hend, 6);
        if endm = '1' then
          r.e(j).cp := (others => '0');
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
    variable ka   : kinda_type;
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
      for wl in 0 to 1 loop
        -- Slots 0, 1 starting here.
        ka := KA_ZERO;
        if ok(wl) = '0' then
          null;
        elsif smx(j) = '0' then
          ka := ("01", '0', enx(j), "01");
        elsif fok(wl) = '0' then
          ka := ("01", '0', '0', "10");
        elsif fsm = '0' then
          ka := ("10", '0', fen, "11");
        elsif wl = 0 and r.t(j).s2 = QB and okx(QB) = '1' and enx(QB) = '1' then
          -- The start after slot 1 is the end marker at the line end.
          ka := ("11", '0', '1', "00");
        else
          ka := ("10", '1', '0', "00");
        end if;
        r.t(j).ka(wl) := ka;
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
    for j in 0 to QB-1 loop
      r.f(j).cp := t.e(j).cp;
      r.f(j).off := t.e(j).off;
      r.f(j).hend := t.e(j).hend;
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
  signal prev_last: std_logic := '1';
  signal wpos     : unsigned(19 downto 0) := (others => '0');
  signal pop      : std_logic;
  signal mv1, mv2 : std_logic;
  signal mvq      : std_logic;

  -- Line file: walk tables of the queued lines hp .. hp+qcnt-1 (line n in
  -- slot n mod 3), read directly by the walk; registers written with a clock
  -- enable, so the only logic is the lookup multiplexers. The fields go to a
  -- queue with the same slots. The window is lines hp + lag + (0, 1) (lag =
  -- 1: the window advanced last cycle; the head moves this cycle).
  signal wf       : wline_array(0 to 2);
  signal qf       : fline_array(0 to 2);
  signal hp       : unsigned(1 downto 0) := "00";
  signal qcnt     : unsigned(1 downto 0) := "00";
  signal lag      : std_logic := '0';
  signal win_f    : fline_array(0 to 1);

  attribute ram_style : string;
  attribute ram_style of wf : signal is "registers";

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
  signal fx_bt0   : unsigned(17 downto 0);
  signal fx_bt1   : unsigned(17 downto 0);
  signal fx_en2   : unsigned(5 downto 0);
  signal fx_en3   : unsigned(5 downto 0);

begin

  -----------------------------------------------------------------------------
  -- Input pipeline: FIFO -> x1 -> x2 -> (A) line table -> pt -> (B) walk
  -- table -> queue. All movement decisions depend only on registered state.
  -----------------------------------------------------------------------------
  -- Fields of the window lines, for the field extraction next cycle.
  qf_gen: for i in 0 to 1 generate
    win_f(i) <= qf(to_integer(m3(hp, to_integer(unsigned'("" & lag)) + i)));
  end generate;

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
        qf(to_integer(m3(hp, to_integer(qcnt)))) <= line_fields(pt);
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
        x1.first <= prev_last;
        x1.lc <= lc_next;
        x1v <= '1';
        prev_last <= cs_last;
        lc_next <= lc_next + 1;
        wpos <= (lc_next + 1) & "00000";
      elsif mv1 = '1' then
        x1v <= '0';
      end if;
      if reset = '1' then
        x1v <= '0';
        x2v <= '0';
        ptv <= '0';
        prev_last <= '1';
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
    variable ta, tb, t1 : went_type;
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
      ka := ta.ka(w0);
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
      t1 := wf(to_integer(s1)).t(to_integer(ta.step(4 downto 0)));

      -- Next start, and whether it is past the first window line (ge).
      ra := "00";
      if ka.cont = '0' then
        cnt := '0' & ka.n;
        endc := ka.endc;
        case ka.sel is
          when "00" => rn := relc; ge := rge;
          when "01" => rn := ta.bt; ra := l0; ge := ta.btge or to_unsigned(w0, 1)(0);
          when "10" => rn := resize(e1, 18); ge := ge7(e1, lg);
          when others =>
            rn := t1.bt; ra := l1; ge := t1.btge;
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
      fx_base <= win_f(0).lc;
      fx_q <= win_f;
      fx_bt0 <= ta.bt;
      fx_bt1 <= t1.bt;
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
    variable pe   : fent_type;
    variable wl   : std_logic;
    variable wp   : unsigned(5 downto 0);
    variable base : unsigned(19 downto 0);
    variable en   : unsigned(17 downto 0);
    variable li   : unsigned(17 downto 0);
  begin
    if rising_edge(clk) then
      base := fx_base & "00000";
      for k in 0 to QK-1 loop
        -- Position relative to the window.
        wp := fx_e(k)(5 downto 0);
        wp(5) := fx_e(k)(5) xor fx_sh;
        wl := wp(5);
        if wl = '0' then
          pe := fx_q(0).f(to_integer(wp(4 downto 0)));
        else
          pe := fx_q(1).f(to_integer(wp(4 downto 0)));
        end if;
        case k is
          when 0 => li := fx_bt0 - resize(pe.hend, 18);
          when 1 => li := fx_bt1 - resize(pe.hend, 18);
          when 2 => li := resize((fx_en2 - pe.hend) and "001111", 18);
          when others => li := resize((fx_en3 - pe.hend) and "001111", 18);
        end case;
        el(k).cp <= pe.cp;
        el(k).off <= pe.off;
        el(k).li <= li(16 downto 0);
        el(k).q <= base + resize(wl & "00000", 20) + resize(pe.hend, 20);
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
