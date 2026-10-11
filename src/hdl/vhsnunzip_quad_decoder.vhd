library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.vhsnunzip_int_pkg.all;
use work.vhsnunzip_quad_pkg.all;

-- Snappy element decoder for the quad datapath: up to 4 elements per cycle.
--
-- Every compressed line passes through an input pipeline that decodes an
-- element at each of its 32 byte positions (with lookahead into the next
-- line for headers that cross the line end): whether an element can start
-- there, whether it is "small" (total compressed size at most 12 bytes, so
-- the next element start is reachable by a table lookup), its size and its
-- fields. These line tables queue up in front of the decoder window.
--
-- The window covers two lines (A, B). For it, a registered table holds the
-- step to the next element start and the two-step position s2 for each of
-- the 64 positions. When the window advances, the table for (B, C) is built
-- from the two queued line tables, so the loop-carried path is only the walk
-- from one element start to the next: e1 = step(e0), e2 = s2(e0),
-- e3 = step(e2), e4 = s2(e2), i.e. two serial table lookups.
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
    li    : unsigned(16 downto 0);
    hend  : unsigned(5 downto 0);   -- end of the header, relative to the line;
                                    -- the element ends at hend + li
  end record;

  constant PENT_ZERO : pent_type := (
    ok => '0', sm => '0', endm => '0', sz4 => (others => '0'),
    cp => (others => '0'), off => (others => '0'), li => (others => '0'),
    hend => (others => '0'));

  type pent_array is array (natural range <>) of pent_type;

  type ptab_type is record
    e     : pent_array(0 to QB-1);
    last  : std_logic;
    endi  : unsigned(QI-1 downto 0);
    lc    : unsigned(14 downto 0);
    vlen  : unsigned(2 downto 0);   -- varint length if the line starts a chunk
  end record;

  constant PTAB_ZERO : ptab_type := (
    e => (others => PENT_ZERO), last => '0', endi => (others => '0'),
    lc => (others => '0'), vlen => (others => '0'));

  type ptab_array is array (natural range <>) of ptab_type;

  -- Outcome of the first half of the walk (slots 0, 1) from an element
  -- start, for one window offset (lag): number of elements emitted, whether
  -- the walk continues with slot 2, whether it ends with an end marker, and
  -- where the next start comes from (0: unchanged, 1: bt of slot 0, 2: slot 1
  -- start, 3: bt of slot 1).
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

  type kinda_array is array (0 to 1) of kinda_type;
  type kindb_array is array (0 to 1) of kindb_type;

  -- Window table entry. The registered window table covers three lines
  -- (96 positions); the decoder window is lines (lag, lag+1) of it. Positions
  -- 96 .. 127 are empty.
  type went_type is record
    ka    : kinda_array;            -- indexed by lag
    kb    : kindb_array;
    step  : unsigned(6 downto 0);   -- next element start (if small)
    s2    : unsigned(6 downto 0);   -- start after that one
    bt    : unsigned(17 downto 0);  -- next element start (any size)
    btge  : std_logic_vector(0 to 1);  -- bt >= 32 * (lag + 1)
  end record;

  constant WENT_ZERO : went_type := (
    ka => (others => ("00", '0', '0', "00")), kb => (others => ("00", '0', "00")),
    step => (others => '0'), s2 => (others => '0'), bt => (others => '0'), btge => "00");

  type wtab_type is array (0 to 127) of went_type;

  type u3_arr is array (natural range <>) of unsigned(2 downto 0);

  type wt3_type is record
    t     : wtab_type;
    lv    : std_logic_vector(0 to 2);  -- line valid
    last  : std_logic_vector(0 to 2);
    vlen  : u3_arr(0 to 2);
  end record;

  type u4_arr is array (natural range <>) of unsigned(3 downto 0);

  -- Decodes the elements starting in line x; y is the next line (yv: valid
  -- and in the same chunk).
  function line_table(x : line_type; y : line_type; yv : std_logic) return ptab_type is
    variable w      : byte_array(0 to 47);
    variable e      : integer range 0 to 127;
    type int_array is array (natural range <>) of integer;
    variable islit  : std_logic_vector(0 to 39);
    variable lh     : int_array(0 to 39);
    type u17_array is array (natural range <>) of unsigned(16 downto 0);
    variable litlen : u17_array(0 to 39);
    variable lacc   : unsigned(16 downto 0);
    variable fld    : unsigned(5 downto 0);
    variable nb     : integer range 0 to 4;
    variable tag    : std_logic_vector(1 downto 0);
    variable clen   : integer range 0 to 5;
    variable m      : integer range 0 to 63;
    variable haslit : std_logic;
    variable hend   : integer range 0 to 63;
    variable lfld   : unsigned(5 downto 0);
    variable r      : ptab_type;
  begin
    for i in 0 to QB-1 loop
      w(i) := x.data(i);
    end loop;
    for i in 0 to 15 loop
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

    -- Literal headers.
    for i in 0 to 39 loop
      islit(i) := not w(i)(1) and not w(i)(0);
      fld := unsigned(w(i)(7 downto 2));
      if fld < 60 then
        lh(i) := 1;
        litlen(i) := resize(fld, 17) + 1;
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
        litlen(i) := lacc + 1;
      end if;
    end loop;

    -- Elements.
    for j in 0 to QB-1 loop
      tag := w(j)(1 downto 0);
      r.e(j).cp := (others => '0');
      r.e(j).off := (others => '0');
      case tag is
        when "01" =>
          clen := 2;
          r.e(j).cp := resize(unsigned(w(j)(4 downto 2)), 7) + 4;
          r.e(j).off := "00000" & unsigned(w(j)(7 downto 5)) & unsigned(w(j + 1));
        when "10" =>
          clen := 3;
          r.e(j).cp := resize(unsigned(w(j)(7 downto 2)), 7) + 1;
          r.e(j).off := unsigned(w(j + 2)) & unsigned(w(j + 1));
        when "11" =>
          clen := 5;
          r.e(j).cp := resize(unsigned(w(j)(7 downto 2)), 7) + 1;
          r.e(j).off := unsigned(w(j + 2)) & unsigned(w(j + 1));
        when others =>
          clen := 0;
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
        r.e(j).ok := '1';
      else
        r.e(j).ok := '0';
      end if;
      if haslit = '1' then
        r.e(j).li := litlen(m);
        hend := m + lh(m);
        r.e(j).sz4 := to_unsigned(clen + 2, 4) + lfld(3 downto 0);
        if resize(lfld, 7) + clen <= 10 then
          r.e(j).sm := r.e(j).ok;
        else
          r.e(j).sm := '0';
        end if;
      else
        r.e(j).li := (others => '0');
        hend := m;
        r.e(j).sz4 := to_unsigned(clen, 4);
        r.e(j).sm := r.e(j).ok;
      end if;
      r.e(j).hend := to_unsigned(hend, 6);
      r.e(j).endm := '0';
      if x.last = '1' and j = e then
        r.e(j).ok := '1';
        r.e(j).sm := '0';
        r.e(j).endm := '1';
        r.e(j).cp := (others => '0');
        r.e(j).li := (others => '0');
        r.e(j).hend := to_unsigned(j, 6);
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

  -- Element at position p can be emitted for window offset lg: inside the
  -- window, and an end marker only in the window's first line. An end marker
  -- at the start of a line belongs to the line before it (the chunk ended
  -- exactly at that line's end).
  function okw(ok : std_logic; endm : std_logic; p : natural; lg : natural) return std_logic is
    variable inw, first : boolean;
  begin
    if lg = 0 then
      inw := p < 64;
      first := p <= 32;
    else
      inw := p < 96;
      first := p > 32 and p <= 64;
    end if;
    if inw and (endm = '0' or first) then
      return ok;
    end if;
    return '0';
  end function;

  -- Builds the registered table for the three lines l(0..2) (valid bits v).
  -- Lines after the end of a chunk are masked, except for an end marker
  -- right after the last byte of a line that ends exactly at its end.
  --
  -- Element emission rules (these match the throughput model): continue past
  -- an element only if it is small; a non-small ("big") element may only be
  -- emitted in slot 0 or 1 and ends the cycle; in slots 2 and 3 only small
  -- elements and end markers are emitted.
  function win3(l : ptab_array(0 to 2); v : std_logic_vector(0 to 2)) return wt3_type is
    variable r    : wt3_type;
    variable pe   : pent_array(0 to 95);
    variable li   : integer range 0 to 2;
    variable mask : std_logic_vector(0 to 2);
    variable fok  : std_logic_vector(0 to 1);
    variable ok   : std_logic_vector(0 to 1);
    variable fsm  : std_logic;
    variable fen  : std_logic;
    variable c    : std_logic;
    variable ka   : kinda_type;
    variable kb   : kindb_type;
  begin
    mask := v;
    if v(0) = '1' and l(0).last = '1' then
      mask(1) := '0';
      mask(2) := '0';
    elsif v(1) = '1' and l(1).last = '1' then
      mask(2) := '0';
    end if;
    for j in 0 to 95 loop
      li := j / 32;
      pe(j) := l(li).e(j mod 32);
      if mask(li) = '0' then
        pe(j) := PENT_ZERO;
        if li > 0 and j mod 32 = 0 and v(li-1) = '1' and l(li-1).last = '1' and l(li-1).endi = QB-1
           and mask(li-1) = '1' then
          pe(j).ok := '1';
          pe(j).endm := '1';
          pe(j).hend := to_unsigned(0, 6);
        end if;
      end if;
    end loop;
    for j in 0 to 127 loop
      r.t(j) := WENT_ZERO;
    end loop;
    for j in 0 to 95 loop
      li := j / 32;
      r.t(j).step := to_unsigned(j, 7) + resize(pe(j).sz4, 7);
      r.t(j).bt := to_unsigned(32 * li, 18) + resize(pe(j).hend, 18) + resize(pe(j).li, 18);
      for lg in 0 to 1 loop
        if r.t(j).bt >= 32 * (lg + 1) then
          r.t(j).btge(lg) := '1';
        else
          r.t(j).btge(lg) := '0';
        end if;
      end loop;
      r.t(j).s2 := r.t(j).step;
      -- The element after this one (if this one is small).
      fok := "00";
      fsm := '0';
      fen := '0';
      for d in 2 to 12 loop
        if j + d < 96 and pe(j).sz4 = d then
          for lg in 0 to 1 loop
            fok(lg) := okw(pe(j + d).ok, pe(j + d).endm, j + d, lg);
          end loop;
          fsm := pe(j + d).sm;
          fen := pe(j + d).endm;
          r.t(j).s2 := to_unsigned(j + d, 7) + resize(pe(j + d).sz4, 7);
        end if;
      end loop;
      for lg in 0 to 1 loop
        ok(lg) := okw(pe(j).ok, pe(j).endm, j, lg);
        -- Slots 0, 1 starting here.
        ka := ("00", '0', '0', "00");
        if ok(lg) = '0' then
          null;
        elsif pe(j).sm = '0' then
          ka := ("01", '0', pe(j).endm, "01");
        elsif fok(lg) = '0' then
          ka := ("01", '0', '0', "10");
        elsif fsm = '0' then
          ka := ("10", '0', fen, "11");
        else
          ka := ("10", '1', '0', "00");
        end if;
        r.t(j).ka(lg) := ka;
        -- Slots 2, 3 starting here.
        kb := ("00", '0', "00");
        if ok(lg) = '0' or (pe(j).sm = '0' and pe(j).endm = '0') then
          null;
        elsif pe(j).sm = '0' then
          kb := ("01", '1', "00");
        elsif fok(lg) = '0' or (fsm = '0' and fen = '0') then
          kb := ("01", '0', "01");
        elsif fsm = '0' then
          kb := ("10", '1', "01");
        else
          kb := ("10", '0', "10");
        end if;
        r.t(j).kb(lg) := kb;
      end loop;
    end loop;
    for i in 0 to 2 loop
      r.lv(i) := v(i);
      r.last(i) := l(i).last;
      r.vlen(i) := l(i).vlen;
    end loop;
    return r;
  end function;

  -- Input pipeline.
  signal x1, x2   : line_type := LINE_ZERO;
  signal x1v, x2v : std_logic := '0';
  signal lc_next  : unsigned(14 downto 0) := (others => '0');
  signal prev_last: std_logic := '1';
  signal wpos     : unsigned(19 downto 0) := (others => '0');
  signal pop      : std_logic;
  signal mv1, mv2 : std_logic;
  signal ptn      : ptab_type;

  -- Line table queue: a circular buffer of 8 starting at the head pointer hp.
  signal qm       : ptab_array(0 to 7) := (others => PTAB_ZERO);
  signal hp       : unsigned(2 downto 0) := "000";
  signal qcnt     : unsigned(3 downto 0) := "0000";
  signal qfull    : std_logic;

  -- Window table for lines hp .. hp+2, and the line offset of the window
  -- (lag = 1: the window advanced last cycle; the queue head moves this
  -- cycle).
  signal wt       : wt3_type;
  signal lag      : std_logic := '0';
  signal win_in   : ptab_array(0 to 2);
  signal win_v    : std_logic_vector(0 to 2);

  -- Next element start: rel_raw - 32 * rsh relative to the start of line hp
  -- (rel_raw is relative to the line before the last head move, rsh says
  -- whether the head moved).
  signal rel_raw  : unsigned(17 downto 0) := (others => '0');
  signal rsh      : std_logic := '0';
  signal newchunk : std_logic := '1';

  -- Field extraction stage: element starts relative to the line before the
  -- last head move (fx_sh: whether it moved), and the two window lines
  -- (which start at the line after the head move).
  type u7_arr is array (natural range <>) of unsigned(6 downto 0);
  signal fx_e     : u7_arr(0 to QK-1);
  signal fx_cnt   : unsigned(2 downto 0) := "000";
  signal fx_endc  : std_logic := '0';
  signal fx_sh    : std_logic := '0';
  signal fx_base  : unsigned(14 downto 0) := (others => '0');
  signal fx_q     : ptab_array(0 to 1);

begin

  -----------------------------------------------------------------------------
  -- Input pipeline: FIFO -> x1 -> x2 -> line table -> queue. All movement
  -- decisions depend only on registered state.
  -----------------------------------------------------------------------------
  -- The window table for the next cycle covers lines hp + lag + (0 .. 2).
  q_gen: for i in 0 to 2 generate
    win_in(i) <= qm(to_integer(hp + ("00" & lag) + i));
    win_v(i) <= '1' when qcnt > i + to_integer(unsigned'("" & lag)) else '0';
  end generate;
  qfull <= qcnt(3);

  mv2 <= x2v and (x1v or x2.last) and not qfull;

  -- The new line table goes to the slot after the last valid one; that slot
  -- doesn't change when the window advances in the same cycle.
  qwr_proc: process (clk) is
  begin
    if rising_edge(clk) then
      if mv2 = '1' then
        qm(to_integer(hp + qcnt(2 downto 0))) <= ptn;
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

  ptn <= line_table(x2, x1, x1v and not x2.last);

  in_proc: process (clk) is
  begin
    if rising_edge(clk) then
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
        prev_last <= '1';
        lc_next <= (others => '0');
        wpos <= (others => '0');
      end if;
    end if;
  end process;

  -----------------------------------------------------------------------------
  -- Window table: recomputed every cycle for the lines that will be hp ..
  -- hp+2 next cycle (the head moves by lag), from registered state only.
  -----------------------------------------------------------------------------
  wt_proc: process (clk) is
  begin
    if rising_edge(clk) then
      wt <= win3(win_in, win_v);
      if reset = '1' then
        wt.lv <= "000";
      end if;
    end if;
  end process;

  main_proc: process (clk) is
    variable fl, sl     : integer range 0 to 2;
    variable ready      : std_logic;
    variable run        : std_logic;
    variable relc       : unsigned(17 downto 0);
    variable l0         : unsigned(1 downto 0);
    variable e0, e1, e2 : unsigned(6 downto 0);
    variable e3, e4     : unsigned(6 downto 0);
    variable ta, tb, t1 : went_type;
    variable ka         : kinda_type;
    variable kb         : kindb_type;
    variable bt1        : unsigned(17 downto 0);
    variable rn         : unsigned(17 downto 0);
    variable cnt        : unsigned(2 downto 0);
    variable endc       : std_logic;
    variable ncnt       : unsigned(3 downto 0);
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
  begin
    if rising_edge(clk) then

      if lag = '1' then fl := 1; sl := 2; lg := 1; else fl := 0; sl := 1; lg := 0; end if;
      ready := wt.lv(fl) and (wt.lv(sl) or wt.last(fl));

      -- Current start, relative to line hp.
      relc := rel_raw;
      if rsh = '1' then
        relc := rel_raw - 32;
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
      l0 := rel_raw(6 downto 5) - ("0" & rsh);
      e0 := l0 & rel_raw(4 downto 0);
      ta := wt.t(to_integer(e0));
      ka := ta.ka(lg);
      e1 := ta.step;
      e2 := ta.s2;
      tb := wt.t(to_integer(e2));
      kb := tb.kb(lg);
      e3 := tb.step;
      e4 := tb.s2;
      t1 := wt.t(to_integer(e1));
      bt1 := t1.bt;

      -- Next start, and whether it is past the first window line (ge).
      if ka.cont = '0' then
        cnt := '0' & ka.n;
        endc := ka.endc;
        case ka.sel is
          when "00" => rn := relc; ge := rge;
          when "01" => rn := ta.bt; ge := ta.btge(lg);
          when "10" => rn := resize(e1, 18); ge := ge7(e1, lg);
          when others => rn := bt1; ge := t1.btge(lg);
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
      fx_base <= win_in(0).lc;
      fx_q <= win_in(0 to 1);

      -- Window advance. All positions here are relative to line hp; the head
      -- moves by lag at the end of this cycle.
      rsh <= lag;
      if endc = '1' then
        -- End of chunk in the first window line: the next chunk starts in
        -- the line after it.
        lag <= '1';
        if wt.lv(sl) = '1' then
          rel_raw <= to_unsigned(32 * sl, 18) + resize(wt.vlen(sl), 18);
          newchunk <= '0';
        else
          rel_raw <= to_unsigned(32 * sl, 18);
          newchunk <= '1';
        end if;
      elsif newchunk = '1' then
        lag <= '0';
        rel_raw <= to_unsigned(32 * fl, 18);
        if wt.lv(fl) = '1' then
          rel_raw <= to_unsigned(32 * fl, 18) + resize(wt.vlen(fl), 18);
          newchunk <= '0';
        end if;
      elsif ge = '1' and wt.lv(sl) = '1' and wt.last(fl) = '0' and wt.lv(fl) = '1' then
        lag <= '1';
        rel_raw <= rn;
      else
        lag <= '0';
        rel_raw <= rn;
      end if;

      -- Queue pointers.
      ncnt := qcnt;
      if lag = '1' then
        hp <= hp + 1;
        ncnt := ncnt - 1;
      end if;
      if mv2 = '1' then
        ncnt := ncnt + 1;
      end if;
      qcnt <= ncnt;

      if reset = '1' then
        qcnt <= "0000";
        hp <= "000";
        lag <= '0';
        rsh <= '0';
        newchunk <= '1';
        rel_raw <= (others => '0');
        fx_cnt <= "000";
      end if;
    end if;
  end process;

  -- Field extraction for the elements emitted in the previous cycle. They
  -- start in the two window lines.
  fx_proc: process (clk) is
    variable pe   : pent_type;
    variable wl   : std_logic;
    variable wp   : unsigned(5 downto 0);
    variable base : unsigned(19 downto 0);
  begin
    if rising_edge(clk) then
      base := fx_base & "00000";
      for k in 0 to QK-1 loop
        -- Position relative to the window.
        wp := fx_e(k)(5 downto 0);
        wp(5) := fx_e(k)(5) xor fx_sh;
        wl := wp(5);
        if wl = '0' then
          pe := fx_q(0).e(to_integer(wp(4 downto 0)));
        else
          pe := fx_q(1).e(to_integer(wp(4 downto 0)));
        end if;
        el(k).cp <= pe.cp;
        el(k).off <= pe.off;
        el(k).li <= pe.li;
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
        report "DEC c=" & integer'image(cyc) & " rel=" & integer'image(to_integer(rel_raw)) & " rsh=" & std_logic'image(rsh) & " lag=" & std_logic'image(lag)
          & " nc=" & std_logic'image(newchunk) & " qcnt=" & integer'image(to_integer(qcnt)) & " x1v=" & std_logic'image(x1v)
          & " x2v=" & std_logic'image(x2v) & " pop=" & std_logic'image(pop) & " lc=" & integer'image(to_integer(lc_next))
          & " lv=" & std_logic'image(wt.lv(0)) & std_logic'image(wt.lv(1)) & std_logic'image(wt.lv(2)) & " el_cnt=" & integer'image(to_integer(el_cnt)) & " sp=" & std_logic'image(el_space)
          & " cons=" & integer'image(to_integer(cons_pos));
      end if;
    end if;
  end process;
  -- pragma translate_on

end behavior;
