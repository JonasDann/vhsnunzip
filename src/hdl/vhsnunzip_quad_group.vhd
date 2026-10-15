library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.vhsnunzip_int_pkg.all;
use work.vhsnunzip_quad_pkg.all;

-- Element pre-pass, element FIFO and group former for the quad datapath.
--
-- Each cycle the group former takes up to 4 elements from the head of the
-- element stream and assigns them to slots 0..3, with at most 32 output bytes
-- in total. Slot k is only used if slots 0..k-1 completed their elements. A
-- slot that cannot complete its element (budget, or one of the cuts below) is
-- the last slot of the group; the rest of its element is the head (slot 0) of
-- the next group.
--
-- To keep the loop-carried path short, every element carries its absolute
-- output positions (start O, copy end Ecp, end E), computed by a running sum
-- in the pre-pass, together with everything about its copy that does not
-- depend on where the group starts: where the copy source lies relative to
-- the three preceding elements (for copy-source resolution) and how far that
-- source extends, whether it is a long-term copy and which lines it needs.
-- The loop itself only compares these positions with the group start p and
-- p+32, decides which slots are used, and selects the group end. The group
-- command is built from a snapshot of the window in the following stage (G2b).
--
-- Copy rules (slot k, element copy starting at output position O):
--  - offset 4 or 8: stride piece (out[x] = out[x - off]); all stride pieces of
--    a group must have the same offset;
--  - self-overlapping (offset < copy length): slot 0 replicates power-of-two
--    offsets and cuts other offsets to one period (the rest continues with a
--    doubled offset); in slots > 0 the group ends before the slot;
--  - source inside the group (src = O - off >= p): copy-source resolution;
--    the piece is cut where the element that produced the source bytes ends
--    its copy or literal part; a source in a stride piece ends the group
--    before the slot;
--  - source straddling the group start: cut to the part before p;
--  - long-term source (more than 31 lines back): a group reads at most four
--    long-term lines (any four, one per read bus). If the second line of a
--    two-line piece can't be granted the piece is cut at the line end; if the
--    first can't, the group ends before the slot.
entity vhsnunzip_quad_group is
  port (
    clk         : in  std_logic;
    reset       : in  std_logic;

    -- Elements from the decoder.
    el          : in  qel_array(0 to QK-1);
    el_cnt      : in  unsigned(2 downto 0);
    el_space    : out std_logic;

    -- Compressed position up to which the literal buffer has been written.
    lb_wpos     : in  unsigned(19 downto 0);

    -- Output backpressure.
    stall       : in  std_logic;

    -- Group output (registered, R0).
    gr          : out qgroup_type
  );
end vhsnunzip_quad_group;

architecture behavior of vhsnunzip_quad_group is

  -- Output positions modulo 2^18: every comparison is between positions less
  -- than 2^17 apart (a chunk is at most 64 KiB).
  constant PW : natural := 18;
  subtype pos_type is unsigned(PW-1 downto 0);

  -- Element with absolute positions and precomputed copy information. The
  -- group former compares positions with the group start p; for a slot after
  -- the head everything follows from g = o - p (less than 32 if the element
  -- starts in the group) and small per-element constants.
  type qe_type is record
    cp     : unsigned(6 downto 0);
    off    : unsigned(15 downto 0);
    li     : unsigned(16 downto 0);
    q      : pos_type;
    last   : std_logic;
    pw2    : std_logic;
    s4     : std_logic;
    strb   : std_logic;   -- strideable (offset 4 or 8)
    hascp  : std_logic;   -- cp /= 0
    o      : pos_type;    -- start
    ecp    : pos_type;    -- copy end
    e      : pos_type;    -- end
    omc    : unsigned(6 downto 0);  -- off - cp (signed; used when off < 32)
    slan   : unsigned(4 downto 0);  -- source lane (src mod 32) (pre-pass only)
    cov    : unsigned(5 downto 0);  -- CSR: bytes of the copy the source part covers;
                                    -- long-term: 32 - slan
    jd     : unsigned(1 downto 0);  -- CSR: source in element (this - jd); 0 = none
    part   : std_logic;   -- CSR: source in that element's literal part
    cutc   : std_logic;   -- CSR: the source part ends before this copy does
    tgts   : std_logic;   -- CSR: the source element is strideable
    lt     : std_logic;   -- long-term copy
    two    : std_logic;   -- long-term copy spanning two lines
    ovl    : std_logic;   -- self-overlapping (off < cp)
    olt32  : std_logic;   -- off < 32
    len7   : unsigned(6 downto 0);  -- min(cp + li, 127) (pre-pass only)
    -- Stage 2b -> 2c, for the CSR source element (this - jd): distance back
    -- to its start and to its end, its copy length plus off, whether it is
    -- strideable.
    bbs    : unsigned(6 downto 0);
    bbe    : unsigned(6 downto 0);
    yco    : unsigned(7 downto 0);
    ystr   : std_logic;
  end record;

  constant QE_ZERO : qe_type := (
    cp => (others => '0'), off => (others => '0'), li => (others => '0'), q => (others => '0'),
    last => '0', pw2 => '0', s4 => '0', strb => '0', hascp => '0',
    o => (others => '0'), ecp => (others => '0'), e => (others => '0'),
    omc => (others => '0'), slan => (others => '0'), cov => (others => '0'),
    jd => "00", part => '0', cutc => '0', tgts => '0', lt => '0', two => '0',
    ovl => '0', olt32 => '0', len7 => (others => '0'),
    bbs => (others => '0'), bbe => (others => '0'), yco => (others => '0'), ystr => '0');

  type qe_array is array (natural range <>) of qe_type;
  type pos_array is array (natural range <>) of pos_type;

  -- What the group former needs of an element after the FIFO. The start o
  -- is left out: elements are consecutive, so the start of window slot k >= 1
  -- is the end of slot k - 1 whenever it matters (slot k is only used if
  -- slot k - 1 isn't the end of a chunk), and the head doesn't need it.
  constant QE_BITS : natural := 16 + 3*PW + 5 + 7 + 6 + 2 + 7;

  function qe_pack(e : qe_type) return std_logic_vector is
  begin
    return std_logic_vector(e.off) & std_logic_vector(e.q)
         & e.last & e.pw2 & e.s4 & e.strb & e.hascp
         & std_logic_vector(e.ecp) & std_logic_vector(e.e)
         & std_logic_vector(e.omc) & std_logic_vector(e.cov)
         & std_logic_vector(e.jd) & e.part & e.cutc & e.tgts & e.lt & e.two & e.ovl & e.olt32;
  end function;

  function qe_unpack(v : std_logic_vector(QE_BITS-1 downto 0)) return qe_type is
    variable e : qe_type := QE_ZERO;
    variable i : natural := QE_BITS;
    impure function take(n : natural) return unsigned is
    begin
      i := i - n;
      return unsigned(v(i+n-1 downto i));
    end function;
  begin
    e.off := take(16);
    e.q := take(PW);
    e.last := v(i-1); e.pw2 := v(i-2); e.s4 := v(i-3); e.strb := v(i-4); e.hascp := v(i-5);
    i := i - 5;
    e.ecp := take(PW);
    e.e := take(PW);
    e.omc := take(7);
    e.cov := take(6);
    e.jd := unsigned(v(8 downto 7));
    e.part := v(6); e.cutc := v(5); e.tgts := v(4); e.lt := v(3);
    e.two := v(2); e.ovl := v(1); e.olt32 := v(0);
    return e;
  end function;

  -- Signed comparison of positions.
  function lt_pos(a : pos_type; b : pos_type) return boolean is
    variable d : pos_type;
  begin
    d := a - b;
    return d(PW-1) = '1';
  end function;

  -- Pre-pass stage 0: lengths and in-batch prefix sums.
  signal p0_el    : qe_array(0 to QK-1);
  signal p0_ps    : pos_array(0 to QK-1);
  signal p0_pe    : pos_array(0 to QK-1);
  signal p0_cnt   : unsigned(2 downto 0) := "000";
  signal p0_hasl  : std_logic := '0';

  -- Pre-pass stage 1: positions and copy flags.
  signal run_sum  : pos_type := (others => '0');
  signal p2a      : qe_array(0 to QK-1);
  signal p2a_cnt  : unsigned(2 downto 0) := "000";

  -- Pre-pass stage 2b: CSR source element.
  signal p2b      : qe_array(0 to QK-1);
  signal p2b_cnt  : unsigned(2 downto 0) := "000";

  -- Pre-pass stage 2c: CSR cut positions.
  signal p2       : qe_array(0 to QK-1);
  signal p2_cnt   : unsigned(2 downto 0) := "000";
  signal tail     : qe_array(0 to 2) := (others => QE_ZERO);
  signal tail_v   : std_logic_vector(0 to 2) := (others => '0');

  -- Element FIFO: 4 banks of 32 entries.
  constant FD_LOG2 : natural := 5;
  type bank_mem is array (0 to 2**FD_LOG2-1) of std_logic_vector(QE_BITS-1 downto 0);
  signal fmem0, fmem1, fmem2, fmem3 : bank_mem;
  signal fwe      : std_logic_vector(0 to 3);
  signal fwa      : u5_array(0 to 3);
  signal fwd      : qe_array(0 to 3);
  type slv_qe_array is array (natural range <>) of std_logic_vector(QE_BITS-1 downto 0);
  signal frp      : slv_qe_array(0 to 3);
  signal wp       : unsigned(FD_LOG2+2 downto 0) := (others => '0');
  signal rp       : unsigned(FD_LOG2+2 downto 0) := (others => '0');
  signal fcnt     : unsigned(FD_LOG2+2 downto 0);

  -- Group former state.
  signal w        : qe_array(0 to QK-1) := (others => QE_ZERO);
  signal wv       : std_logic_vector(0 to QK-1) := (others => '0');
  signal p        : pos_type := (others => '0');
  signal hoff     : unsigned(15 downto 0) := (others => '0');
  signal hfresh   : std_logic := '1';
  signal bubble   : std_logic := '0';
  signal wpos_r   : pos_type := (others => '0');
  signal lsafe    : std_logic := '0';

  -- Snapshot for G2b.
  type snap_type is record
    valid : std_logic;
    p0    : pos_type;
    pe    : pos_type;
    last  : std_logic;
    hoff  : unsigned(15 downto 0);
    act   : std_logic_vector(0 to QK-1);
    str   : std_logic_vector(0 to QK-1);
    rep   : std_logic;
    csr   : std_logic_vector(0 to QK-1);
    hist  : std_logic_vector(0 to QK-1);
    lt    : std_logic_vector(0 to QK-1);
    cut   : std_logic_vector(0 to QK-1);   -- slot ends the group inside its element
    w     : qe_array(0 to QK-1);
  end record;
  signal sn       : snap_type;

  -- G2b output.
  type u11_array is array (natural range <>) of unsigned(10 downto 0);
  signal gb       : qgroup_type;
  signal gb_ltr   : std_logic_vector(0 to QK-1);
  signal gb_ltn   : u5_array(0 to QK-1);
  signal gb_ltl   : u11_array(0 to QK-1);

  -- Pipeline registers stay flip-flops (not shift-register LUTs).
  attribute shreg_extract : string;
  attribute shreg_extract of p0_el, p0_ps, p0_pe, p2a, p2b, p2, tail, sn, gb : signal is "no";

  -- Internal copy of the gr output, which is read back (VHDL-93 can't read an out port).
  signal gr_r     : qgroup_type;

begin

  gr <= gr_r;

  -----------------------------------------------------------------------------
  -- Pre-pass stage 1: derived flags and absolute positions.
  -----------------------------------------------------------------------------
  -- Stage 0: flags, lengths and in-batch prefix sums.
  p0_proc: process (clk) is
    variable e    : qel_type;
    variable x    : qe_type;
    variable acc  : pos_type;
  begin
    if rising_edge(clk) then
      acc := (others => '0');
      p0_hasl <= '0';
      for k in 0 to QK-1 loop
        e := qel_derive(el(k));
        x := QE_ZERO;
        x.cp := e.cp;
        x.off := e.off;
        x.li := e.li;
        x.q := e.q(PW-1 downto 0);
        x.last := e.last;
        x.pw2 := e.pw2;
        x.s4 := e.s4;
        if e.cp /= 0 then
          x.hascp := '1';
        end if;
        if e.cp /= 0 and (e.s4 = '1' or e.s8 = '1') then
          x.strb := '1';
        end if;
        if e.li(16 downto 7) /= 0 or resize(e.cp, 8) + resize(e.li(6 downto 0), 8) > 127 then
          x.len7 := (others => '1');
        else
          x.len7 := resize(e.cp + e.li(6 downto 0), 7);
        end if;
        p0_el(k) <= x;
        p0_ps(k) <= acc;
        if k < el_cnt then
          acc := acc + resize(e.cp, PW) + resize(e.li, PW);
          if e.last = '1' then
            p0_hasl <= '1';
          end if;
        end if;
        p0_pe(k) <= acc;
      end loop;
      p0_cnt <= el_cnt;
      if reset = '1' then
        p0_cnt <= "000";
      end if;
    end if;
  end process;

  -- Stage 1: absolute positions (running sum per chunk), copy flags and
  -- long-term lines (only the low bits of the source position matter).
  p1_proc: process (clk) is
    variable x    : qe_type;
    variable src  : unsigned(5 downto 0);
    variable slan : unsigned(5 downto 0);
  begin
    if rising_edge(clk) then
      for k in 0 to QK-1 loop
        x := p0_el(k);
        x.o := run_sum + p0_ps(k);
        x.ecp := run_sum + p0_ps(k) + resize(x.cp, PW);
        x.e := run_sum + p0_pe(k);
        src := run_sum(5 downto 0) + p0_ps(k)(5 downto 0) - x.off(5 downto 0);
        if x.off < x.cp then x.ovl := '1'; end if;
        if x.off < 32 then x.olt32 := '1'; end if;
        x.omc := resize(x.off(6 downto 0), 7) - x.cp;
        -- Long-term copy. Offsets up to 960 are always within the 32 lines of
        -- short-term memory, whatever the output lane; longer offsets read
        -- from long-term memory (their lines are always written by then).
        if x.off > 960 and x.cp /= 0 then
          x.lt := '1';
        end if;
        slan := '0' & src(4 downto 0);
        if slan + resize(x.cp, 6) > 32 or x.cp > 32 then
          x.two := '1';
        end if;
        x.slan := src(4 downto 0);
        p2a(k) <= x;
      end loop;
      if p0_hasl = '1' then
        run_sum <= (others => '0');
      else
        run_sum <= run_sum + p0_pe(QK-1);
      end if;
      p2a_cnt <= p0_cnt;
      if reset = '1' then
        p2a_cnt <= "000";
        run_sum <= (others => '0');
      end if;
    end if;
  end process;

  -----------------------------------------------------------------------------
  -- Pre-pass stage 2: copy-source location and stop positions.
  -----------------------------------------------------------------------------
  -- Stage 2b: copy-source location among the three preceding elements. A
  -- source inside the group needs off < 32 (the copy starts inside the
  -- group), so only the bytes just before the element matter: the distances
  -- back from its start to the preceding element boundaries are sums of the
  -- preceding lengths (saturated at 127). The three candidates are checked
  -- in parallel and the matching one is kept for stage 2c.
  p2b_proc: process (clk) is
    variable lst  : qe_array(0 to QK+2);
    variable lv   : std_logic_vector(0 to QK+2);
    variable x    : qe_type;
    variable y    : qe_type;
    variable hasl : boolean;
    type u8_arr is array (natural range <>) of unsigned(7 downto 0);
    variable bb   : u8_arr(0 to 3);
    variable s3   : unsigned(8 downto 0);
    variable off8 : unsigned(7 downto 0);
  begin
    if rising_edge(clk) then
      lst(0 to 2) := tail;
      lv(0 to 2) := tail_v;
      for k in 0 to QK-1 loop
        lst(3 + k) := p2a(k);
        if k < p2a_cnt then lv(3 + k) := '1'; else lv(3 + k) := '0'; end if;
      end loop;

      hasl := false;
      for k in 0 to QK-1 loop
        x := lst(3 + k);
        off8 := resize(x.off(4 downto 0), 8);
        -- Sums of the preceding lengths (saturated at 127), in parallel.
        s3 := resize(lst(2 + k).len7, 9) + resize(lst(1 + k).len7, 9) + resize(lst(k).len7, 9);
        bb(0) := (others => '0');
        bb(1) := resize(lst(2 + k).len7, 8);
        bb(2) := resize(lst(2 + k).len7, 8) + resize(lst(1 + k).len7, 8);
        if bb(2)(7) = '1' then bb(2) := to_unsigned(127, 8); end if;
        if s3(8 downto 7) /= 0 then bb(3) := to_unsigned(127, 8); else bb(3) := s3(7 downto 0); end if;
        -- At most one candidate matches; keep only its numbers.
        x.jd := "00";
        x.bbs := (others => '0');
        x.bbe := (others => '0');
        x.yco := (others => '0');
        x.ystr := '0';
        for d in 1 to 3 loop
          y := lst(3 + k - d);
          if lv(3 + k - d) = '1' and x.olt32 = '1' and bb(d - 1) < off8 and off8 <= bb(d) then
            x.jd := to_unsigned(d, 2);
            x.bbs := bb(d)(6 downto 0);
            x.bbe := bb(d - 1)(6 downto 0);
            x.yco := resize(y.cp, 8);
            x.ystr := y.strb;
          end if;
        end loop;
        x.yco := x.yco + off8;
        p2b(k) <= x;
        if lv(3 + k) = '1' and x.last = '1' then
          hasl := true;
        end if;
      end loop;
      p2b_cnt <= p2a_cnt;

      -- Keep the last three elements as predecessors for the next batch,
      -- unless a chunk ended.
      for t in 0 to 2 loop
        tail(t) <= lst(to_integer(p2a_cnt) + t);
        tail_v(t) <= lv(to_integer(p2a_cnt) + t);
      end loop;
      if hasl then
        tail_v <= "000";
      end if;
      if reset = '1' then
        p2b_cnt <= "000";
        tail_v <= "000";
      end if;
    end if;
  end process;

  -- Stage 2c: CSR source element and how far its part covers the copy (at
  -- most 63).
  p2c_proc: process (clk) is
    variable x    : qe_type;
    variable cv   : unsigned(5 downto 0);
    variable off8 : unsigned(7 downto 0);
    variable bp   : unsigned(7 downto 0);
    variable ccv  : unsigned(6 downto 0);
  begin
    if rising_edge(clk) then
      for k in 0 to QK-1 loop
        x := p2b(k);
        -- Source in element (this - jd), which starts bbs bytes back and ends
        -- bbe bytes back: it is in that element's copy part if bbs < cp(that)
        -- + off, which then covers cp(that) + off - bbs bytes of this copy;
        -- otherwise in its literal part, which covers off - bbe bytes.
        off8 := resize(x.off(4 downto 0), 8);
        bp := resize(x.bbe, 8);
        if resize(x.bbs, 8) < x.yco then
          x.part := '0';
          x.tgts := x.ystr;
          ccv := resize(x.yco - resize(x.bbs, 8), 7);
        else
          x.part := '1';
          x.tgts := '0';
          ccv := resize(off8 - bp, 7);
        end if;
        -- Coverage (at most 63) and whether it ends before the copy does.
        if ccv < 63 then cv := ccv(5 downto 0); else cv := to_unsigned(63, 6); end if;
        if x.jd = 0 then
          x.part := '0';
          x.tgts := '0';
          cv := to_unsigned(63, 6);
        end if;
        x.cov := cv;
        if resize(cv, 7) < x.cp then x.cutc := '1'; else x.cutc := '0'; end if;
        -- A long-term copy never resolves inside the group; its cut length is
        -- the rest of the first source line instead.
        if x.lt = '1' then
          x.cov := 32 - resize(x.slan, 6);
        end if;
        p2(k) <= x;
      end loop;
      p2_cnt <= p2b_cnt;
      if reset = '1' then
        p2_cnt <= "000";
      end if;
    end if;
  end process;

  -----------------------------------------------------------------------------
  -- Element FIFO.
  -----------------------------------------------------------------------------
  fcnt <= wp - rp;

  fifo_wr_comb: process (p2, p2_cnt, wp) is
    variable i   : unsigned(1 downto 0);
    variable idx : unsigned(FD_LOG2+2 downto 0);
  begin
    for b in 0 to 3 loop
      i := to_unsigned(b, 2) - wp(1 downto 0);
      idx := wp + resize(i, FD_LOG2+3);
      if i < p2_cnt then
        fwe(b) <= '1';
      else
        fwe(b) <= '0';
      end if;
      fwa(b) <= resize(idx(FD_LOG2+1 downto 2), 5);
      fwd(b) <= p2(to_integer(i));
    end loop;
  end process;

  fifo_wr_proc: process (clk) is
  begin
    if rising_edge(clk) then
      if fwe(0) = '1' then fmem0(to_integer(fwa(0)(FD_LOG2-1 downto 0))) <= qe_pack(fwd(0)); end if;
      if fwe(1) = '1' then fmem1(to_integer(fwa(1)(FD_LOG2-1 downto 0))) <= qe_pack(fwd(1)); end if;
      if fwe(2) = '1' then fmem2(to_integer(fwa(2)(FD_LOG2-1 downto 0))) <= qe_pack(fwd(2)); end if;
      if fwe(3) = '1' then fmem3(to_integer(fwa(3)(FD_LOG2-1 downto 0))) <= qe_pack(fwd(3)); end if;
      wp <= wp + p2_cnt;
      if reset = '1' then
        wp <= (others => '0');
      end if;
    end if;
  end process;

  fifo_rd_comb: process (fmem0, fmem1, fmem2, fmem3, rp) is
    variable idx : unsigned(FD_LOG2+2 downto 0);
    variable a   : integer range 0 to 2**FD_LOG2-1;
  begin
    for b in 0 to 3 loop
      -- Bank b holds element rp + ((b - rp) mod 4).
      idx := rp + resize(to_unsigned(b, 2) - rp(1 downto 0), FD_LOG2+3);
      a := to_integer(idx(FD_LOG2+1 downto 2));
      case b is
        when 0 => frp(0) <= fmem0(a);
        when 1 => frp(1) <= fmem1(a);
        when 2 => frp(2) <= fmem2(a);
        when others => frp(3) <= fmem3(a);
      end case;
    end loop;
  end process;

  -- Room for one more decoder batch: 128 entries minus the window refill and
  -- the batches in flight.
  space_proc: process (clk) is
  begin
    if rising_edge(clk) then
      if fcnt < 2**(FD_LOG2+2) - 36 then
        el_space <= '1';
      else
        el_space <= '0';
      end if;
      if reset = '1' then
        el_space <= '0';
      end if;
    end if;
  end process;

  -----------------------------------------------------------------------------
  -- Literal availability: a group reads literal bytes within 64 bytes of the
  -- current literal position, which is at most two groups ahead of the last
  -- issued group's lnext. Otherwise wait until every literal in the window is
  -- in the buffer.
  -----------------------------------------------------------------------------
  lsafe_proc: process (clk) is
    variable d    : unsigned(15 downto 0);
    variable d20  : pos_type;
    variable lend : pos_type;
    variable lq, le, lc : pos_type;
    variable ok   : std_logic;
  begin
    if rising_edge(clk) then
      wpos_r <= lb_wpos(PW-1 downto 0);
      d := wpos_r(15 downto 0) - gr_r.lnext;
      ok := '0';
      if d(15) = '0' and d >= 320 then
        ok := '1';
      end if;
      -- Escape: the literal of the last element in the window (and so
      -- every literal before it) is completely in the buffer.
      lq := (others => '0');
      le := (others => '0');
      lc := (others => '0');
      for k in 0 to QK-1 loop
        if wv(k) = '1' then
          lq := w(k).q;
          le := w(k).e;
          lc := w(k).ecp;
        end if;
      end loop;
      lend := lq + le - lc;
      d20 := wpos_r - lend;
      if wv(0) = '1' and d20(PW-1) = '0' then
        ok := '1';
      end if;
      lsafe <= ok;
      if reset = '1' then
        lsafe <= '0';
      end if;
    end if;
  end process;

  -----------------------------------------------------------------------------
  -- G2: group formation loop.
  --
  -- Everything that depends on the group start p is a comparison of p with a
  -- precomputed position; long-term read grants only depend on p through the
  -- line count of a partially done head. The next window, FIFO read pointer
  -- and head state are precomputed for every possible number of consumed
  -- elements and selected at the end.
  -----------------------------------------------------------------------------
  g2_proc: process (clk) is
    variable go       : std_logic;
    variable h        : qe_type;
    variable pe32     : pos_type;
    variable hc, hov  : std_logic;
    variable r0, u    : pos_type;
    variable g        : pos_array(0 to QK-1);
    variable gl       : unsigned(4 downto 0);
    variable fits     : std_logic_vector(0 to QK-1);
    variable inb      : std_logic_vector(0 to QK-1);
    variable str      : std_logic_vector(0 to QK-1);
    variable csr      : std_logic_vector(0 to QK-1);
    variable hist     : std_logic_vector(0 to QK-1);
    variable ltk      : std_logic_vector(0 to QK-1);
    variable blk      : std_logic_vector(0 to QK-1);
    variable cut      : std_logic_vector(0 to QK-1);
    variable fin      : std_logic_vector(0 to QK-1);
    variable act      : std_logic_vector(0 to QK-1);
    variable lastk    : std_logic_vector(0 to QK-1);
    variable stp      : pos_array(0 to QK-1);
    variable rep0     : std_logic;
    variable dbl0     : std_logic;
    variable conf     : std_logic;
    variable rq       : std_logic_vector(0 to QK-1);
    variable rq2      : std_logic_vector(0 to QK-1);  -- two lines requested
    variable used     : unsigned(2 downto 0);
    variable hs5      : unsigned(4 downto 0);
    variable pend     : pos_type;
    variable glast    : std_logic;
    variable partial  : std_logic;
    variable moh      : std_logic_vector(0 to QK);
    variable nv       : integer range 0 to QK;
    variable fc       : integer range 0 to QK;
    variable nw       : qe_array(0 to QK-1);
    variable nwv      : std_logic_vector(0 to QK-1);
    variable nrp      : unsigned(FD_LOG2+2 downto 0);
    variable cand     : unsigned(FD_LOG2+2 downto 0);
    variable csrc     : slv_qe_array(0 to 2*QK-1);
    variable ws       : u3_array(0 to QK-1);
    variable code     : unsigned(2 downto 0);
    variable keep     : integer range 0 to QK;
    variable pop      : integer range 0 to QK;
    variable vcnt     : integer range 0 to 2*QK;
    variable snv      : snap_type;
  begin
    if rising_edge(clk) then

      go := wv(0) and not stall and not bubble and lsafe;
      pe32 := p + 32;
      h := w(0);

      -- Comparisons with p. Head: copy bytes left (r0). Slots 1..3: g = o - p,
      -- with o the end of the slot before (the slot starts in the group if
      -- g < 32); all their other conditions are small comparisons with g.
      r0 := h.ecp - p;
      if r0 /= 0 and r0(PW-1) = '0' then hc := '1'; else hc := '0'; end if;
      if hoff < 32 and r0(PW-1) = '0' and (r0(PW-2 downto 5) /= 0 or r0(4 downto 0) > hoff(4 downto 0)) then
        hov := '1';
      else
        hov := '0';
      end if;
      for k in 0 to QK-1 loop
        u := w(k).e - p;
        if u(PW-1 downto 6) = 0 and u(5 downto 0) <= 32 then fits(k) := '1'; else fits(k) := '0'; end if;
        if k = 0 then
          g(k) := (others => '0');
        else
          g(k) := w(k-1).e - p;
        end if;
        if g(k)(PW-1 downto 5) = 0 or k = 0 then inb(k) := '1'; else inb(k) := '0'; end if;
      end loop;

      str := (others => '0');
      csr := (others => '0');
      hist := (others => '0');
      ltk := (others => '0');
      blk := (others => '0');
      cut := (others => '0');
      for k in 0 to QK-1 loop
        stp(k) := pe32;
      end loop;

      -- Slot 0: the head element, possibly partially done.
      rep0 := '0';
      dbl0 := '0';
      if hc = '1' then
        if h.strb = '1' then
          str(0) := '1';
        elsif hov = '1' and h.pw2 = '1' then
          rep0 := '1';
        else
          hist(0) := '1';
          ltk(0) := h.lt;
          if hov = '1' then
            dbl0 := '1';
            cut(0) := '1';
            stp(0) := p + resize(hoff(4 downto 0), PW);
          end if;
        end if;
      end if;

      -- Slots 1..3.
      for k in 1 to QK-1 loop
        conf := '0';
        for j in 0 to k-1 loop
          if w(j).strb = '1' and w(j).s4 /= w(k).s4 then
            conf := '1';
          end if;
        end loop;
        gl := g(k)(4 downto 0);
        if w(k).hascp = '1' then
          if w(k).strb = '1' and conf = '0' then
            str(k) := '1';
          elsif w(k).ovl = '1' then
            blk(k) := '1';
          elsif w(k).olt32 = '1' and gl >= w(k).off(4 downto 0) then
            -- Source inside the group.
            csr(k) := '1';
            if w(k).tgts = '1' or w(k).jd = 0 then
              blk(k) := '1';
            end if;
            if w(k).cutc = '1' and resize(gl, 7) + resize(w(k).cov, 7) < 32 then
              cut(k) := '1';
              stp(k) := w(k-1).e + resize(w(k).cov, PW);
            end if;
          else
            hist(k) := '1';
            -- The source straddles the group start (src + cp > p).
            if w(k).olt32 = '1' and (w(k).omc(6) = '1' or gl > w(k).omc(4 downto 0)) then
              cut(k) := '1';
              stp(k) := p + resize(w(k).off(4 downto 0), PW);
            end if;
          end if;
        end if;
        -- Long-term copies (offset > 960) never straddle or resolve inside the
        -- group, so this doesn't depend on p.
        if w(k).lt = '1' and not (w(k).strb = '1' and conf = '0') and w(k).ovl = '0' then
          ltk(k) := '1';
        end if;
      end loop;

      -- Long-term line requests. The head uses its precomputed line count
      -- when it is fresh; for a remainder it follows from the copy bytes left
      -- (r0) and the source lane (p - off), which only takes the low bits of
      -- r0 besides the comparisons made for hc and hov.
      for k in 0 to QK-1 loop
        if k = 0 then
          rq(k) := h.lt and not h.strb;
          if hfresh = '1' then
            rq2(k) := rq(k) and h.two;
          else
            hs5 := p(4 downto 0) - hoff(4 downto 0);
            rq(k) := rq(k) and hc;
            if hs5 /= 0 and (r0(PW-2 downto 5) /= 0 or resize(r0(4 downto 0), 6) + resize(hs5, 6) > 32) then
              rq2(k) := rq(k);
            else
              rq2(k) := '0';
            end if;
          end if;
        else
          rq(k) := ltk(k);
          rq2(k) := rq(k) and w(k).two;
        end if;
      end loop;
      -- At most four lines per group. The head always gets its lines; used
      -- saturates at 4 (a slot that doesn't get its first line ends the group).
      used := "000";
      for k in 0 to QK-1 loop
        if k > 0 and rq(k) = '1' then
          if used(2) = '1' then
            blk(k) := '1';
          elsif w(k).two = '1' and used = 3 and resize(g(k)(4 downto 0), 7) + resize(w(k).cov, 7) < 32 then
            -- The first source line ends inside the group.
            cut(k) := '1';
            stp(k) := w(k-1).e + resize(w(k).cov, PW);
          end if;
        end if;
        if used(2) = '0' then
          if rq2(k) = '1' and used /= 3 then
            used := used + 2;
          elsif rq(k) = '1' then
            used := used + 1;
          end if;
        end if;
      end loop;

      for k in 0 to QK-1 loop
        fin(k) := fits(k) and not cut(k);
      end loop;

      -- Usable slots.
      act(0) := go;
      for k in 1 to QK-1 loop
        act(k) := act(k-1) and fin(k-1) and not w(k-1).last and wv(k) and inb(k) and not blk(k);
      end loop;
      for k in 0 to QK-1 loop
        if k = QK-1 then
          lastk(k) := act(k);
        else
          lastk(k) := act(k) and not act(k+1);
        end if;
      end loop;

      -- Group end, consumed elements (one-hot), chunk end. All selections
      -- are one-hot AND-OR multiplexers.
      pend := (others => '0');
      glast := '0';
      partial := '0';
      moh := (others => '0');
      moh(0) := not act(0);
      for k in 0 to QK-1 loop
        for b in 0 to PW-1 loop
          pend(b) := pend(b) or (lastk(k) and fin(k) and w(k).e(b))
                             or (lastk(k) and not fin(k) and stp(k)(b));
        end loop;
        moh(k + 1) := moh(k + 1) or (lastk(k) and fin(k));
        moh(k) := moh(k) or (lastk(k) and not fin(k));
        partial := partial or (lastk(k) and not fin(k));
        glast := glast or (act(k) and w(k).last);
      end loop;

      -- Snapshot for G2b.
      snv.valid := act(0);
      snv.p0 := p;
      snv.pe := pend;
      snv.last := glast;
      snv.hoff := hoff;
      snv.act := act;
      snv.str := str;
      snv.rep := rep0;
      snv.csr := csr;
      snv.hist := hist;
      snv.lt := ltk;
      for k in 0 to QK-1 loop
        snv.cut(k) := lastk(k) and not fin(k);
      end loop;
      snv.w := w;
      sn <= snv;

      -- Next group start.
      bubble <= '0';
      if act(0) = '1' then
        if glast = '1' then
          p <= (others => '0');
          bubble <= '1';
        else
          p <= pend;
        end if;
      end if;

      -- Next window: slot i takes element i + m of (W ++ FIFO), where m is
      -- the number of consumed elements (one-hot moh). Per slot that is one
      -- binary select over the window entries (0 .. 3) and the FIFO bank
      -- outputs (4 .. 7; FIFO element t is in bank (rp + t) mod 4).
      nv := 0;
      for k in 0 to QK-1 loop
        if wv(k) = '1' then
          nv := k + 1;
        end if;
      end loop;
      if fcnt >= QK then fc := QK; else fc := to_integer(fcnt); end if;
      nwv := (others => '0');
      nrp := (others => '0');
      ws := (others => "000");
      for m in 0 to QK loop
        if m <= nv then
          keep := nv - m;
        else
          keep := 0;
        end if;
        if QK - keep < fc then pop := QK - keep; else pop := fc; end if;
        vcnt := keep + pop;
        cand := rp + pop;
        for b in 0 to FD_LOG2+2 loop
          nrp(b) := nrp(b) or (moh(m) and cand(b));
        end loop;
        for i in 0 to QK-1 loop
          if i < vcnt then
            nwv(i) := nwv(i) or moh(m);
          end if;
          if i + m < nv then
            code := to_unsigned(i + m, 3);
          else
            code := '1' & (rp(1 downto 0) + to_unsigned((i + m - nv) mod 4, 2));
          end if;
          for b in 0 to 2 loop
            ws(i)(b) := ws(i)(b) or (moh(m) and code(b));
          end loop;
        end loop;
      end loop;
      for j in 0 to QK-1 loop
        csrc(j) := qe_pack(w(j));
        csrc(QK + j) := frp(j);
      end loop;
      for i in 0 to QK-1 loop
        nw(i) := qe_unpack(csrc(to_integer(ws(i))));
      end loop;
      w <= nw;
      wv <= nwv;
      rp <= nrp;

      -- Head state.
      if moh(0) = '1' and wv(0) = '1' then
        if act(0) = '1' then
          hfresh <= '0';
        end if;
        if dbl0 = '1' then
          hoff <= hoff(14 downto 0) & '0';
        end if;
      else
        hoff <= nw(0).off;
        hfresh <= not partial;
      end if;

      if reset = '1' then
        wv <= (others => '0');
        p <= (others => '0');
        bubble <= '0';
        rp <= (others => '0');
        sn.valid <= '0';
        hfresh <= '1';
      end if;
    end if;
  end process;

  -----------------------------------------------------------------------------
  -- G2b: build the group command.
  -----------------------------------------------------------------------------
  -- Piece boundaries relative to the group start (all within 64 bytes of
  -- it, except for the head, whose copy part may have ended long before).
  g2b_proc: process (clk) is
    variable g     : qgroup_type;
    variable x     : qe_type;
    variable n     : unsigned(7 downto 0);
    variable a     : unsigned(7 downto 0);   -- slot start
    variable ce    : unsigned(7 downto 0);   -- copy end
    variable le    : unsigned(7 downto 0);   -- literal end
    variable t     : unsigned(7 downto 0);
    variable r0    : pos_type;
    variable cdone : boolean;                 -- head: copy part ended before the group
    variable cz    : boolean;                 -- no copy part in the group
    variable dl    : pos_type;
    variable lq    : unsigned(15 downto 0);   -- last used slot: literal position,
    variable lc    : pos_type;                -- and copy end
    variable src   : pos_type;
    variable xo    : pos_type;                -- start of slot k >= 1
    variable ltr   : std_logic_vector(0 to QK-1);
    variable ltl   : u11_array(0 to QK-1);
    variable ltn   : u5_array(0 to QK-1);
  begin
    if rising_edge(clk) then
      g := QGROUP_ZERO;
      g.valid := sn.valid;
      g.p0 := sn.p0(15 downto 0);
      n := resize(sn.pe(5 downto 0) - sn.p0(5 downto 0), 8);
      g.n := n(5 downto 0);
      g.last := sn.last;
      g.lnext := gb.lnext;
      lq := sn.w(0).q(15 downto 0);
      lc := sn.w(0).ecp;
      for k in 0 to QK-1 loop
        x := sn.w(k);
        -- An element that doesn't end the group ends inside it.
        if sn.cut(k) = '1' then
          le := n;
        else
          le := resize(x.e(6 downto 0) - sn.p0(6 downto 0), 8);
        end if;
        if k = 0 then
          a := (others => '0');
          r0 := x.ecp - sn.p0;
          cdone := r0(PW-1) = '1' or r0 = 0;
          if cdone then
            ce := (others => '0');
          elsif r0(PW-2 downto 7) /= 0 then
            ce := (others => '1');
          else
            ce := '0' & r0(6 downto 0);
          end if;
        else
          xo := sn.w(k-1).e;
          a := resize(xo(5 downto 0) - sn.p0(5 downto 0), 8);
          ce := x.ecp(7 downto 0) - sn.p0(7 downto 0);
          cdone := false;
        end if;
        if k = 0 then
          cz := cdone or n = 0;
        else
          cz := x.hascp = '0' or n <= a;
        end if;
        g.s(k).act := sn.act(k);
        g.s(k).pos := a(5 downto 0);
        -- Copy part [a, min(ce, n)), literal part [max(ce, a), min(le, n)).
        if ce < n then t := ce; else t := n; end if;
        if t > a then
          g.s(k).c := resize(t - a, 6);
        else
          g.s(k).c := (others => '0');
        end if;
        if le > ce then
          g.s(k).l := resize(le - ce, 6);
        else
          g.s(k).l := (others => '0');
        end if;
        if cdone then
          -- The literal part starts at the group start.
          g.s(k).q := x.q(15 downto 0) - r0(15 downto 0);
        else
          g.s(k).q := x.q(15 downto 0);
        end if;
        if k = 0 then
          g.s(k).off := sn.hoff;
        else
          g.s(k).off := x.off;
        end if;
        g.s(k).ck := CK_NONE;
        if sn.str(k) = '1' then
          g.s(k).ck := CK_STRIDE;
        elsif k = 0 and sn.rep = '1' then
          g.s(k).ck := CK_REP;
        elsif sn.csr(k) = '1' then
          g.s(k).ck := CK_CSR;
        elsif sn.hist(k) = '1' then
          g.s(k).ck := CK_HIST;
        end if;
        if cz then
          g.s(k).ck := CK_NONE;
        end if;
        g.s(k).csrj := to_unsigned(k, 2) - x.jd;
        g.s(k).csrl := x.part;
        g.s(k).lt := sn.lt(k) and sn.hist(k);

        -- Long-term lines (ports are assigned in the next stage).
        if k = 0 then
          src := sn.p0 - resize(g.s(k).off, PW);
        else
          src := xo - resize(g.s(k).off, PW);
        end if;
        g.s(k).src := src(15 downto 0);
        ltl(k) := src(15 downto 5);
        ltn(k) := src(4 downto 0);
        ltr(k) := '0';
        if sn.act(k) = '1' and sn.hist(k) = '1' and sn.lt(k) = '1' and not cz then
          ltr(k) := '1';
        end if;

        -- The last used slot decides the next literal byte.
        if sn.act(k) = '1' then
          lq := x.q(15 downto 0);
          lc := x.ecp;
        end if;
      end loop;
      -- Next literal byte still needed after this group: q + (pe - ecp) if
      -- the copy part ends before the group end.
      dl := lc - sn.pe;
      if dl(PW-1) = '1' then
        g.lnext := lq + sn.pe(15 downto 0) - lc(15 downto 0);
      else
        g.lnext := lq;
      end if;
      if sn.valid = '0' then
        g.lnext := gb.lnext;
      end if;
      gb <= g;
      gb_ltr <= ltr;
      gb_ltl <= ltl;
      gb_ltn <= ltn;
      if reset = '1' then
        gb.valid <= '0';
        gb.lnext <= (others => '0');
      end if;
    end if;
  end process;

  -- G2c: long-term read bus assignment, in slot order (the head gets buses 0
  -- and 1).
  g2c_proc: process (clk) is
    variable g     : qgroup_type;
    variable n     : unsigned(2 downto 0);
    variable line  : unsigned(10 downto 0);
  begin
    if rising_edge(clk) then
      g := gb;
      g.rd_v := "0000";
      g.rd_a := (others => (others => '0'));
      n := "000";
      for k in 0 to QK-1 loop
        g.s(k).ltb := n(1 downto 0);
        if gb_ltr(k) = '1' then
          -- pragma translate_off
          assert gb.valid = '0' or n /= 4 report "long-term read bus overflow" severity failure;
          -- pragma translate_on
          line := gb_ltl(k);
          g.rd_v(to_integer(n(1 downto 0))) := '1';
          g.rd_a(to_integer(n(1 downto 0))) := line;
          n := n + 1;
          -- The piece spans two lines.
          if resize(gb_ltn(k), 6) + gb.s(k).c > 32 then
            -- pragma translate_off
            assert gb.valid = '0' or n /= 4 report "long-term read bus overflow" severity failure;
            -- pragma translate_on
            g.rd_v(to_integer(n(1 downto 0))) := '1';
            g.rd_a(to_integer(n(1 downto 0))) := line + 1;
            n := n + 1;
          end if;
        end if;
      end loop;
      if gb.valid = '0' then
        g.rd_v := "0000";
      end if;
      gr_r <= g;
      if reset = '1' then
        gr_r.valid <= '0';
        gr_r.rd_v <= "0000";
        gr_r.lnext <= (others => '0');
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
        report "GRP c=" & integer'image(cyc) & " p=" & integer'image(to_integer(p)) & " wv=" & std_logic'image(wv(0)) & std_logic'image(wv(1))
          & " lsafe=" & std_logic'image(lsafe) & " stall=" & std_logic'image(stall) & " bub=" & std_logic'image(bubble)
          & " fcnt=" & integer'image(to_integer(fcnt)) & " w0.e=" & integer'image(to_integer(w(0).e))
          & " w0.last=" & std_logic'image(w(0).last) & " w0.q=" & integer'image(to_integer(w(0).q))
          & " wpos=" & integer'image(to_integer(wpos_r)) & " lnext=" & integer'image(to_integer(gr_r.lnext)) & " gv=" & std_logic'image(gr_r.valid);
      end if;
    end if;
  end process;
  -- pragma translate_on

end behavior;
