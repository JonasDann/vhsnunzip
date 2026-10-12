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
--  - long-term source (more than 31 lines back): the lines are read from
--    even/odd banks with two ports each (B, then A unless the write queue
--    reserved it). If the second line of a two-line piece can't be granted the
--    piece is cut at the line end; if the first can't, the group ends before
--    the slot.
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

    -- Long-term write queue reservation per bank (port A must stay free).
    wq_resv     : in  std_logic_vector(1 downto 0);

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

  type u7x3_type is array (1 to 3) of unsigned(6 downto 0);

  -- Element with absolute positions and precomputed copy information.
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
    o      : pos_type;    -- start (pre-pass only)
    ecp    : pos_type;    -- copy end
    e      : pos_type;    -- end
    om32   : pos_type;    -- start - 32
    em32   : pos_type;    -- end - 32
    src    : pos_type;    -- copy source start (o - off)
    srce   : pos_type;    -- copy source end (src + cp)
    stopc  : pos_type;    -- CSR cut position (o + cov)
    stopc32: pos_type;
    stopl  : pos_type;    -- end of the first long-term line (o + 32 - source lane)
    stopl32: pos_type;
    jd     : unsigned(1 downto 0);  -- CSR: source in element (this - jd); 0 = none
    part   : std_logic;   -- CSR: source in that element's literal part
    cutc   : std_logic;   -- CSR: the source part ends before this copy does
    tgts   : std_logic;   -- CSR: the source element is strideable
    lt     : std_logic;   -- long-term copy
    two    : std_logic;   -- long-term copy spanning two lines
    par    : std_logic;   -- bank (line parity) of the first source line
    ovl    : std_logic;   -- self-overlapping (off < cp)
    olt32  : std_logic;   -- off < 32
    len7   : unsigned(6 downto 0);  -- min(cp + li, 127) (pre-pass only)
    -- CSR candidates, source in element (this - d) (pre-pass only): match,
    -- in its literal part, strideable source, bytes of the copy covered.
    cm     : std_logic_vector(1 to 3);
    cpt    : std_logic_vector(1 to 3);
    ctg    : std_logic_vector(1 to 3);
    ccv    : u7x3_type;
  end record;

  constant QE_ZERO : qe_type := (
    cp => (others => '0'), off => (others => '0'), li => (others => '0'), q => (others => '0'),
    last => '0', pw2 => '0', s4 => '0', strb => '0', hascp => '0',
    o => (others => '0'), ecp => (others => '0'), e => (others => '0'),
    om32 => (others => '0'), em32 => (others => '0'),
    src => (others => '0'), srce => (others => '0'),
    stopc => (others => '0'), stopc32 => (others => '0'),
    stopl => (others => '0'), stopl32 => (others => '0'),
    jd => "00", part => '0', cutc => '0', tgts => '0', lt => '0', two => '0', par => '0',
    ovl => '0', olt32 => '0', len7 => (others => '0'),
    cm => "000", cpt => "000", ctg => "000", ccv => (others => (others => '0')));

  type qe_array is array (natural range <>) of qe_type;
  type pos_array is array (natural range <>) of pos_type;

  -- What the group former needs of an element after the FIFO.
  constant QE_BITS : natural := 16 + 17 + PW + 5 + 10*PW + 2 + 8;

  function qe_pack(e : qe_type) return std_logic_vector is
  begin
    return std_logic_vector(e.off) & std_logic_vector(e.li)
         & std_logic_vector(e.q) & e.last & e.pw2 & e.s4 & e.strb & e.hascp
         & std_logic_vector(e.ecp) & std_logic_vector(e.e)
         & std_logic_vector(e.om32) & std_logic_vector(e.em32)
         & std_logic_vector(e.src) & std_logic_vector(e.srce)
         & std_logic_vector(e.stopc) & std_logic_vector(e.stopc32)
         & std_logic_vector(e.stopl) & std_logic_vector(e.stopl32)
         & std_logic_vector(e.jd) & e.part & e.cutc & e.tgts & e.lt & e.two & e.par & e.ovl & e.olt32;
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
    e.li := take(17);
    e.q := take(PW);
    e.last := v(i-1); e.pw2 := v(i-2); e.s4 := v(i-3); e.strb := v(i-4); e.hascp := v(i-5);
    i := i - 5;
    e.ecp := take(PW);
    e.e := take(PW);
    e.om32 := take(PW);
    e.em32 := take(PW);
    e.src := take(PW);
    e.srce := take(PW);
    e.stopc := take(PW);
    e.stopc32 := take(PW);
    e.stopl := take(PW);
    e.stopl32 := take(PW);
    e.jd := unsigned(v(9 downto 8));
    e.part := v(7); e.cutc := v(6); e.tgts := v(5); e.lt := v(4);
    e.two := v(3); e.par := v(2); e.ovl := v(1); e.olt32 := v(0);
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

  -- Pre-pass stage 1: positions.
  signal p1_cnt   : unsigned(2 downto 0) := "000";
  signal p1_os    : qe_array(0 to QK-1);
  signal run_sum  : pos_type := (others => '0');

  -- Pre-pass stage 2a.
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
  signal hecpo    : pos_type := (others => '0');
  signal hecpo2   : pos_type;
  signal hfresh   : std_logic := '1';
  signal bubble   : std_logic := '0';
  signal wpos_r   : pos_type := (others => '0');
  signal resv_r   : std_logic_vector(1 downto 0) := "00";
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
    two   : std_logic_vector(0 to QK-1);
    par   : std_logic_vector(0 to QK-1);
    w     : qe_array(0 to QK-1);
  end record;
  signal sn       : snap_type;

  -- G2b output.
  type u11_array is array (natural range <>) of unsigned(10 downto 0);
  signal gb       : qgroup_type;
  signal gb_ltr   : std_logic_vector(0 to QK-1);
  signal gb_ltt   : std_logic_vector(0 to QK-1);
  signal gb_ltl   : u11_array(0 to QK-1);

begin

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

  -- Stage 1: absolute positions (running sum per chunk).
  p1_proc: process (clk) is
    variable x    : qe_type;
  begin
    if rising_edge(clk) then
      for k in 0 to QK-1 loop
        x := p0_el(k);
        x.o := run_sum + p0_ps(k);
        x.ecp := run_sum + p0_ps(k) + resize(x.cp, PW);
        x.e := run_sum + p0_pe(k);
        p1_os(k) <= x;
      end loop;
      if p0_hasl = '1' then
        run_sum <= (others => '0');
      else
        run_sum <= run_sum + p0_pe(QK-1);
      end if;
      p1_cnt <= p0_cnt;
      if reset = '1' then
        p1_cnt <= "000";
        run_sum <= (others => '0');
      end if;
    end if;
  end process;

  -----------------------------------------------------------------------------
  -- Pre-pass stage 2: copy-source location, long-term lines, stop positions.
  -----------------------------------------------------------------------------
  -- Stage 2a: per-element source positions and long-term lines.
  p2a_proc: process (clk) is
    variable x    : qe_type;
    variable slan : unsigned(5 downto 0);
  begin
    if rising_edge(clk) then
      for k in 0 to QK-1 loop
        x := p1_os(k);
        x.om32 := x.o - 32;
        x.em32 := x.e - 32;
        x.src := x.o - resize(x.off, PW);
        x.srce := x.src + resize(x.cp, PW);
        if x.off < x.cp then x.ovl := '1'; end if;
        if x.off < 32 then x.olt32 := '1'; end if;
        -- Long-term copy. Offsets up to 960 are always within the 32 lines of
        -- short-term memory, whatever the output lane; longer offsets read
        -- from long-term memory (their lines are always written by then).
        if x.off > 960 and x.cp /= 0 then
          x.lt := '1';
        end if;
        slan := '0' & x.src(4 downto 0);
        if slan + resize(x.cp, 6) > 32 or x.cp > 32 then
          x.two := '1';
        end if;
        x.par := x.src(5);
        x.stopl := x.o + 32 - resize(slan, PW);
        x.stopl32 := x.o - resize(slan, PW);
        p2a(k) <= x;
      end loop;
      p2a_cnt <= p1_cnt;
      if reset = '1' then
        p2a_cnt <= "000";
      end if;
    end if;
  end process;

  -- Stage 2b: copy-source location among the three preceding elements. A
  -- source inside the group needs off < 32 (the copy starts inside the
  -- group), so only the bytes just before the element matter: the distances
  -- back from its start to the preceding element boundaries are sums of the
  -- preceding lengths (saturated at 127). The three candidates are checked
  -- in parallel; stage 2c selects.
  p2b_proc: process (clk) is
    variable lst  : qe_array(0 to QK+2);
    variable lv   : std_logic_vector(0 to QK+2);
    variable x    : qe_type;
    variable y    : qe_type;
    variable hasl : boolean;
    type u8_arr is array (natural range <>) of unsigned(7 downto 0);
    variable bb   : u8_arr(0 to 3);
    variable sb   : unsigned(7 downto 0);
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
        bb(0) := (others => '0');
        bb(1) := resize(lst(2 + k).len7, 8);
        bb(2) := bb(1) + resize(lst(1 + k).len7, 8);
        bb(3) := bb(2) + resize(lst(k).len7, 8);
        if bb(3)(7) = '1' then bb(3) := to_unsigned(127, 8); end if;
        if bb(2)(7) = '1' then bb(2) := to_unsigned(127, 8); bb(3) := to_unsigned(127, 8); end if;
        for d in 1 to 3 loop
          y := lst(3 + k - d);
          x.cm(d) := '0';
          if lv(3 + k - d) = '1' and x.olt32 = '1' and bb(d - 1) < off8 and off8 <= bb(d) then
            x.cm(d) := '1';
          end if;
          sb := bb(d) - off8;
          if sb < y.cp then
            x.cpt(d) := '0';
            x.ctg(d) := y.strb;
            x.ccv(d) := resize(resize(y.cp, 8) - sb, 7);
          else
            x.cpt(d) := '1';
            x.ctg(d) := '0';
            x.ccv(d) := resize(off8 - bb(d - 1), 7);
          end if;
        end loop;
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
  -- most 63), and the cut position of a copy whose source part ends first.
  p2c_proc: process (clk) is
    variable x    : qe_type;
    variable cov  : unsigned(6 downto 0);
  begin
    if rising_edge(clk) then
      for k in 0 to QK-1 loop
        x := p2b(k);
        x.jd := "00";
        x.part := '0';
        x.tgts := '0';
        cov := to_unsigned(63, 7);
        for d in 1 to 3 loop
          if x.cm(d) = '1' then
            x.jd := to_unsigned(d, 2);
            x.part := x.cpt(d);
            x.tgts := x.ctg(d);
            if x.ccv(d) < 63 then
              cov := x.ccv(d);
            end if;
          end if;
        end loop;
        if cov < x.cp then x.cutc := '1'; end if;
        x.stopc := x.o + resize(cov, PW);
        x.stopc32 := x.om32 + resize(cov, PW);
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
    variable ok   : std_logic;
  begin
    if rising_edge(clk) then
      wpos_r <= lb_wpos(PW-1 downto 0);
      resv_r <= wq_resv;
      d := wpos_r(15 downto 0) - gr.lnext;
      ok := '0';
      if d(15) = '0' and d >= 320 then
        ok := '1';
      end if;
      -- Escape: the literal of the last element in the window (and so
      -- every literal before it) is completely in the buffer.
      lend := (others => '0');
      for k in 0 to QK-1 loop
        if wv(k) = '1' then
          lend := w(k).q + resize(w(k).li, PW);
        end if;
      end loop;
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
  -- precomputed position; long-term port grants don't depend on p at all
  -- (only on which elements are in the window). The next window, FIFO read
  -- pointer and head state are precomputed for every possible number of
  -- consumed elements and selected at the end.
  -----------------------------------------------------------------------------
  g2_proc: process (clk) is
    variable go       : std_logic;
    variable h        : qe_type;
    variable pe32     : pos_type;
    variable hc, hov  : std_logic;
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
    variable rqa, rqb : std_logic_vector(0 to QK-1);  -- bank 0 / bank 1 needed
    variable fpar     : std_logic_vector(0 to QK-1);
    variable cap      : u2_array(0 to 1);
    variable used     : u2_array(0 to 1);
    variable b1, b2   : integer range 0 to 1;
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

      -- Comparisons with p.
      if lt_pos(p, h.ecp) then hc := '1'; else hc := '0'; end if;
      if lt_pos(p, hecpo) and hoff < 32 then hov := '1'; else hov := '0'; end if;
      for k in 0 to QK-1 loop
        if not lt_pos(p, w(k).em32) then fits(k) := '1'; else fits(k) := '0'; end if;
        if lt_pos(w(k).om32, p) or k = 0 then inb(k) := '1'; else inb(k) := '0'; end if;
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
        if w(k).hascp = '1' then
          if w(k).strb = '1' and conf = '0' then
            str(k) := '1';
          elsif w(k).ovl = '1' then
            blk(k) := '1';
          elsif not lt_pos(w(k).src, p) then
            csr(k) := '1';
            if w(k).tgts = '1' or w(k).jd = 0 then
              blk(k) := '1';
            end if;
            if w(k).cutc = '1' and lt_pos(w(k).stopc32, p) then
              cut(k) := '1';
              stp(k) := w(k).stopc;
            end if;
          else
            hist(k) := '1';
            if lt_pos(p, w(k).srce) and w(k).olt32 = '1' then
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

      -- Long-term port requests per bank. The head uses its precomputed lines
      -- when it is fresh; a remainder conservatively takes both banks, even if
      -- only its literal is left (so the requests don't depend on p).
      for k in 0 to QK-1 loop
        if k = 0 then
          rq(k) := h.lt and not h.strb;
          if hfresh = '1' then
            fpar(k) := h.par;
            if h.two = '1' then
              rqa(k) := rq(k); rqb(k) := rq(k);
            else
              rqa(k) := rq(k) and not h.par; rqb(k) := rq(k) and h.par;
            end if;
          else
            fpar(k) := h.par;
            rqa(k) := rq(k); rqb(k) := rq(k);
          end if;
        else
          rq(k) := ltk(k);
          fpar(k) := w(k).par;
          if w(k).two = '1' then
            rqa(k) := rq(k); rqb(k) := rq(k);
          else
            rqa(k) := rq(k) and not w(k).par; rqb(k) := rq(k) and w(k).par;
          end if;
        end if;
      end loop;
      cap(0) := "10";
      cap(1) := "10";
      if resv_r(0) = '1' then cap(0) := "01"; end if;
      if resv_r(1) = '1' then cap(1) := "01"; end if;
      used(0) := "00";
      used(1) := "00";
      for k in 0 to QK-1 loop
        if k > 0 and rq(k) = '1' then
          if fpar(k) = '1' then b1 := 1; b2 := 0; else b1 := 0; b2 := 1; end if;
          if used(b1) >= cap(b1) then
            blk(k) := '1';
          elsif w(k).two = '1' and used(b2) >= cap(b2) and lt_pos(w(k).stopl32, p) then
            cut(k) := '1';
            stp(k) := w(k).stopl;
          end if;
        end if;
        if rqa(k) = '1' then used(0) := used(0) + 1; end if;
        if rqb(k) = '1' then used(1) := used(1) + 1; end if;
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
      snv.two := (others => '0');
      snv.par := (others => '0');
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
          hecpo <= hecpo2;
        end if;
      else
        hoff <= nw(0).off;
        hecpo <= nw(0).srce;
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

  -- Copy end minus twice the head offset, for when the offset doubles.
  hecpo2 <= w(0).ecp - resize(hoff & '0', PW);

  -----------------------------------------------------------------------------
  -- G2b: build the group command.
  -----------------------------------------------------------------------------
  g2b_proc: process (clk) is
    variable g     : qgroup_type;
    variable x     : qe_type;
    variable st    : pos_type;
    variable cs, ce, ls, le : pos_type;
    variable used  : u2_array(0 to 1);
    variable b1, b2: integer range 0 to 1;
    variable src   : pos_type;
    variable line  : unsigned(10 downto 0);
    variable pid   : integer range 0 to 3;
    variable lq    : pos_type;
    variable ltr   : std_logic_vector(0 to QK-1);
    variable ltt   : std_logic_vector(0 to QK-1);
    variable ltl   : u11_array(0 to QK-1);
  begin
    if rising_edge(clk) then
      g := QGROUP_ZERO;
      g.valid := sn.valid;
      g.p0 := sn.p0(15 downto 0);
      g.n := resize(sn.pe - sn.p0, 6);
      g.last := sn.last;
      g.lnext := gb.lnext;
      used(0) := "00";
      used(1) := "00";
      for k in 0 to QK-1 loop
        x := sn.w(k);
        if k = 0 then
          st := sn.p0;
        else
          st := x.om32 + 32;
        end if;
        -- Copy part [max(o, p0), min(ecp, pe)), literal part [max(ecp, p0), min(e, pe)).
        cs := st;
        if lt_pos(x.ecp, sn.pe) then ce := x.ecp; else ce := sn.pe; end if;
        if lt_pos(cs, x.ecp) then ls := x.ecp; else ls := cs; end if;
        if lt_pos(x.e, sn.pe) then le := x.e; else le := sn.pe; end if;
        g.s(k).act := sn.act(k);
        g.s(k).pos := resize(st - sn.p0, 6);
        if lt_pos(cs, ce) then
          g.s(k).c := resize(ce - cs, 6);
        else
          g.s(k).c := (others => '0');
        end if;
        if lt_pos(ls, le) then
          g.s(k).l := resize(le - ls, 6);
        else
          g.s(k).l := (others => '0');
        end if;
        g.s(k).q := x.q(15 downto 0) + resize(ls - x.ecp, 16);
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
        if g.s(k).c = 0 then
          g.s(k).ck := CK_NONE;
        end if;
        g.s(k).csrj := to_unsigned(k, 2) - x.jd;
        g.s(k).csrl := x.part;
        g.s(k).lt := sn.lt(k) and sn.hist(k);

        -- Long-term lines (ports are assigned in the next stage).
        src := st - resize(g.s(k).off, PW);
        ltl(k) := src(15 downto 5);
        ltr(k) := '0';
        ltt(k) := '0';
        if sn.act(k) = '1' and sn.hist(k) = '1' and sn.lt(k) = '1' and g.s(k).c /= 0 then
          ltr(k) := '1';
          if resize(src(4 downto 0), 6) + g.s(k).c > 32 then
            ltt(k) := '1';
          end if;
        end if;

        -- Next literal byte still needed after this group.
        if sn.act(k) = '1' then
          if lt_pos(x.ecp, sn.pe) then
            lq := sn.pe - x.ecp;
            g.lnext := x.q(15 downto 0) + lq(15 downto 0);
          else
            g.lnext := x.q(15 downto 0);
          end if;
        end if;
      end loop;
      if sn.valid = '0' then
        g.lnext := gb.lnext;
      end if;
      gb <= g;
      gb_ltr <= ltr;
      gb_ltt <= ltt;
      gb_ltl <= ltl;
      if reset = '1' then
        gb.valid <= '0';
        gb.lnext <= (others => '0');
      end if;
    end if;
  end process;

  -- G2c: long-term read port assignment (the first line of a bank gets port
  -- B, the second port A).
  g2c_proc: process (clk) is
    variable g     : qgroup_type;
    variable used  : u2_array(0 to 1);
    variable b1, b2: integer range 0 to 1;
    variable line  : unsigned(10 downto 0);
    variable pid   : integer range 0 to 3;
  begin
    if rising_edge(clk) then
      g := gb;
      g.rd_v := "0000";
      g.rd_a := (others => (others => '0'));
      used(0) := "00";
      used(1) := "00";
      for k in 0 to QK-1 loop
        if gb_ltr(k) = '1' then
          line := gb_ltl(k);
          if line(0) = '1' then b1 := 1; b2 := 0; else b1 := 0; b2 := 1; end if;
          pid := 2*b1 + to_integer(used(b1)(0 downto 0));
          g.rd_v(pid) := '1';
          g.rd_a(pid) := line(10 downto 1);
          used(b1) := used(b1) + 1;
          if gb_ltt(k) = '1' then
            line := line + 1;
            pid := 2*b2 + to_integer(used(b2)(0 downto 0));
            g.rd_v(pid) := '1';
            g.rd_a(pid) := line(10 downto 1);
            used(b2) := used(b2) + 1;
          end if;
        end if;
      end loop;
      if gb.valid = '0' then
        g.rd_v := "0000";
      end if;
      gr <= g;
      if reset = '1' then
        gr.valid <= '0';
        gr.rd_v <= "0000";
        gr.lnext <= (others => '0');
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
          & " fcnt=" & integer'image(to_integer(fcnt)) & " w0.o=" & integer'image(to_integer(w(0).o)) & " w0.e=" & integer'image(to_integer(w(0).e))
          & " w0.last=" & std_logic'image(w(0).last) & " w0.q=" & integer'image(to_integer(w(0).q)) & " w0.li=" & integer'image(to_integer(w(0).li))
          & " wpos=" & integer'image(to_integer(wpos_r)) & " lnext=" & integer'image(to_integer(gr.lnext)) & " gv=" & std_logic'image(gr.valid);
      end if;
    end if;
  end process;
  -- pragma translate_on

end behavior;
