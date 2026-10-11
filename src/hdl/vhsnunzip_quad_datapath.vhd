library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.vhsnunzip_int_pkg.all;
use work.vhsnunzip_quad_pkg.all;

-- Datapath of the quad core. Fixed-latency pipeline (no stalls):
--
--   R0  : group from the group former (long-term reads are issued from R0)
--   G3a : direct sources -> R0b
--   G3b : copy-source resolution -> R1
--   S1a : per-slot rotations, source lines, ports -> R2
--   S1b : per-lane control -> R3
--   S2a : short-term/literal/long-term reads, rotators, merge -> R4
--   S2b : recent-history fill (copies from the last 32 bytes, replicated
--         slot 0), stride fill, short-term write, output line
--
-- History is split in three by age, relative to the group start p0:
--  - [p0-32, p0): the "recent" register LW, which holds the most recent byte
--    of every lane. Read per output lane in S2b, so it already contains the
--    previous group's output.
--  - older, up to 31 lines back: short-term memory (per-lane LUTRAM, written
--    in S2b, read in S2a; by then every byte older than p0-32 is written).
--  - older still: long-term memory (URAM, even/odd line banks).
entity vhsnunzip_quad_datapath is
  port (
    clk         : in  std_logic;
    reset       : in  std_logic;

    -- Group from the group former (R0).
    gr          : in  qgroup_type;

    -- Literal buffer write port (from the decoder).
    lb_we       : in  std_logic;
    lb_line     : in  unsigned(QI-1 downto 0);
    lb_data     : in  byte_array(0 to QB-1);

    -- Oldest literal byte still needed (to the decoder).
    cons_pos    : out unsigned(15 downto 0);

    -- Long-term read data, valid in S2a (4 cycles after R0).
    lt_rdat     : in  byte_matrix(0 to 3);

    -- Completed output lines, for the long-term memory.
    ln_valid    : out std_logic;
    ln_data     : out byte_array(0 to QB-1);
    ln_line     : out unsigned(10 downto 0);

    -- Output stream.
    out_valid   : out std_logic;
    out_data    : out byte_array(0 to QB-1);
    out_cnt     : out unsigned(QI downto 0);
    out_last    : out std_logic
  );
end vhsnunzip_quad_datapath;

architecture behavior of vhsnunzip_quad_datapath is

  -- R0b: group plus direct sources.
  signal r0       : qgroup_type := QGROUP_ZERO;
  signal r0res    : qres_array(0 to QK-1);
  signal r0c      : std_logic_vector(0 to QK-1);
  signal r0d      : u6_array(0 to QK-1);

  -- R1: group plus resolved sources.
  signal r1       : qgroup_type := QGROUP_ZERO;
  signal r1res    : qres_array(0 to QK-1);

  -- Stride source per lane: one-hot over the distance 4*i, i = 1 .. 8.
  type sl8_array is array (natural range <>) of std_logic_vector(1 to 8);

  -- R2: per-slot scalars.
  type s1a_slot_type is record
    act   : std_logic;
    pos   : unsigned(5 downto 0);
    c     : unsigned(5 downto 0);
    l     : unsigned(5 downto 0);
    rk    : std_logic_vector(2 downto 0);
    rsl   : unsigned(1 downto 0);
    rot   : unsigned(4 downto 0);
    srcl  : unsigned(4 downto 0);
    sline : unsigned(4 downto 0);
    pid1  : unsigned(1 downto 0);
    pid2  : unsigned(1 downto 0);
    dist  : unsigned(6 downto 0);
    off   : unsigned(4 downto 0);
    sh    : unsigned(4 downto 0);
    qlane : unsigned(4 downto 0);
    qline : unsigned(4 downto 0);
    lrot  : unsigned(4 downto 0);
  end record;
  type s1a_slot_array is array (natural range <>) of s1a_slot_type;

  type s1a_type is record
    valid : std_logic;
    p0l   : unsigned(4 downto 0);
    n     : unsigned(5 downto 0);
    last  : std_logic;
    line0 : unsigned(10 downto 0);
    lnext : unsigned(15 downto 0);
    d8    : std_logic;
    strl  : std_logic_vector(0 to QB-1);  -- lane is in a stride piece
    s     : s1a_slot_array(0 to QK-1);
  end record;
  signal r2       : s1a_type;
  signal r2_valid : std_logic := '0';

  -- R3: per-lane control.
  type s1b_type is record
    valid : std_logic;
    p0l   : unsigned(4 downto 0);
    n     : unsigned(5 downto 0);
    last  : std_logic;
    line0 : unsigned(10 downto 0);
    lnext : unsigned(15 downto 0);
    -- per slot
    rk    : ck_array(0 to QK-1);
    rsl   : u2_array(0 to QK-1);
    rot   : u5_array(0 to QK-1);
    lrot  : u5_array(0 to QK-1);
    st_a  : u5_matrix(0 to QK-1);
    li_a  : u5_matrix(0 to QK-1);
    lt_p  : u2_matrix(0 to QK-1);
    -- per output lane
    ing   : std_logic_vector(0 to QB-1);
    own   : u3_array(0 to QB-1);
    rec   : std_logic_vector(0 to QB-1);
    idx   : u5_array(0 to QB-1);
    str   : std_logic_vector(0 to QB-1);
    slw   : std_logic_vector(0 to QB-1);
    sone  : sl8_array(0 to QB-1);
  end record;
  signal r3       : s1b_type;
  signal r3_valid : std_logic := '0';

  -- R4: merged data plus S2b control.
  type s2a_type is record
    p0l   : unsigned(4 downto 0);
    n     : unsigned(5 downto 0);
    last  : std_logic;
    line0 : unsigned(10 downto 0);
    m     : byte_array(0 to QB-1);
    ing   : std_logic_vector(0 to QB-1);
    rec   : std_logic_vector(0 to QB-1);
    idx   : u5_array(0 to QB-1);
    str   : std_logic_vector(0 to QB-1);
    slw   : std_logic_vector(0 to QB-1);
    sone  : sl8_array(0 to QB-1);
  end record;
  signal r4       : s2a_type;
  signal r4_valid : std_logic := '0';

  -- Memory read results in S2a.
  signal st_rd    : byte_matrix(0 to QK-1);
  signal lb_rd    : byte_matrix(0 to QK-1);

  -- S2b.
  signal lw       : byte_array(0 to QB-1) := (others => X"00");
  signal fin      : byte_array(0 to QB-1);
  signal st_we    : std_logic_vector(0 to QB-1);
  signal st_wa    : u5_array(0 to QB-1);
  signal pend     : std_logic := '0';
  signal pend_cnt : unsigned(QI downto 0);

  function pid_of(line : unsigned(10 downto 0); g : qgroup_type) return unsigned is
    variable b : integer range 0 to 1;
  begin
    if line(0) = '1' then b := 1; else b := 0; end if;
    if g.rd_v(2*b) = '1' and g.rd_a(2*b) = line(10 downto 1) then
      return to_unsigned(2*b, 2);
    else
      return to_unsigned(2*b + 1, 2);
    end if;
  end function;

begin

  -----------------------------------------------------------------------------
  -- G3: copy-source resolution.
  -----------------------------------------------------------------------------
  -- G3a: direct sources, and for copies resolved into an earlier slot's copy
  -- part, the source slot and the offset within it.
  g3a_proc: process (clk) is
    variable res   : qres_array(0 to QK-1);
    variable o     : unsigned(15 downto 0);
    variable srel  : unsigned(5 downto 0);
    variable delta : unsigned(5 downto 0);
    variable j     : integer range 0 to QK-1;
  begin
    if rising_edge(clk) then
      for k in 0 to QK-1 loop
        o := gr.p0 + resize(gr.s(k).pos, 16);
        res(k).rk := RK_NONE;
        res(k).src := o - gr.s(k).off;
        res(k).rsl := to_unsigned(k, 2);
        res(k).per := gr.s(k).off(4 downto 0);
        res(k).sh := (others => '0');
        r0c(k) <= '0';
        srel := gr.s(k).pos - gr.s(k).off(5 downto 0);
        j := to_integer(gr.s(k).csrj);
        delta := srel - gr.s(j).pos;
        case gr.s(k).ck is
          when CK_HIST =>
            if gr.s(k).lt = '1' then
              res(k).rk := RK_LT;
            else
              res(k).rk := RK_ST;
            end if;
          when CK_REP =>
            res(k).rk := RK_REP;
          when CK_STRIDE =>
            res(k).rk := RK_STRIDE;
          when CK_CSR =>
            if gr.s(k).csrl = '1' then
              delta := srel - gr.s(j).pos - gr.s(j).c;
              res(k).rk := RK_LIT;
              res(k).rsl := to_unsigned(j, 2);
              res(k).src := gr.s(j).q + resize(delta, 16);
            else
              r0c(k) <= gr.s(k).act;
            end if;
          when others =>
            null;
        end case;
        if gr.s(k).act = '0' then
          res(k).rk := RK_NONE;
        end if;
        r0d(k) <= delta;
      end loop;
      r0 <= gr;
      r0res <= res;
      if reset = '1' then
        r0.valid <= '0';
      end if;
    end if;
  end process;

  -- G3b: resolve copies whose source is an earlier slot's copy part.
  g3b_proc: process (clk) is
    variable res   : qres_array(0 to QK-1);
    variable delta : unsigned(5 downto 0);
    variable j     : integer range 0 to QK-1;
  begin
    if rising_edge(clk) then
      res := r0res;
      for k in 1 to QK-1 loop
        if r0c(k) = '1' then
          j := to_integer(r0.s(k).csrj);
          delta := r0d(k);
          case res(j).rk is
            when RK_REP =>
              -- Source in a replicated piece: periodic with the same period,
              -- shifted by the position within the period.
              res(k).rk := RK_REP;
              res(k).src := res(j).src;
              res(k).per := res(j).per;
              res(k).sh := (res(j).sh + delta(4 downto 0)) and (res(j).per - 1);
            when RK_LIT =>
              res(k).rk := RK_LIT;
              res(k).rsl := res(j).rsl;
              res(k).src := res(j).src + resize(delta, 16);
            when others =>
              res(k).rk := res(j).rk;
              res(k).src := res(j).src + resize(delta, 16);
          end case;
        end if;
      end loop;
      r1 <= r0;
      r1res <= res;
      if reset = '1' then
        r1.valid <= '0';
      end if;
    end if;
  end process;

  -----------------------------------------------------------------------------
  -- S1a: per-slot scalars.
  -----------------------------------------------------------------------------
  s1a_proc: process (clk) is
    variable o     : unsigned(15 downto 0);
    variable ol    : unsigned(15 downto 0);
    variable src   : unsigned(15 downto 0);
    variable dist  : unsigned(15 downto 0);
    variable line  : unsigned(10 downto 0);
    variable rr    : unsigned(5 downto 0);
    variable v     : s1a_type;
  begin
    if rising_edge(clk) then
      v.valid := r1.valid;
      v.p0l := r1.p0(4 downto 0);
      v.n := r1.n;
      v.last := r1.last;
      v.line0 := r1.p0(15 downto 5);
      v.lnext := r1.lnext;
      v.d8 := '0';
      for k in QK-1 downto 0 loop
        if r1res(k).rk = RK_STRIDE and r1.s(k).off(3) = '1' then
          v.d8 := '1';
        end if;
      end loop;
      -- Stride piece membership by lane.
      for x in 0 to QB-1 loop
        rr := resize(to_unsigned(x, 5) - r1.p0(4 downto 0), 6);
        v.strl(x) := '0';
        for k in 0 to QK-1 loop
          if r1res(k).rk = RK_STRIDE and rr >= r1.s(k).pos and rr < r1.s(k).pos + r1.s(k).c then
            v.strl(x) := '1';
          end if;
        end loop;
      end loop;
      for k in 0 to QK-1 loop
        o := r1.p0 + resize(r1.s(k).pos, 16);
        ol := o + resize(r1.s(k).c, 16);
        src := r1res(k).src;
        v.s(k).act := r1.s(k).act;
        v.s(k).pos := r1.s(k).pos;
        v.s(k).c := r1.s(k).c;
        v.s(k).l := r1.s(k).l;
        v.s(k).rk := r1res(k).rk;
        v.s(k).rsl := r1res(k).rsl;
        v.s(k).rot := src(4 downto 0) - o(4 downto 0);
        v.s(k).srcl := src(4 downto 0);
        v.s(k).sline := src(9 downto 5);
        line := src(15 downto 5);
        v.s(k).pid1 := pid_of(line, r1);
        v.s(k).pid2 := pid_of(line + 1, r1);
        dist := r1.p0 - src;
        if dist > 64 then
          v.s(k).dist := (others => '1');
        else
          v.s(k).dist := dist(6 downto 0);
        end if;
        v.s(k).off := r1res(k).per;
        v.s(k).sh := r1res(k).sh;
        v.s(k).qlane := r1.s(k).q(4 downto 0);
        v.s(k).qline := r1.s(k).q(9 downto 5);
        v.s(k).lrot := r1.s(k).q(4 downto 0) - ol(4 downto 0);
      end loop;
      r2 <= v;
      r2_valid <= r1.valid;
      if reset = '1' then
        r2_valid <= '0';
      end if;
    end if;
  end process;

  -----------------------------------------------------------------------------
  -- S1b: per-lane control.
  -----------------------------------------------------------------------------
  s1b_proc: process (clk) is
    variable v     : s1b_type;
    variable rr    : unsigned(5 downto 0);
    variable cend  : unsigned(5 downto 0);
    variable lend  : unsigned(5 downto 0);
    variable rel   : unsigned(5 downto 0);
    variable found : boolean;
    variable cand  : boolean;
  begin
    if rising_edge(clk) then
      v.valid := r2.valid;
      v.p0l := r2.p0l;
      v.n := r2.n;
      v.last := r2.last;
      v.line0 := r2.line0;
      v.lnext := r2.lnext;

      -- Per-slot source addressing (indexed by source lane).
      for k in 0 to QK-1 loop
        v.rk(k) := r2.s(k).rk;
        v.rsl(k) := r2.s(k).rsl;
        v.rot(k) := r2.s(k).rot;
        v.lrot(k) := r2.s(k).lrot;
        for ly in 0 to QB-1 loop
          if ly < r2.s(k).srcl then
            v.st_a(k)(ly) := r2.s(k).sline + 1;
            v.lt_p(k)(ly) := r2.s(k).pid2;
          else
            v.st_a(k)(ly) := r2.s(k).sline;
            v.lt_p(k)(ly) := r2.s(k).pid1;
          end if;
          if ly < r2.s(k).qlane then
            v.li_a(k)(ly) := r2.s(k).qline + 1;
          else
            v.li_a(k)(ly) := r2.s(k).qline;
          end if;
        end loop;
      end loop;

      -- Per output lane.
      for x in 0 to QB-1 loop
        rr := resize(to_unsigned(x, 5) - r2.p0l, 6);
        if rr < r2.n then
          v.ing(x) := '1';
        else
          v.ing(x) := '0';
        end if;
        v.own(x) := "000";
        v.rec(x) := '0';
        v.idx(x) := (others => '0');
        v.str(x) := '0';
        v.slw(x) := '0';
        v.sone(x) := (others => '0');
        for k in 0 to QK-1 loop
          cend := r2.s(k).pos + r2.s(k).c;
          lend := cend + r2.s(k).l;
          if r2.s(k).act = '1' and rr >= r2.s(k).pos and rr < cend then
            v.own(x) := to_unsigned(k, 2) & '0';
            rel := rr - r2.s(k).pos;
            v.idx(x) := to_unsigned(x, 5) + r2.s(k).rot;
            case r2.s(k).rk is
              when RK_ST =>
                if resize(r2.s(k).dist, 8) <= resize(rel, 8) + 32 then
                  v.rec(x) := '1';
                end if;
              when RK_REP =>
                v.rec(x) := '1';
                v.idx(x) := r2.p0l - r2.s(k).off + ((rel(4 downto 0) + r2.s(k).sh) and (r2.s(k).off - 1));
              when RK_STRIDE =>
                v.str(x) := '1';
              when others =>
                null;
            end case;
          elsif r2.s(k).act = '1' and rr >= cend and rr < lend then
            v.own(x) := to_unsigned(k, 2) & '1';
          end if;
        end loop;

        -- Stride source: first predecessor at distance j*d (d = 4 or 8,
        -- i.e. 4*i) that is not a stride byte of this group; before the
        -- group -> recent register.
        found := false;
        for i in 1 to 8 loop
          cand := r2.d8 = '0' or i mod 2 = 0;
          if cand and not found then
            if rr < 4 * i then
              v.slw(x) := '1';
              v.sone(x)(i) := '1';
              found := true;
            elsif r2.strl((x - 4 * i) mod QB) = '0' then
              v.sone(x)(i) := '1';
              found := true;
            end if;
          end if;
        end loop;
      end loop;

      r3 <= v;
      r3_valid <= r2_valid;
      if reset = '1' then
        r3_valid <= '0';
      end if;
    end if;
  end process;

  -----------------------------------------------------------------------------
  -- Per-lane memories: short-term history and literal buffer.
  -----------------------------------------------------------------------------
  lane_gen: for ly in 0 to QB-1 generate
    signal stm : byte_array(0 to 31) := (others => X"00");
    signal lbm : byte_array(0 to 31) := (others => X"00");
  begin
    wr_proc: process (clk) is
    begin
      if rising_edge(clk) then
        if st_we(ly) = '1' then
          stm(to_integer(st_wa(ly))) <= fin(ly);
        end if;
        if lb_we = '1' then
          lbm(to_integer(lb_line)) <= lb_data(ly);
        end if;
      end if;
    end process;
    rd_gen: for k in 0 to QK-1 generate
    begin
      st_rd(k)(ly) <= stm(to_integer(r3.st_a(k)(ly)));
      lb_rd(k)(ly) <= lbm(to_integer(r3.li_a(k)(ly)));
    end generate;
  end generate;

  -----------------------------------------------------------------------------
  -- S2a: source select, rotators, merge.
  -----------------------------------------------------------------------------
  s2a_proc: process (clk) is
    variable sv    : byte_array(0 to QB-1);
    variable rc    : byte_matrix(0 to QK-1);
    variable rl    : byte_matrix(0 to QK-1);
    variable v     : s2a_type;
  begin
    if rising_edge(clk) then
      for k in 0 to QK-1 loop
        for ly in 0 to QB-1 loop
          case r3.rk(k) is
            when RK_LT =>
              sv(ly) := lt_rdat(to_integer(r3.lt_p(k)(ly)))(ly);
            when RK_LIT =>
              sv(ly) := lb_rd(to_integer(r3.rsl(k)))(ly);
            when others =>
              sv(ly) := st_rd(k)(ly);
          end case;
        end loop;
        rc(k) := rotl(sv, r3.rot(k));
        rl(k) := rotl(lb_rd(k), r3.lrot(k));
      end loop;
      for x in 0 to QB-1 loop
        if r3.own(x)(0) = '0' then
          v.m(x) := rc(to_integer(r3.own(x)(2 downto 1)))(x);
        else
          v.m(x) := rl(to_integer(r3.own(x)(2 downto 1)))(x);
        end if;
      end loop;
      v.p0l := r3.p0l;
      v.n := r3.n;
      v.last := r3.last;
      v.line0 := r3.line0;
      v.ing := r3.ing;
      v.rec := r3.rec;
      v.idx := r3.idx;
      v.str := r3.str;
      v.slw := r3.slw;
      v.sone := r3.sone;
      r4 <= v;
      r4_valid <= r3_valid;
      if r3_valid = '1' then
        cons_pos <= r3.lnext;
      end if;
      if reset = '1' then
        r4_valid <= '0';
        cons_pos <= (others => '0');
      end if;
    end if;
  end process;

  -----------------------------------------------------------------------------
  -- S2b: recent/stride fill, history write, output.
  -----------------------------------------------------------------------------
  s2b_comb_proc: process (r4, r4_valid, lw) is
    variable base : byte_array(0 to QB-1);
    variable sl   : std_logic_vector(7 downto 0);
    variable sb   : std_logic_vector(7 downto 0);
  begin
    for x in 0 to QB-1 loop
      if r4.rec(x) = '1' then
        base(x) := lw(to_integer(r4.idx(x)));
      else
        base(x) := r4.m(x);
      end if;
    end loop;
    for x in 0 to QB-1 loop
      sl := X"00";
      sb := X"00";
      for i in 1 to 8 loop
        if r4.sone(x)(i) = '1' then
          sl := sl or lw((x - 4 * i) mod QB);
          sb := sb or base((x - 4 * i) mod QB);
        end if;
      end loop;
      if r4.str(x) = '1' then
        if r4.slw(x) = '1' then
          fin(x) <= sl;
        else
          fin(x) <= sb;
        end if;
      else
        fin(x) <= base(x);
      end if;
      st_we(x) <= r4_valid and r4.ing(x);
      if to_unsigned(x, 5) < r4.p0l then
        st_wa(x) <= r4.line0(4 downto 0) + 1;
      else
        st_wa(x) <= r4.line0(4 downto 0);
      end if;
    end loop;
  end process;

  s2b_reg_proc: process (clk) is
    variable pe    : unsigned(6 downto 0);
    variable full  : std_logic;
    variable oline : byte_array(0 to QB-1);
    variable part  : byte_array(0 to QB-1);
    variable nlw   : byte_array(0 to QB-1);
  begin
    if rising_edge(clk) then
      pe := resize(r4.p0l, 7) + resize(r4.n, 7);
      if pe >= 32 then full := '1'; else full := '0'; end if;
      for x in 0 to QB-1 loop
        if to_unsigned(x, 5) >= r4.p0l then
          oline(x) := fin(x);
        else
          oline(x) := lw(x);
        end if;
        if r4.ing(x) = '1' then
          part(x) := fin(x);
          nlw(x) := fin(x);
        else
          part(x) := lw(x);
          nlw(x) := lw(x);
        end if;
      end loop;
      if r4_valid = '1' then
        lw <= nlw;
      end if;

      ln_valid <= r4_valid and full;
      ln_data <= oline;
      ln_line <= r4.line0;

      out_valid <= '0';
      out_last <= '0';
      out_cnt <= to_unsigned(QB, QI+1);
      out_data <= oline;
      pend <= '0';
      if pend = '1' then
        out_valid <= '1';
        out_last <= '1';
        out_cnt <= pend_cnt;
        out_data <= lw;
      elsif r4_valid = '1' then
        if r4.last = '1' then
          out_valid <= '1';
          if full = '1' then
            if pe = 32 then
              out_last <= '1';
            else
              pend <= '1';
              pend_cnt <= resize(pe - 32, QI+1);
            end if;
          else
            out_last <= '1';
            out_cnt <= resize(pe, QI+1);
            out_data <= part;
          end if;
        elsif full = '1' then
          out_valid <= '1';
        end if;
      end if;

      if reset = '1' then
        out_valid <= '0';
        ln_valid <= '0';
        pend <= '0';
      end if;
    end if;
  end process;

end behavior;
