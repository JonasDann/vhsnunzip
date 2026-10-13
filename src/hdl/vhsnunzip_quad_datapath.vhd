library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.vhsnunzip_int_pkg.all;
use work.vhsnunzip_quad_pkg.all;

-- S2a of the quad datapath: copy source select per slot, rotators and merge
-- (registered output). Kept as its own hierarchy level so synthesis maps
-- the merge onto wide multiplexers.
entity vhsnunzip_quad_merge is
  port (
    clk   : in  std_logic;
    sc    : in  u3_matrix(0 to QK-1);
    rot   : in  u4_array(0 to QK-1);
    lrot  : in  u4_array(0 to QK-1);
    own   : in  u4_array(0 to QB-1);
    st_rd : in  byte_matrix(0 to QK-1);
    lb_rd : in  byte_matrix(0 to QK-1);
    lt    : in  byte_matrix(0 to 3);
    m     : out byte_array(0 to QB-1)
  );
end vhsnunzip_quad_merge;

architecture behavior of vhsnunzip_quad_merge is

  -- Rotate left by r (0 .. 15): the first two stages of rotl. The last stage
  -- (by 16) is part of the merge.
  function rotl4(v : byte_array(0 to QB-1); r : unsigned(3 downto 0)) return byte_array is
    variable a, b : byte_array(0 to QB-1);
  begin
    for x in 0 to QB-1 loop
      a(x) := v((x + to_integer(r(1 downto 0))) mod QB);
    end loop;
    for x in 0 to QB-1 loop
      b(x) := a((x + 4 * to_integer(r(3 downto 2))) mod QB);
    end loop;
    return b;
  end function;

begin

  s2a_proc: process (clk) is
    variable sv    : byte_array(0 to QB-1);
    variable rc    : byte_matrix(0 to QK-1);
    variable rl    : byte_matrix(0 to QK-1);
    variable s     : unsigned(2 downto 0);
    variable pc    : byte_array(0 to 4*QK-1);
  begin
    if rising_edge(clk) then
      for k in 0 to QK-1 loop
        for ly in 0 to QB-1 loop
          s := sc(k)(ly);
          if k = 0 then
            case s(1 downto 0) is
              when "01" => sv(ly) := lt(0)(ly);
              when "10" => sv(ly) := lt(2)(ly);
              when others => sv(ly) := st_rd(0)(ly);
            end case;
          else
            case to_integer(s) is
              when 1 => sv(ly) := lt(0)(ly);
              when 2 => sv(ly) := lt(1)(ly);
              when 3 => sv(ly) := lt(2)(ly);
              when 4 => sv(ly) := lt(3)(ly);
              when 5 => sv(ly) := lb_rd(0)(ly);
              when 6 =>
                if k >= 2 then sv(ly) := lb_rd(1)(ly); else sv(ly) := st_rd(k)(ly); end if;
              when 7 =>
                if k >= 3 then sv(ly) := lb_rd(2)(ly); else sv(ly) := st_rd(k)(ly); end if;
              when others => sv(ly) := st_rd(k)(ly);
            end case;
          end if;
        end loop;
        rc(k) := rotl4(sv, rot(k));
        rl(k) := rotl4(lb_rd(k), lrot(k));
      end loop;
      -- Merge, including the last rotation stage (by 16).
      for x in 0 to QB-1 loop
        for k in 0 to QK-1 loop
          pc(4*k) := rc(k)(x);
          pc(4*k+1) := rc(k)((x + 16) mod QB);
          pc(4*k+2) := rl(k)(x);
          pc(4*k+3) := rl(k)((x + 16) mod QB);
        end loop;
        m(x) <= pc(to_integer(own(x)));
      end loop;
    end if;
  end process;

end behavior;

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

    -- Output stream. out_valid pushes a line; its data, byte count and last
    -- flag are handed over in the cycle before (out_p*, valid with
    -- out_pvalid: the D side of the output register).
    out_valid   : out std_logic;
    out_pvalid  : out std_logic;
    out_pdata   : out byte_array(0 to QB-1);
    out_pcnt    : out unsigned(QI downto 0);
    out_plast   : out std_logic
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


  -- R2: per-slot scalars. Piece boundaries are relative to the group start
  -- (saturated at 63): the copy part of slot k is [pos, cb), its literal part
  -- [cb, lb); the copy bytes from rb on come from the recent register.
  type s1a_slot_type is record
    rb    : unsigned(5 downto 0);
    cb    : unsigned(5 downto 0);
    lb    : unsigned(5 downto 0);
    rk    : std_logic_vector(2 downto 0);
    rsl   : unsigned(1 downto 0);
    rot   : unsigned(4 downto 0);
    lrot  : unsigned(4 downto 0);
    -- Source addressing: source lanes below the source start lane (ms) and
    -- below the literal start lane (mq) read the next line; copy source
    -- select code for the two cases.
    sline : unsigned(4 downto 0);
    slin1 : unsigned(4 downto 0);
    qline : unsigned(4 downto 0);
    qlin1 : unsigned(4 downto 0);
    ms    : std_logic_vector(0 to QB-1);
    mq    : std_logic_vector(0 to QB-1);
    sca   : unsigned(2 downto 0);
    scb   : unsigned(2 downto 0);
    -- Recent-register index of a copy byte at output lane x: x + ia, or for
    -- a replicated piece ((x + ia) and im) + ib with the group's period.
    ia    : unsigned(4 downto 0);
    repf  : std_logic;
    strf  : std_logic;
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
    -- Replicated pieces all repeat slot 0's period per: ib = p0 - per, im = per - 1.
    ib    : unsigned(4 downto 0);
    im    : unsigned(4 downto 0);
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
    -- per slot: rotation (without bit 4) of the copy and literal sources,
    -- and per source lane: memory addresses and copy source select
    -- (0: short-term, 1 .. 4: long-term port, 5 .. 7: literal of slot 0 .. 2;
    -- slot 0 only uses 0, 1 and 2 = long-term port 2).
    rot   : u4_array(0 to QK-1);
    lrot  : u4_array(0 to QK-1);
    st_a  : u5_matrix(0 to QK-1);
    li_a  : u5_matrix(0 to QK-1);
    sc    : u3_matrix(0 to QK-1);
    -- per output lane: piece select (slot, literal, rotation bit 4)
    own   : u4_array(0 to QB-1);
    ing   : std_logic_vector(0 to QB-1);
    rec   : std_logic_vector(0 to QB-1);
    idx   : u5_array(0 to QB-1);
    str   : std_logic_vector(0 to QB-1);
    sdi   : u3_array(0 to QB-1);    -- stride source distance 4 * (sdi + 1)
  end record;
  signal r3       : s1b_type;
  signal r3_valid : std_logic := '0';

  -- R4: merged data plus S2b control.
  type s2a_type is record
    p0l   : unsigned(4 downto 0);
    n     : unsigned(5 downto 0);
    last  : std_logic;
    line0 : unsigned(10 downto 0);
    ing   : std_logic_vector(0 to QB-1);
    rec   : std_logic_vector(0 to QB-1);
    idx   : u5_array(0 to QB-1);
    str   : std_logic_vector(0 to QB-1);
    sdi   : u3_array(0 to QB-1);    -- stride source distance 4 * (sdi + 1)
  end record;
  signal r4       : s2a_type;
  signal r4_valid : std_logic := '0';
  signal r4m      : byte_array(0 to QB-1);

  attribute keep_hierarchy : string;
  attribute keep_hierarchy of merge_inst : label is "yes";

  -- Pipeline registers stay flip-flops (not shift-register LUTs).
  attribute shreg_extract : string;
  attribute shreg_extract of r0, r0res, r1, r1res, r2, r3, r4 : signal is "no";

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
  signal oline    : byte_array(0 to QB-1);
  signal oline_d  : byte_array(0 to QB-1);
  signal lw_d     : byte_array(0 to QB-1);
  signal full_d   : std_logic;
  signal ov_d     : std_logic;
  signal pend_d   : std_logic;
  signal pcnt_d   : unsigned(QI downto 0);

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

  function sat6(v : unsigned) return unsigned is
  begin
    if v > 63 then
      return to_unsigned(63, 6);
    end if;
    return resize(v, 6);
  end function;


begin

  -----------------------------------------------------------------------------
  -- G3: copy-source resolution.
  -----------------------------------------------------------------------------
  -- G3a: direct sources, and for copies resolved into an earlier slot's copy
  -- part, the source slot and the offset within it.
  g3a_proc: process (clk) is
    variable res   : qres_array(0 to QK-1);
    variable srel  : unsigned(5 downto 0);
    variable delta : unsigned(5 downto 0);
    variable j     : integer range 0 to QK-1;
  begin
    if rising_edge(clk) then
      for k in 0 to QK-1 loop
        res(k).rk := RK_NONE;
        res(k).src := gr.s(k).src;
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
    variable cb    : unsigned(6 downto 0);
    variable lb    : unsigned(6 downto 0);
    variable rb    : unsigned(6 downto 0);
    variable pid1  : unsigned(1 downto 0);
    variable pid2  : unsigned(1 downto 0);
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
      v.ib := r1.p0(4 downto 0) - r1res(0).per;
      v.im := r1res(0).per - 1;
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
        cb := resize(r1.s(k).pos, 7) + resize(r1.s(k).c, 7);
        lb := cb + resize(r1.s(k).l, 7);
        dist := r1.p0 - src;
        rb := cb;
        case r1res(k).rk is
          when RK_ST =>
            -- Copy bytes whose source is at or after p0 - 32.
            if dist <= 32 then
              rb := resize(r1.s(k).pos, 7);
            elsif dist <= 64 then
              rb := resize(r1.s(k).pos, 7) + resize(dist(6 downto 0) - 32, 7);
            end if;
          when RK_REP =>
            rb := resize(r1.s(k).pos, 7);
          when others =>
            null;
        end case;
        if r1.s(k).act = '1' then
          v.s(k).rb := sat6(rb);
          v.s(k).cb := sat6(cb);
          v.s(k).lb := sat6(lb);
        else
          v.s(k).rb := to_unsigned(63, 6);
          v.s(k).cb := to_unsigned(63, 6);
          v.s(k).lb := to_unsigned(63, 6);
        end if;
        v.s(k).rk := r1res(k).rk;
        v.s(k).rsl := r1res(k).rsl;
        v.s(k).rot := src(4 downto 0) - o(4 downto 0);
        v.s(k).sline := src(9 downto 5);
        v.s(k).slin1 := src(9 downto 5) + 1;
        v.s(k).qline := r1.s(k).q(9 downto 5);
        v.s(k).qlin1 := r1.s(k).q(9 downto 5) + 1;
        for ly in 0 to QB-1 loop
          if ly < src(4 downto 0) then v.s(k).ms(ly) := '1'; else v.s(k).ms(ly) := '0'; end if;
          if ly < r1.s(k).q(4 downto 0) then v.s(k).mq(ly) := '1'; else v.s(k).mq(ly) := '0'; end if;
        end loop;
        line := src(15 downto 5);
        pid1 := pid_of(line, r1);
        pid2 := pid_of(line + 1, r1);
        v.s(k).sca := "000";
        v.s(k).scb := "000";
        if r1res(k).rk = RK_LT then
          if k = 0 then
            -- Slot 0 always gets port B (0 or 2) of a bank.
            v.s(k).sca := '0' & pid1(1) & not pid1(1);
            v.s(k).scb := '0' & pid2(1) & not pid2(1);
          else
            v.s(k).sca := resize(pid1, 3) + 1;
            v.s(k).scb := resize(pid2, 3) + 1;
          end if;
        elsif r1res(k).rk = RK_LIT and k > 0 then
          v.s(k).sca := resize(r1res(k).rsl, 3) + 5;
          v.s(k).scb := resize(r1res(k).rsl, 3) + 5;
        end if;
        v.s(k).lrot := r1.s(k).q(4 downto 0) - ol(4 downto 0);
        if r1res(k).rk = RK_REP then
          -- Replicated period per before the group: byte at
          -- p0l - per + ((x - p0l - pos + sh) mod per).
          v.s(k).ia := r1res(k).sh - r1.p0(4 downto 0) - r1.s(k).pos(4 downto 0);
          v.s(k).repf := '1';
        else
          v.s(k).ia := src(4 downto 0) - o(4 downto 0);
          v.s(k).repf := '0';
        end if;
        if r1res(k).rk = RK_STRIDE then
          v.s(k).strf := '1';
        else
          v.s(k).strf := '0';
        end if;
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
    variable tr    : std_logic_vector(0 to QK-1);
    variable tc    : std_logic_vector(0 to QK-1);
    variable tl    : std_logic_vector(0 to QK-1);
    variable s1    : std_logic;
    variable s0    : std_logic;
    variable sl    : integer range 0 to QK-1;
    variable lit   : std_logic;
    variable rc    : std_logic;
    variable h     : std_logic;
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
        v.rot(k) := r2.s(k).rot(3 downto 0);
        v.lrot(k) := r2.s(k).lrot(3 downto 0);
        for ly in 0 to QB-1 loop
          if r2.s(k).ms(ly) = '1' then
            v.st_a(k)(ly) := r2.s(k).slin1;
            v.sc(k)(ly) := r2.s(k).scb;
          else
            v.st_a(k)(ly) := r2.s(k).sline;
            v.sc(k)(ly) := r2.s(k).sca;
          end if;
          if r2.s(k).mq(ly) = '1' then
            v.li_a(k)(ly) := r2.s(k).qlin1;
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
        for k in 0 to QK-1 loop
          if rr >= r2.s(k).rb then tr(k) := '1'; else tr(k) := '0'; end if;
          if rr >= r2.s(k).cb then tc(k) := '1'; else tc(k) := '0'; end if;
          if rr >= r2.s(k).lb then tl(k) := '1'; else tl(k) := '0'; end if;
        end loop;
        -- Owner slot: number of literal ends passed (they are ordered).
        s1 := tl(1);
        s0 := (tl(0) and not tl(1)) or tl(2);
        sl := to_integer(unsigned'(s1 & s0));
        lit := '0';
        rc := '0';
        for k in 0 to QK-1 loop
          lit := lit or (tc(k) and not tl(k));
          rc := rc or (tr(k) and not tc(k));
        end loop;
        if lit = '1' then
          h := r2.s(sl).lrot(4);
        else
          h := r2.s(sl).rot(4);
        end if;
        v.own(x) := to_unsigned(sl, 2) & lit & h;
        v.str(x) := r2.s(sl).strf and not lit;
        if r2.s(sl).repf = '1' then
          v.idx(x) := ((to_unsigned(x, 5) + r2.s(sl).ia) and r2.im) + r2.ib;
        else
          v.idx(x) := to_unsigned(x, 5) + r2.s(sl).ia;
        end if;
        v.rec(x) := rc;

        -- Stride source: first predecessor at distance j*d (d = 4 or 8,
        -- i.e. 4*i) that is not a stride byte of this group; before the
        -- group -> recent register.
        v.sdi(x) := "000";
        found := false;
        for i in 1 to 8 loop
          cand := r2.d8 = '0' or i mod 2 = 0;
          if cand and not found then
            if rr < 4 * i then
              -- Source before the group: read through the recent-register
              -- index instead of the stride select.
              if v.str(x) = '1' then
                v.rec(x) := '1';
                v.idx(x) := to_unsigned((x - 4 * i) mod QB, 5);
                v.str(x) := '0';
              end if;
              found := true;
            elsif r2.strl((x - 4 * i) mod QB) = '0' then
              v.sdi(x) := to_unsigned(i - 1, 3);
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
  -- Every 2-bit slice of every read port is one LUT of a RAM32M16 (seven
  -- per primitive). The short-term memory has a write address per lane; all
  -- lanes of the literal buffer share one.
  lane_gen: for ly in 0 to QB-1 generate
    signal wd : std_logic_vector(31 downto 0);
    signal ra : std_logic_vector(79 downto 0);
    signal rd : std_logic_vector(31 downto 0);
  begin
    port_gen: for p in 0 to 15 generate
      wd(2*p+1 downto 2*p) <= fin(ly)(2*(p mod 4)+1 downto 2*(p mod 4));
      ra(5*p+4 downto 5*p) <= std_logic_vector(r3.st_a(p / 4)(ly));
      st_rd(p / 4)(ly)(2*(p mod 4)+1 downto 2*(p mod 4)) <= rd(2*p+1 downto 2*p);
    end generate;
    stm_inst: entity work.vhsnunzip_quad_lram
      generic map (N => 16)
      port map (
        clk => clk,
        we  => st_we(ly),
        wa  => std_logic_vector(st_wa(ly)),
        wd  => wd,
        ra  => ra,
        rd  => rd
      );
  end generate;

  lb_blk: block is
    signal wd : std_logic_vector(32*32-1 downto 0);
    signal ra : std_logic_vector(32*80-1 downto 0);
    signal rd : std_logic_vector(32*32-1 downto 0);
  begin
    port_gen: for p in 0 to 32*16-1 generate
      constant ly : natural := p / 16;
      constant k  : natural := (p mod 16) / 4;
      constant sl : natural := p mod 4;
    begin
      wd(2*p+1 downto 2*p) <= lb_data(ly)(2*sl+1 downto 2*sl);
      ra(5*p+4 downto 5*p) <= std_logic_vector(r3.li_a(k)(ly));
      lb_rd(k)(ly)(2*sl+1 downto 2*sl) <= rd(2*p+1 downto 2*p);
    end generate;
    lbm_inst: entity work.vhsnunzip_quad_lram
      generic map (N => 32*16)
      port map (
        clk => clk,
        we  => lb_we,
        wa  => std_logic_vector(lb_line),
        wd  => wd,
        ra  => ra,
        rd  => rd
      );
  end block;

  -----------------------------------------------------------------------------
  -- S2a: source select, rotators, merge.
  -----------------------------------------------------------------------------
  merge_inst: entity work.vhsnunzip_quad_merge
    port map (
      clk   => clk,
      sc    => r3.sc,
      rot   => r3.rot,
      lrot  => r3.lrot,
      own   => r3.own,
      st_rd => st_rd,
      lb_rd => lb_rd,
      lt    => lt_rdat,
      m     => r4m
    );

  s2a_proc: process (clk) is
  begin
    if rising_edge(clk) then
      r4.p0l <= r3.p0l;
      r4.n <= r3.n;
      r4.last <= r3.last;
      r4.line0 <= r3.line0;
      r4.ing <= r3.ing;
      r4.rec <= r3.rec;
      r4.idx <= r3.idx;
      r4.str <= r3.str;
      r4.sdi <= r3.sdi;
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
  s2b_comb_proc: process (r4, r4m, r4_valid, lw) is
    variable base : byte_array(0 to QB-1);
    variable sc8  : byte_array(0 to 7);
  begin
    for x in 0 to QB-1 loop
      if r4.rec(x) = '1' then
        base(x) := lw(to_integer(r4.idx(x)));
      else
        base(x) := r4m(x);
      end if;
    end loop;
    for x in 0 to QB-1 loop
      for i in 1 to 8 loop
        sc8(i - 1) := base((x - 4 * i) mod QB);
      end loop;
      if r4.str(x) = '1' then
        fin(x) <= sc8(to_integer(r4.sdi(x)));
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

  -- The output line goes to the output FIFO (from the D side of the output
  -- register) and the long-term memory (from the register). Bytes past the
  -- group end are don't-care (out_cnt limits them).
  -- A flush line (pend) always comes in a bubble cycle: the group former
  -- leaves one after the last group of a chunk.
  s2b_out_proc: process (r4, r4_valid, pend, pend_cnt, lw, fin, reset) is
    variable pe    : unsigned(6 downto 0);
    variable full  : std_logic;
    variable ov    : std_logic;
    variable ol    : std_logic;
    variable oc    : unsigned(QI downto 0);
    variable pd    : std_logic;
    variable pc    : unsigned(QI downto 0);
  begin
    pe := resize(r4.p0l, 7) + resize(r4.n, 7);
    if pe >= 32 then full := '1'; else full := '0'; end if;
    for x in 0 to QB-1 loop
      if pend = '1' or to_unsigned(x, 5) < r4.p0l then
        oline_d(x) <= lw(x);
      else
        oline_d(x) <= fin(x);
      end if;
      if r4_valid = '1' and r4.ing(x) = '1' then
        lw_d(x) <= fin(x);
      else
        lw_d(x) <= lw(x);
      end if;
    end loop;

    ov := '0';
    ol := '0';
    oc := to_unsigned(QB, QI+1);
    pd := '0';
    pc := pend_cnt;
    if pend = '1' then
      ov := '1';
      ol := '1';
      oc := pend_cnt;
    elsif r4_valid = '1' then
      if r4.last = '1' then
        ov := '1';
        if full = '1' then
          if pe = 32 then
            ol := '1';
          else
            pd := '1';
            pc := resize(pe - 32, QI+1);
          end if;
        else
          ol := '1';
          oc := resize(pe, QI+1);
        end if;
      elsif full = '1' then
        ov := '1';
      end if;
    end if;
    if reset = '1' then
      ov := '0';
      pd := '0';
    end if;

    full_d <= full;
    ov_d <= ov;
    pend_d <= pd;
    pcnt_d <= pc;
    out_pvalid <= ov;
    out_plast <= ol;
    out_pcnt <= oc;
  end process;

  out_pdata <= oline_d;

  s2b_reg_proc: process (clk) is
  begin
    if rising_edge(clk) then
      oline <= oline_d;
      lw <= lw_d;
      ln_valid <= r4_valid and full_d;
      ln_line <= r4.line0;
      out_valid <= ov_d;
      pend <= pend_d;
      pend_cnt <= pcnt_d;
      if reset = '1' then
        ln_valid <= '0';
      end if;
    end if;
  end process;

  ln_data <= oline;

  -- pragma translate_off
  chk_proc: process (clk) is
  begin
    if rising_edge(clk) then
      assert not (pend = '1' and r4_valid = '1') report "group during flush line" severity failure;
    end if;
  end process;
  -- pragma translate_on

end behavior;
