library std;
use std.textio.all;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

library work;
use work.vhsnunzip_utils_pkg.all;
use work.vhsnunzip_int_pkg.all;

-- Standalone test for the speculative dual-issue decoder. It drives the same
-- cd.tv as the single-issue decoder test, and checks that the *flattened*
-- output (el0, then el1 where present) matches the single-issue el.tv element
-- for element -- i.e. the dual decoder emits exactly the same element sequence,
-- just up to two per cycle.
entity vhsnunzip_decoder_dual_tc is
end vhsnunzip_decoder_dual_tc;

architecture testcase of vhsnunzip_decoder_dual_tc is

  signal clk        : std_logic := '0';
  signal reset      : std_logic := '1';
  signal done       : boolean := false;

  signal cd         : compressed_stream_double := COMPRESSED_STREAM_DOUBLE_INIT;
  signal cd_ready   : std_logic := '0';

  signal el         : dual_element_stream := DUAL_ELEMENT_STREAM_INIT;
  signal el_ready   : std_logic := '0';

begin

  uut: vhsnunzip_decoder_dual
    generic map (
      SPEC_OFFSETS  => C_BYTES - 2
    )
    port map (
      clk           => clk,
      reset         => reset,
      cd            => cd,
      cd_ready      => cd_ready,
      el            => el,
      el_ready      => el_ready
    );

  clk_proc: process is
  begin
    wait for 500 ps;
    clk <= '0';
    wait for 500 ps;
    clk <= '1';
    if done then
      wait;
    end if;
  end process;

  reset_proc: process is
  begin
    reset <= '1';
    wait until rising_edge(clk);
    wait until rising_edge(clk);
    wait until rising_edge(clk);
    reset <= '0';
    wait;
  end process;

  source_proc: process is
    file     fil  : text;
    variable lin  : line;
    variable s1   : positive := 1;
    variable s2   : positive := 1;
    variable rnd  : real;
    variable cd_v : compressed_stream_double;
  begin
    file_open(fil, "cd.tv", read_mode);
    cd.valid <= '0';

    wait until reset = '0';
    wait until rising_edge(clk);

    while not endfile(fil) loop

      loop
        uniform(s1, s2, rnd);
        exit when rnd < 0.5;
        wait until rising_edge(clk);
      end loop;

      readline(fil, lin);
      stream_des(lin, cd_v, true);

      cd <= cd_v;
      loop
        wait until rising_edge(clk);
        exit when cd_ready = '1';
      end loop;
      cd.valid <= '0';

    end loop;
    file_close(fil);
    wait;
  end process;

  sink_proc: process is
    file     fil  : text;
    variable lin  : line;
    variable s1   : positive := 2;
    variable s2   : positive := 2;
    variable rnd  : real;
    variable el_v : element_stream;

    -- Counters: total elements and number emitted as a paired second slot.
    variable n_elem : natural := 0;
    variable n_pair : natural := 0;

    -- Compare an expected element (from el.tv) against an actual decoded one.
    procedure cmp(exp : element_stream; act : element_stream) is
    begin
      assert std_match(exp.cp_val, act.cp_val) severity failure;
      assert std_match(exp.cp_off, act.cp_off) severity failure;
      assert std_match(exp.cp_len, act.cp_len) severity failure;
      assert std_match(exp.li_val, act.li_val) severity failure;
      assert std_match(exp.li_off, act.li_off) severity failure;
      assert std_match(exp.li_len, act.li_len) severity failure;
      assert std_match(exp.ld_pop, act.ld_pop) severity failure;
      assert std_match(exp.last, act.last) severity failure;
    end procedure;

  begin
    done <= false;

    file_open(fil, "el.tv", read_mode);
    el_ready <= '0';

    wait until reset = '0';
    wait until rising_edge(clk);

    while not endfile(fil) loop

      loop
        uniform(s1, s2, rnd);
        exit when rnd < 0.3;
        wait until rising_edge(clk);
      end loop;

      -- Accept one dual transfer.
      el_ready <= '1';
      loop
        wait until rising_edge(clk);
        exit when el.el0.valid = '1';
      end loop;
      el_ready <= '0';

      -- Slot 0 is always present; compare to the next expected element.
      readline(fil, lin);
      stream_des(lin, el_v, false);
      cmp(el_v, el.el0);
      n_elem := n_elem + 1;

      -- Slot 1 is present when a second element was decoded this cycle.
      if el.el1.valid = '1' then
        assert not endfile(fil)
          report "dual decoder produced a second element but el.tv is exhausted"
          severity failure;
        readline(fil, lin);
        stream_des(lin, el_v, false);
        cmp(el_v, el.el1);
        n_elem := n_elem + 1;
        n_pair := n_pair + 1;
      end if;

    end loop;
    file_close(fil);

    report "dual decoder: " & integer'image(n_elem) & " elements, "
         & integer'image(n_pair) & " emitted as a paired second slot ("
         & integer'image((200 * n_pair) / n_elem) & "% dual-issued)"
      severity note;
    -- Pairing is data dependent: incompressible inputs decode to one giant
    -- literal with no copy elements to pair, so zero pairing is legitimate. The
    -- hard check is the per-element data comparison above; this is informational.
    assert n_pair > 0
      report "note: no dual-issue pairing occurred (literal-bound input)"
      severity note;

    -- No spurious extra elements.
    el_ready <= '1';
    for i in 0 to 100 loop
      wait until rising_edge(clk);
      exit when el.el0.valid = '1';
    end loop;
    el_ready <= '0';

    assert el.el0.valid = '0' report "spurious data!" severity failure;

    done <= true;
    wait;
  end process;

end testcase;
