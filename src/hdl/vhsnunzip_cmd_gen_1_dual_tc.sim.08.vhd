library std;
use std.textio.all;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

library work;
use work.vhsnunzip_int_pkg.all;

-- Standalone test for the dual-issue command generator stage 1. It drives the
-- single-issue element vectors (el.tv) bundled two per dual transfer, and checks
-- that the flattened output (c1_0, then c1_1 where present) matches the
-- single-issue partial-command vectors (c1.tv).
entity vhsnunzip_cmd_gen_1_dual_tc is
end vhsnunzip_cmd_gen_1_dual_tc;

architecture testcase of vhsnunzip_cmd_gen_1_dual_tc is

  signal clk        : std_logic := '0';
  signal reset      : std_logic := '1';
  signal done       : boolean := false;

  signal el         : dual_element_stream := DUAL_ELEMENT_STREAM_INIT;
  signal el_ready   : std_logic := '0';

  signal c1         : dual_partial_command_stream := DUAL_PARTIAL_COMMAND_STREAM_INIT;
  signal c1_ready   : std_logic := '0';

begin

  uut: vhsnunzip_cmd_gen_1_dual
    port map (
      clk           => clk,
      reset         => reset,
      el            => el,
      el_ready      => el_ready,
      c1            => c1,
      c1_ready      => c1_ready
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
    variable el_v : dual_element_stream;
  begin
    file_open(fil, "el.tv", read_mode);
    el.el0.valid <= '0';
    el.el1.valid <= '0';

    wait until reset = '0';
    wait until rising_edge(clk);

    while not endfile(fil) loop

      loop
        uniform(s1, s2, rnd);
        exit when rnd < 0.5;
        wait until rising_edge(clk);
      end loop;

      -- Read up to two consecutive elements into one dual transfer.
      readline(fil, lin);
      stream_des(lin, el_v.el0, true);
      el_v.el1 := ELEMENT_STREAM_INIT;
      if not endfile(fil) then
        readline(fil, lin);
        stream_des(lin, el_v.el1, true);
      end if;

      el <= el_v;
      loop
        wait until rising_edge(clk);
        exit when el_ready = '1';
      end loop;
      el.el0.valid <= '0';
      el.el1.valid <= '0';

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
    variable c1_v : partial_command_stream;

    variable n_cmd  : natural := 0;
    variable n_pair : natural := 0;

    procedure cmp(exp : partial_command_stream; act : partial_command_stream) is
    begin
      assert std_match(exp.cp_off, act.cp_off) severity failure;
      assert std_match(exp.cp_len, act.cp_len) severity failure;
      assert std_match(exp.cp_rle, act.cp_rle) severity failure;
      assert std_match(exp.li_val, act.li_val) severity failure;
      assert std_match(exp.li_off, act.li_off) severity failure;
      assert std_match(exp.li_len, act.li_len) severity failure;
      assert std_match(exp.ld_pop, act.ld_pop) severity failure;
      assert std_match(exp.last, act.last) severity failure;
    end procedure;

  begin
    done <= false;

    file_open(fil, "c1.tv", read_mode);
    c1_ready <= '0';

    wait until reset = '0';
    wait until rising_edge(clk);

    while not endfile(fil) loop

      loop
        uniform(s1, s2, rnd);
        exit when rnd < 0.3;
        wait until rising_edge(clk);
      end loop;

      c1_ready <= '1';
      loop
        wait until rising_edge(clk);
        exit when c1.c1_0.valid = '1';
      end loop;
      c1_ready <= '0';

      readline(fil, lin);
      stream_des(lin, c1_v, false);
      cmp(c1_v, c1.c1_0);
      n_cmd := n_cmd + 1;

      if c1.c1_1.valid = '1' then
        assert not endfile(fil)
          report "dual cmd_gen_1 produced a second command but c1.tv is exhausted"
          severity failure;
        readline(fil, lin);
        stream_des(lin, c1_v, false);
        cmp(c1_v, c1.c1_1);
        n_cmd := n_cmd + 1;
        n_pair := n_pair + 1;
      end if;

    end loop;
    file_close(fil);

    report "dual cmd_gen_1: " & integer'image(n_cmd) & " commands, "
         & integer'image(n_pair) & " co-issued ("
         & integer'image((200 * n_pair) / n_cmd) & "% dual-issued)"
      severity note;
    assert n_pair > 0 report "no dual-issue pairing occurred" severity failure;

    c1_ready <= '1';
    for i in 0 to 100 loop
      wait until rising_edge(clk);
      exit when c1.c1_0.valid = '1';
    end loop;
    c1_ready <= '0';

    assert c1.c1_0.valid = '0' report "spurious data!" severity failure;

    done <= true;
    wait;
  end process;

end testcase;
