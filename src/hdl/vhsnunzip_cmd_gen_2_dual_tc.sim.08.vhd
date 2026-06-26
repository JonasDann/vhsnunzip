library std;
use std.textio.all;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

library work;
use work.vhsnunzip_utils_pkg.all;
use work.vhsnunzip_int_pkg.all;

-- Standalone test for the dual-issue command generator stage 2. It drives the
-- single-issue partial-command vectors (c1.tv) bundled two per dual transfer,
-- and checks that the flattened output (cm0, then cm1 where present) matches the
-- single-issue command vectors (cm.tv).
entity vhsnunzip_cmd_gen_2_dual_tc is
end vhsnunzip_cmd_gen_2_dual_tc;

architecture testcase of vhsnunzip_cmd_gen_2_dual_tc is

  signal clk        : std_logic := '0';
  signal reset      : std_logic := '1';
  signal done       : boolean := false;

  signal c1         : dual_partial_command_stream := DUAL_PARTIAL_COMMAND_STREAM_INIT;
  signal c1_ready   : std_logic := '0';

  signal cm         : dual_command_stream := DUAL_COMMAND_STREAM_INIT;
  signal cm_ready   : std_logic := '0';

begin

  uut: vhsnunzip_cmd_gen_2_dual
    port map (
      clk           => clk,
      reset         => reset,
      c1            => c1,
      c1_ready      => c1_ready,
      lt_off_ld     => '1',
      lt_off        => (others => '0'),
      cm            => cm,
      cm_ready      => cm_ready
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
    variable c1_v : dual_partial_command_stream;
  begin
    file_open(fil, "c1.tv", read_mode);
    c1.c1_0.valid <= '0';
    c1.c1_1.valid <= '0';

    wait until reset = '0';
    wait until rising_edge(clk);

    while not endfile(fil) loop

      loop
        uniform(s1, s2, rnd);
        exit when rnd < 0.5;
        wait until rising_edge(clk);
      end loop;

      readline(fil, lin);
      stream_des(lin, c1_v.c1_0, true);
      c1_v.c1_1 := PARTIAL_COMMAND_STREAM_INIT;
      if not endfile(fil) then
        readline(fil, lin);
        stream_des(lin, c1_v.c1_1, true);
      end if;

      c1 <= c1_v;
      loop
        wait until rising_edge(clk);
        exit when c1_ready = '1';
      end loop;
      c1.c1_0.valid <= '0';
      c1.c1_1.valid <= '0';

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
    variable cm_v : command_stream;

    variable n_cmd  : natural := 0;
    variable n_pair : natural := 0;

    procedure cmp(exp : command_stream; act : command_stream) is
    begin
      assert std_match(exp.lt_val, act.lt_val) severity failure;
      assert std_match(exp.lt_adev, act.lt_adev) severity failure;
      assert std_match(exp.lt_adod, act.lt_adod) severity failure;
      assert std_match(exp.lt_swap, act.lt_swap) severity failure;
      assert std_match(exp.st_addr, act.st_addr) severity failure;
      assert std_match(exp.cp_rol, act.cp_rol) severity failure;
      assert std_match(exp.cp_rle, act.cp_rle) severity failure;
      assert std_match(exp.cp_end, act.cp_end) severity failure;
      assert std_match(exp.li_rol, act.li_rol) severity failure;
      assert std_match(exp.li_end, act.li_end) severity failure;
      assert std_match(exp.ld_pop, act.ld_pop) severity failure;
      assert std_match(exp.last, act.last) severity failure;
    end procedure;

  begin
    done <= false;

    file_open(fil, "cm.tv", read_mode);
    cm_ready <= '0';

    wait until reset = '0';
    wait until rising_edge(clk);

    while not endfile(fil) loop

      loop
        uniform(s1, s2, rnd);
        exit when rnd < 0.3;
        wait until rising_edge(clk);
      end loop;

      cm_ready <= '1';
      loop
        wait until rising_edge(clk);
        exit when cm.cm0.valid = '1';
      end loop;
      cm_ready <= '0';

      readline(fil, lin);
      stream_des(lin, cm_v, false);
      cmp(cm_v, cm.cm0);
      n_cmd := n_cmd + 1;

      if cm.cm1.valid = '1' then
        assert not endfile(fil)
          report "dual cmd_gen_2 produced a second command but cm.tv is exhausted"
          severity failure;
        readline(fil, lin);
        stream_des(lin, cm_v, false);
        cmp(cm_v, cm.cm1);
        n_cmd := n_cmd + 1;
        n_pair := n_pair + 1;
      end if;

    end loop;
    file_close(fil);

    report "dual cmd_gen_2: " & integer'image(n_cmd) & " commands, "
         & integer'image(n_pair) & " co-issued ("
         & integer'image((200 * n_pair) / n_cmd) & "% dual-issued)"
      severity note;
    -- Pairing is data dependent: incompressible inputs decode to one giant
    -- literal that stage 2 splits over many single-command cycles (and which is
    -- input-bandwidth limited anyway), so zero pairing is legitimate. The hard
    -- check is the per-command data comparison above; this is informational.
    assert n_pair > 0
      report "note: no dual-issue pairing occurred (literal-bound input)"
      severity note;

    cm_ready <= '1';
    for i in 0 to 100 loop
      wait until rising_edge(clk);
      exit when cm.cm0.valid = '1';
    end loop;
    cm_ready <= '0';

    assert cm.cm0.valid = '0' report "spurious data!" severity failure;

    done <= true;
    wait;
  end process;

end testcase;
