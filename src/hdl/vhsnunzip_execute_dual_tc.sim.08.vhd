library std;
use std.textio.all;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

library work;
use work.vhsnunzip_utils_pkg.all;
use work.vhsnunzip_int_pkg.all;

-- Standalone test for the behavioral dual-issue execute. It reads the semantic
-- command vectors (cmde.tv), co-issues them two per cycle (never pairing across
-- a chunk boundary), and drives the dual execute. The decompressed output is
-- checked per chunk against the single-issue reference (de.tv) by accumulated
-- byte content -- i.e. the dual execute must reproduce exactly the same
-- decompressed bytes, independent of how either side packetizes lines.
entity vhsnunzip_execute_dual_tc is
end vhsnunzip_execute_dual_tc;

architecture testcase of vhsnunzip_execute_dual_tc is

  constant HISTSIZE : natural := 65536;

  signal clk        : std_logic := '0';
  signal reset      : std_logic := '1';
  signal done       : boolean := false;

  signal cm         : dual_exec_command := DUAL_EXEC_COMMAND_INIT;
  signal cm_ready   : std_logic := '0';

  signal de         : decompressed_stream := DECOMPRESSED_STREAM_INIT;
  signal de_ready   : std_logic := '0';

begin

  uut: vhsnunzip_execute_dual
    port map (
      clk           => clk,
      reset         => reset,
      cm            => cm,
      cm_ready      => cm_ready,
      de            => de,
      de_ready      => de_ready
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
    variable a    : exec_command;
    variable b    : exec_command;
    variable cm_v : dual_exec_command;
  begin
    file_open(fil, "cmde.tv", read_mode);
    cm.c0.valid <= '0';
    cm.c1.valid <= '0';

    wait until reset = '0';
    wait until rising_edge(clk);

    while not endfile(fil) loop

      loop
        uniform(s1, s2, rnd);
        exit when rnd < 0.5;
        wait until rising_edge(clk);
      end loop;

      -- Read command 0; co-issue command 1 unless command 0 ends the chunk.
      readline(fil, lin);
      stream_des(lin, a, true);
      cm_v.c0 := a;
      cm_v.c1 := EXEC_COMMAND_INIT;
      if a.last = '0' and not endfile(fil) then
        readline(fil, lin);
        stream_des(lin, b, true);
        cm_v.c1 := b;
      end if;

      cm <= cm_v;
      loop
        wait until rising_edge(clk);
        exit when cm_ready = '1';
      end loop;
      cm.c0.valid <= '0';
      cm.c1.valid <= '0';

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
    variable de_v : decompressed_stream;

    -- Accumulated decompressed bytes for the current chunk: actual (from the
    -- dual execute) and expected (from de.tv).
    variable act  : byte_array(0 to HISTSIZE-1);
    variable exp  : byte_array(0 to HISTSIZE-1);
    variable alen : natural := 0;
    variable elen : natural := 0;
    variable chunk_no : natural := 0;
  begin
    done <= false;

    file_open(fil, "de.tv", read_mode);
    de_ready <= '0';

    wait until reset = '0';
    wait until rising_edge(clk);

    loop

      loop
        uniform(s1, s2, rnd);
        exit when rnd < 0.3;
        wait until rising_edge(clk);
      end loop;

      -- Accept one output transfer from the dual execute.
      de_ready <= '1';
      loop
        wait until rising_edge(clk);
        exit when de.valid = '1';
      end loop;
      de_ready <= '0';

      for i in 0 to to_integer(de.cnt) - 1 loop
        act(alen) := de.data(i);
        alen := alen + 1;
      end loop;

      if de.last = '1' then

        -- Read the matching expected chunk from de.tv (concatenate lines until
        -- the single-issue last flag).
        elen := 0;
        loop
          assert not endfile(fil)
            report "de.tv exhausted before dual execute finished a chunk"
            severity failure;
          readline(fil, lin);
          stream_des(lin, de_v, true);
          for i in 0 to to_integer(de_v.cnt) - 1 loop
            exp(elen) := de_v.data(i);
            elen := elen + 1;
          end loop;
          exit when de_v.last = '1';
        end loop;

        assert alen = elen
          report "chunk " & integer'image(chunk_no) & " length mismatch: dual="
               & integer'image(alen) & " ref=" & integer'image(elen)
          severity failure;
        for i in 0 to alen - 1 loop
          assert std_match(act(i), exp(i))
            report "chunk " & integer'image(chunk_no) & " data mismatch at byte "
                 & integer'image(i)
            severity failure;
        end loop;

        report "dual execute: chunk " & integer'image(chunk_no) & " ok ("
             & integer'image(alen) & " bytes)" severity note;
        chunk_no := chunk_no + 1;
        alen := 0;

        exit when endfile(fil);

      end if;

    end loop;
    file_close(fil);

    -- No spurious extra output.
    de_ready <= '1';
    for i in 0 to 100 loop
      wait until rising_edge(clk);
      exit when de.valid = '1';
    end loop;
    de_ready <= '0';

    assert de.valid = '0' report "spurious data!" severity failure;

    done <= true;
    wait;
  end process;

end testcase;
