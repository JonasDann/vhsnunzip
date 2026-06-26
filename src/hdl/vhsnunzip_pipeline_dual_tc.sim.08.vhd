library std;
use std.textio.all;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

library work;
use work.vhsnunzip_utils_pkg.all;
use work.vhsnunzip_int_pkg.all;

-- Integration test for the speculative dual-issue pipeline. Drives the
-- compressed line vectors (cs.tv) through vhsnunzip_pipeline_dual, mocks the
-- long-term memory exactly as the single-issue pipeline_tc does (a 6-stage read
-- pipeline + write-on-output), and checks the decompressed output against the
-- single-issue golden vectors (de.tv). The fold produces byte-identical
-- decompressed data to single-issue (only the cycle timing differs), so this is
-- the end-to-end correctness proof for the doubled datapath.
entity vhsnunzip_pipeline_dual_tc is
end vhsnunzip_pipeline_dual_tc;

architecture testcase of vhsnunzip_pipeline_dual_tc is

  signal clk        : std_logic := '0';
  signal reset      : std_logic := '1';
  signal done       : boolean := false;

  signal co         : compressed_stream_single := COMPRESSED_STREAM_SINGLE_INIT;
  signal co_ready   : std_logic := '0';

  signal de         : decompressed_stream := DECOMPRESSED_STREAM_INIT;
  signal de_ready   : std_logic := '0';
  signal de_exp     : decompressed_stream := DECOMPRESSED_STREAM_INIT;

  signal lt_valid   : std_logic;
  signal lt_ready   : std_logic;
  signal lt_adev    : unsigned(C_AW-1 downto 0);
  signal lt_adod    : unsigned(C_AW-1 downto 0);
  signal lt_next    : std_logic;
  signal lt_even    : byte_array(0 to C_BYTES-1);
  signal lt_odd     : byte_array(0 to C_BYTES-1);

  -- Second long-term read port (cm1), backed by the same mock memory. Shares
  -- the arbiter ready so the two reads stay in lockstep, mirroring the real
  -- mirror-URAM port pair.
  signal lt_valid1  : std_logic;
  signal lt_adev1   : unsigned(C_AW-1 downto 0);
  signal lt_adod1   : unsigned(C_AW-1 downto 0);
  signal lt_next1   : std_logic;
  signal lt_even1   : byte_array(0 to C_BYTES-1);
  signal lt_odd1    : byte_array(0 to C_BYTES-1);

  type lt_stage is record
    valid           : std_logic;
    even            : byte_array(0 to C_BYTES-1);
    odd             : byte_array(0 to C_BYTES-1);
  end record;
  type lt_pipeline is array (natural range <>) of lt_stage;
  signal lt_stages  : lt_pipeline(0 to 5);
  signal lt_stages1 : lt_pipeline(0 to 5);

  type lt_mem_array is array (natural range <>) of byte_array(0 to C_BYTES-1);
  signal lt_mem_ev  : lt_mem_array(0 to 2**C_AW-1);
  signal lt_mem_od  : lt_mem_array(0 to 2**C_AW-1);
  signal lt_ptr     : unsigned(C_AW downto 0) := (others => '0');

begin

  uut: vhsnunzip_pipeline_dual
    generic map (
      SPEC_OFFSETS  => C_BYTES - 2
    )
    port map (
      clk           => clk,
      reset         => reset,
      co            => co,
      co_ready      => co_ready,
      lt_rd_valid   => lt_valid,
      lt_rd_ready   => lt_ready,
      lt_rd_adev    => lt_adev,
      lt_rd_adod    => lt_adod,
      lt_rd_next    => lt_next,
      lt_rd_even    => lt_even,
      lt_rd_odd     => lt_odd,
      lt_rd_valid1  => lt_valid1,
      lt_rd_ready1  => lt_ready,
      lt_rd_adev1   => lt_adev1,
      lt_rd_adod1   => lt_adod1,
      lt_rd_next1   => lt_next1,
      lt_rd_even1   => lt_even1,
      lt_rd_odd1    => lt_odd1,
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
    variable co_v : compressed_stream_single;
  begin
    file_open(fil, "cs.tv", read_mode);
    co.valid <= '0';

    wait until reset = '0';
    wait until rising_edge(clk);

    while not endfile(fil) loop

      loop
        uniform(s1, s2, rnd);
        exit when rnd < 0.5;
        wait until rising_edge(clk);
      end loop;

      readline(fil, lin);
      stream_des(lin, co_v, true);

      co <= co_v;
      loop
        wait until rising_edge(clk);
        exit when co_ready = '1';
      end loop;
      co.valid <= '0';

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
  begin
    done <= false;

    file_open(fil, "de.tv", read_mode);
    de_ready <= '0';

    wait until reset = '0';
    wait until rising_edge(clk);

    while not endfile(fil) loop

      loop
        uniform(s1, s2, rnd);
        exit when rnd < 0.3;
        wait until rising_edge(clk);
      end loop;

      readline(fil, lin);
      stream_des(lin, de_v, false);
      de_exp <= de_v;

      de_ready <= '1';
      loop
        wait until rising_edge(clk);
        exit when de.valid = '1';
      end loop;
      de_ready <= '0';

      for i in 0 to C_BYTES-1 loop
        assert std_match(de_v.data(i), de.data(i)) severity failure;
      end loop;
      assert std_match(de_v.last, de.last) severity failure;
      assert std_match(de_v.cnt, de.cnt) severity failure;

    end loop;
    file_close(fil);

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

  lt_arb_mock_proc: process is
    variable s1   : positive := 3;
    variable s2   : positive := 3;
    variable rnd  : real;
  begin

    lt_ready <= '0';

    loop
      uniform(s1, s2, rnd);
      exit when rnd < 0.3;
      wait until rising_edge(clk);
    end loop;

    lt_ready <= '1';

    loop
      uniform(s1, s2, rnd);
      exit when rnd < 0.3;
      wait until rising_edge(clk);
    end loop;

  end process;

  lt_mem_mock_proc: process (clk) is
  begin
    if rising_edge(clk) then

      -- Handle decompressed data writes.
      if de.valid = '1' and de_ready = '1' then
        if de.last = '1' then
          lt_ptr <= (others => '0');
        else
          if lt_ptr(0) = '0' then
            lt_mem_ev(to_integer(lt_ptr(C_AW downto 1))) <= de.data;
          else
            lt_mem_od(to_integer(lt_ptr(C_AW downto 1))) <= de.data;
          end if;
          lt_ptr <= lt_ptr + 1;
        end if;
      end if;

      -- Model a read pipeline that meets the requirements (cm0 port).
      lt_stages(1 to 5) <= lt_stages(0 to 4);
      lt_stages(0).valid <= lt_valid and lt_ready;
      lt_stages(0).even <= lt_mem_ev(to_integer(lt_adev));
      lt_stages(0).odd <= lt_mem_od(to_integer(lt_adod));

      lt_next <= lt_stages(4).valid;
      lt_even <= lt_stages(5).even;
      lt_odd  <= lt_stages(5).odd;

      -- Identical read pipeline for the mirror (cm1) port, reading the same
      -- memory at cm1's address and sharing the arbiter ready.
      lt_stages1(1 to 5) <= lt_stages1(0 to 4);
      lt_stages1(0).valid <= lt_valid1 and lt_ready;
      lt_stages1(0).even <= lt_mem_ev(to_integer(lt_adev1));
      lt_stages1(0).odd <= lt_mem_od(to_integer(lt_adod1));

      lt_next1 <= lt_stages1(4).valid;
      lt_even1 <= lt_stages1(5).even;
      lt_odd1  <= lt_stages1(5).odd;

      if reset = '1' then
        lt_ptr <= (others => '0');
      end if;
    end if;
  end process;

end testcase;
