library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.vhsnunzip_utils_pkg.all;
use work.vhsnunzip_int_pkg.all;

-- Behavioral dual-issue execute (functional reference for the dual-issue
-- datapath). See the component declaration in vhsnunzip_int_pkg for the role.
--
-- Each cycle this retires command 0 and, when present, command 1 into the
-- decompressed output. The crux is the same-cycle RAW hazard: command 1's copy
-- may read bytes command 0 produces in the *same* cycle. Because consecutive
-- commands are contiguous in decompressed-output space, command 1's copy reads
-- are always either already-committed history or exactly command 0's just-
-- produced bytes, so they are always satisfiable. Here that falls out naturally
-- because command 0 is appended to the history buffer before command 1 reads it.
--
-- This is NOT synthesizable: it uses an absolute per-chunk history buffer rather
-- than the short-term SRL + long-term URAM split of the single-issue datapath.
-- The synthesizable microarchitecture (two rotators, duplicated short-term SRLs,
-- a second long-term read path, and a forward mux from command 0's output line)
-- is a refinement of this proven functional reference, in the same way the dual
-- decoder's chained two-pass form is the reference for its parallel form.
entity vhsnunzip_execute_dual is
  port (
    clk         : in  std_logic;
    reset       : in  std_logic;

    -- Up to two semantic commands per cycle.
    cm          : in  dual_exec_command;
    cm_ready    : out std_logic;

    -- Decompressed data output stream.
    de          : out decompressed_stream;
    de_ready    : in  std_logic
  );
end vhsnunzip_execute_dual;

architecture behavior of vhsnunzip_execute_dual is

  -- Maximum decompressed chunk size (the Snappy window is 64kiB; chunks in the
  -- regression are <= 64kiB). Copies never reach back further than this.
  constant HISTSIZE : natural := 65536;

  -- Output line queue depth. A co-issued pair can complete up to two full lines
  -- and, on the last command, one trailing boundary line, so room for three is
  -- reserved before a pair is retired.
  constant QDEPTH   : natural := 8;

  type de_queue is array (natural range <>) of decompressed_stream;

begin
  proc: process (clk) is

    -- Absolute decompressed history for the current chunk. hlen is the write
    -- position (= bytes produced so far this chunk); emitted is how many of
    -- those have been pushed out as full lines.
    variable hist     : byte_array(0 to HISTSIZE-1);
    variable hlen     : natural := 0;
    variable emitted  : natural := 0;

    -- Input holding register.
    variable cinh     : dual_exec_command := DUAL_EXEC_COMMAND_INIT;

    -- Output line queue and the registered output transfer.
    variable oq       : de_queue(0 to QDEPTH-1);
    variable oq_cnt   : natural := 0;
    variable deh      : decompressed_stream := DECOMPRESSED_STREAM_INIT;

    -- Retire one command: append its copy bytes then its literal bytes to the
    -- chunk history. Copy bytes are produced sequentially so run-length and
    -- self-overlapping copies, and the same-cycle hazard, read freshly produced
    -- bytes (command 0's bytes are already in `hist` when command 1 runs).
    procedure run(c : exec_command) is
      variable cc : natural;
      variable lc : natural;
    begin
      if c.valid = '1' then
        cc := to_integer(c.cp_count);
        lc := to_integer(c.li_count);
        for k in 0 to C_BYTES-1 loop
          if k < cc then
            hist(hlen) := hist(hlen - to_integer(c.cp_off));
            hlen := hlen + 1;
          end if;
        end loop;
        for k in 0 to C_BYTES-1 loop
          if k < lc then
            hist(hlen) := c.li(k);
            hlen := hlen + 1;
          end if;
        end loop;
      end if;
    end procedure;

    procedure qpush(d : byte_array; last : std_logic; cnt : natural) is
      variable e : decompressed_stream;
    begin
      e.valid := '1';
      e.data  := d;
      e.last  := last;
      e.cnt   := to_unsigned(cnt, C_CNT);
      oq(oq_cnt) := e;
      oq_cnt := oq_cnt + 1;
    end procedure;

    variable line   : byte_array(0 to C_BYTES-1);
    variable tail   : natural;
    variable islast : std_logic;

  begin
    if rising_edge(clk) then

      -- Output: drain the queue into the registered output transfer.
      if de_ready = '1' then
        deh.valid := '0';
      end if;
      if deh.valid = '0' and oq_cnt > 0 then
        deh := oq(0);
        for i in 0 to QDEPTH-2 loop
          oq(i) := oq(i+1);
        end loop;
        oq_cnt := oq_cnt - 1;
      end if;

      -- Latch a new command pair into the input holding register when free.
      if cinh.c0.valid = '0' then
        cinh := cm;
      end if;

      -- Retire the held pair when there is queue room.
      if cinh.c0.valid = '1' and oq_cnt <= QDEPTH-3 then

        islast := cinh.c0.last;
        run(cinh.c0);
        if cinh.c0.last = '0' and cinh.c1.valid = '1' then
          run(cinh.c1);
          islast := cinh.c1.last;
        end if;

        -- Push any completed full lines; defer the chunk's final (partial or
        -- empty) boundary line until the last command so it carries `last`.
        while hlen - emitted >= C_BYTES loop
          for i in 0 to C_BYTES-1 loop
            line(i) := hist(emitted + i);
          end loop;
          qpush(line, '0', C_BYTES);
          emitted := emitted + C_BYTES;
        end loop;

        if islast = '1' then
          tail := hlen - emitted;           -- 0 .. C_BYTES-1
          line := (others => (others => '0'));
          for i in 0 to C_BYTES-1 loop
            if i < tail then
              line(i) := hist(emitted + i);
            end if;
          end loop;
          qpush(line, '1', tail);
          hlen := 0;
          emitted := 0;
        end if;

        cinh.c0.valid := '0';
        cinh.c1.valid := '0';
      end if;

      if reset = '1' then
        cinh.c0.valid := '0';
        cinh.c1.valid := '0';
        oq_cnt  := 0;
        hlen    := 0;
        emitted := 0;
        deh.valid := '0';
      end if;

      cm_ready <= not cinh.c0.valid;
      de <= deh;

    end if;
  end process;
end behavior;
