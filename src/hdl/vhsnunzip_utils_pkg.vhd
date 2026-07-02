library std;
use std.textio.all;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_textio.all;

package vhsnunzip_utils_pkg is

  -- Returns 'U' during simulation, but '0' during synthesis.
  function undef_fn return std_logic;

  -- Ceiling of the base-2 logarithm.
  function clog2(n : positive) return natural;

  -- Datapath lane width in bytes: the number of bytes the pipeline produces
  -- per cycle, the width of a memory/short-term line, and the width of the
  -- compressed/decompressed stream lines. The original core was hardwired to
  -- 8; everything width-dependent is now derived from this constant so it can
  -- be widened (e.g. to 16, ~doubling single-issue throughput) in one place.
  -- Must be a power of two.
  --
  -- Setting this to 16 has been verified in simulation for the UNBUFFERED core
  -- (vhsnunzip_unbuffered / vhsnunzip_pipeline). When changing it, also set the
  -- matching width WI in tests/emu/streams.py so the reference model and test
  -- vectors track the RTL. NOTE: the buffered and multicore toplevels
  -- (vhsnunzip_buffered, vhsnunzip) still assume 8-byte lines and only work at
  -- C_BYTES = 8; the synthesis RAM (vhsnunzip_ram.syn.vhd) also needs widening
  -- (2 URAMs per bank) for C_BYTES > 8.
  constant C_BYTES : positive := 16;

  -- Derived widths (all equal to the original hardcoded values at C_BYTES=8):
  --   C_IDX : bits to index a byte within one line          (endi, start)
  --   C_CNT : bits for a 0..C_BYTES inclusive count          (cnt, cp/li_end)
  --   C_WIN : bits to index a byte within the two-line window (cp/li_rol, li_off)
  constant C_IDX : positive := clog2(C_BYTES);
  constant C_CNT : positive := clog2(C_BYTES) + 1;
  constant C_WIN : positive := clog2(2 * C_BYTES);

  -- Global default for the DUAL_ISSUE datapath flag exposed by vhsnunzip_unbuffered:
  --
  --   false -> the proven single-issue datapath (vhsnunzip_pipeline): one Snappy
  --            element per cycle. This is the default.
  --   true  -> the dual-issue datapath (vhsnunzip_pipeline_dual): its
  --            decoder emits a second element per cycle by decoding it in parallel
  --            and selecting the slot matching element 0's actual size, which
  --            breaks the chained element0->element1 decode that would otherwise
  --            dominate the critical path.
  constant C_DUAL_ISSUE : boolean := false;

  -- Number of speculative element-1 start offsets the dual-issue decoder
  -- evaluates. Element 0's start offset is line-local (0 .. C_BYTES-1) and its
  -- size is at most C_BYTES-1, so N = C_BYTES-2 covers every reachable element-0
  -- size (full co-issue). This is an internal detail of the dual-issue datapath
  -- (not exposed by the toplevels): the DUAL_ISSUE flag above just selects whether that
  -- datapath is built at all, and when it is, it always runs at full coverage.
  constant C_SPEC_OFFSETS : natural := C_BYTES - 2;

  -- Per-bank long-term memory line-address width. The 64kiB history window is
  -- split into an even and an odd bank of C_BYTES-byte lines, so each bank
  -- holds 32768/C_BYTES lines. The combined (virtual) line pointer is C_AW+1
  -- bits wide. (= 12 at C_BYTES=8, matching the original.)
  constant C_AW  : positive := 15 - C_IDX;

end package vhsnunzip_utils_pkg;

package body vhsnunzip_utils_pkg is

  function undef_fn return std_logic is
    variable retval : std_logic := '0';
  begin
    -- pragma translate_off
    retval := 'U';
    -- pragma translate_on
    return retval;
  end function;

  function clog2(n : positive) return natural is
    variable r : natural := 0;
    variable v : positive := 1;
  begin
    while v < n loop
      v := v * 2;
      r := r + 1;
    end loop;
    return r;
  end function;

end package body vhsnunzip_utils_pkg;
