# Dual-issue vhsnunzip — implementation plan

## Goal and scope

Roughly double the single-stream throughput of the **unbuffered streaming core**
(`vhsnunzip_unbuffered` / `vhsnunzip_pipeline`). The buffered and multicore
toplevels rely on 64 kiB chunking and chunk-level parallelism, which a single
arbitrary-length Snappy stream (one Parquet page) does not provide, so they are
out of scope here.

Two independent throughput levers must be combined:

| Lever | From → to | Needed because |
|-------|-----------|----------------|
| **Element rate** | 1 → 2 elements/cycle | decoder emits one element/cycle; on short-match data avg element ≈ 5–6 B, so element rate is the wall |
| **Byte rate** | 8 → 16 B/cycle | two ~6-byte elements per cycle need a datapath wider than 8 bytes to retire their output |

Neither lever alone reaches 2× on the data we measured (widening alone was
~1.17× on text; the element rate is the binding constraint). They are delivered
in two stages: a **wide single-issue datapath** (Phase 1, independently useful
and de-risks the memory/rotator work), then **dual-issue with a hazard
resolution stage** layered on top (Phases 2–5).

## Hazard recap (from the Phase 0 analysis)

Dual issue pairs consecutive elements (slot 0, slot 1). The only same-cycle
hazard is **slot-1 copy reading bytes slot-0 produced this cycle**
(`off1 ≤ len0 + len1 − 1`). Cross-pair dependencies are one cycle apart and
already covered by the history write→read latency.

Rather than a combinational slot-0→slot-1 bypass (which lengthens the execute
critical path), a **registered resolution stage** rewrites slot-1's sources so
the execute stage only ever reads committed data:

- **slot-0 = short literal** → slot-1's hazardous bytes come from slot-0's
  literal bytes, carried as command payload (≤ `MAX_LITERAL` bytes) and fed to
  slot-1's per-byte literal mux. Always resolvable; partial overlap is fine
  because the per-byte literal/copy mux already exists.
- **slot-0 = non-overlapping copy** (`off0 ≥ len0`) → slot-1's hazardous bytes
  read committed history at effective offset `off1 + off0`.
  - **full overlap** (`off1 ≤ len0`, all slot-1 bytes hazardous): a single
    rewritten copy with one rotation → dual-issue fires.
  - **partial overlap** (`len0 < off1 ≤ len0+len1−1`): slot-1 needs *two*
    offsets (`off1` and `off1+off0`) across its bytes — one rotation cannot do
    this. Either split slot-1 at byte `off1−len0` (pair only the first part) or
    fall back. **This case is not separated in the current analyzer** (see
    Phase 0.5).
- **slot-0 = overlapping / RLE copy** (`off0 < len0`) → chasing the offset lands
  back inside slot-0's fresh output; **residual, single-issue fallback**.

Phase 0 (TPC-H lineitem) showed the residual is ~0.1% of hazards, so the
resolution stage recovers nearly all the throughput plain fallback forfeits
(e.g. `l_orderkey` 1.58× → 2.00× decode rate).

## Phase 0.5 — refine the metric before RTL (analyzer, ~½ day)

The `resolvable, offset` bucket currently conflates the clean single-rewrite
case with the partial-overlap case that needs a splitter. Split it:

- `offset_full` : `off1 ≤ len0` (single rewritten copy, dual-issue fires)
- `offset_partial` : `len0 < off1 ≤ len0+len1−1` (needs slot-1 split or fallback)

Add a third greedy model where `offset_partial` also falls back, to bound the
dual-issue fire rate **without** building a slot-1 splitter. This decides
whether Phase 3 needs the splitter at all. (File:
`oasis-bench/util/analyze_snappy.py`, in the pairing pass / `ElementStats`.)

## Phase 1 — wide single-issue datapath (16 B/cycle)

Still one element per cycle, but lines and the datapath are 16 bytes. Shippable
on its own (helps long-match columns immediately) and isolates the risky memory
and rotator changes from the dual-issue control.

**Types (`vhsnunzip_int_pkg.vhd`)**
- `compressed_stream_single.data`: 8 → 16 bytes; `endi` 3 → 4 bits.
- `compressed_stream_double.data`: 16 → 32 bytes (two 16-byte lines); `start`,
  `endi` widen.
- `command_stream`: `st_addr`, `cp_rol`/`li_rol` (now 0..15), `cp_end`/`li_end`
  (0..16), `lt_swap` per-pair-of-banks; address fields widen.
- `decompressed_stream.data`: 8 → 16 bytes; `cnt` 4 → 5 bits.

**Memory / history**
- Short-term: 16 SRLs (one per byte lane) instead of 8; line = 16 bytes.
- Long-term: 16-byte even/odd lines → **32-byte read window**. A 16-byte URAM
  line = 2 URAM words (72-bit/8 B each), so even+odd = **4 URAMs/core** (was 2).
  Update `vhsnunzip_ram`, `vhsnunzip_port_arbiter`, and the `ram_command` /
  `ram_request` widths. (Block-RAM variant scales similarly.)
- The interleave still exists to give a window twice the line width for the
  rotator; it just doubles.

**Rotator / per-byte mux (`vhsnunzip_pipeline.vhd`, `s1_reg_proc` /
`s2_mux_data_proc`)**
- Generalize `LOOKAHEAD_LOOKUP` and the `s2_rol_sel`/`st_addr`/`li_addr`
  per-byte computation from an 8-byte / 16-window scheme to **16-byte / 32-
  window** (rotate amounts 0..15, `shift` over 16). This is mechanical but is
  the timing-critical block — budget effort here.
- Widen `hold_valid`, `cp_end_th`/`li_end_th` thermometer codes, holding
  register `s3_hold_data`, and the strobe logic to 16 bytes.

**Front-end**
- `vhsnunzip_pre_decoder`: emit 32-byte double lines.
- `vhsnunzip_decoder` / `_long`: seek over 16-byte lines; element headers
  crossing a 16-byte boundary handled by the lookahead line exactly as today.
- `vhsnunzip_cmd_gen_1/2`: split copies/literals into **16-byte** chunks;
  address generation at 16-byte granularity.
- Literal SRL FIFO (`ld_srl_gen`): 16 lanes; one push/pop per cycle still.

**Output**
- Widen `decompressed_stream` and `vhsnunzip_unbuffered`'s output bus to 16 B.

**Verification gate**: update the Python datapath model to 16-byte lines,
regenerate `*.tv`, pass all existing `*_tc.sim.08.vhd` testbenches, then add
long-copy/long-literal directed cases. Single-issue correctness must hold before
Phase 2.

## Phase 2 — dual decode front-end

Emit **two element commands per cycle**.

- **Pre-decoder window**: ensure ≥ two elements + slot-1 header are in view.
  Slot-0 footprint ≤ 5 B (copy4) or ≤ 1+`MAX_LITERAL`; slot-1 header ≤ 5 B → ≤
  ~10 B, inside the 32-byte double window. Confirm; widen to 3 lines only if a
  margin analysis says so.
- **Speculative slot-1 location**: slot-0's compressed footprint is in a tiny
  set — copies 2/3/5 B, short literals 1+`len`. Decode slot-1 candidate headers
  at offsets {2,3,5} (plus the short-literal offsets) **in parallel**, then mux
  on slot-0's decoded size. This keeps the parse off the critical path (≈3
  small header decoders + a mux, vs the ~16-way general scheme).
- **Single-issue fallback at decode**: if slot-0 is a long literal (footprint >
  threshold), issue slot-0 alone this cycle; slot-1 becomes next cycle's slot-0.
- **Output**: a new `dual_element_stream` record = two `element_stream` payloads
  + a `pair_valid`/`slot1_valid` flag. Add `vhsnunzip_decoder_dual`
  (keep the single decoder for the non-dual build via a generic).
- **Literal FIFO**: now up to **2 pushes and 2 pops per cycle** (two literals in
  a pair). This is a real change to `li_level` bookkeeping and the SRL write
  ports — design the FIFO to accept two lines/cycle, or widen each entry.

## Phase 3 — hazard resolution stage

A registered stage between command generation and execute. Inputs: the two
decoded elements + slot-0's `(type, off0, len0)` and its ≤`MAX_LITERAL` literal
bytes. Outputs: the **fused per-byte control** for the 16-byte execute line, with
slot-1's sources already rewritten.

Per slot-1 byte, choose the source:
1. committed history at `off1` (non-hazardous bytes, `j < off1−len0`),
2. committed history at `off1+off0` (slot-0 non-overlapping copy, hazardous
   bytes) — emit slot-1 as one rewritten copy when `off1 ≤ len0` (full overlap),
3. literal-bypass byte (slot-0 short literal) — set the byte's mux to "literal"
   and index slot-0's carried literal bytes,
4. raise `fallback` (slot-0 overlapping copy, or partial overlap if no splitter)
   → demote the pair to single-issue.

All rewrites are address arithmetic / small muxes registered before execute, so
no live slot-0 result enters the execute critical path. Reuse the existing
per-byte machinery (`s2_mux_sel`, `s2_rol_sel`, per-byte `st_addr`/`li_addr`,
`s2_lt_sel`) — this stage produces those signals for slot-1's bytes.

**Open item** (Phase 0.5 decides): if `offset_partial` is common, add a slot-1
splitter (pair slot-0 + slot-1-low at `off1`, defer slot-1-high at `off1+off0` to
the next cycle). If rare, just fall back.

## Phase 4 — dual-issue execute + output assembly

- Drive the 16-byte datapath with the **fused command**: the 16 output bytes are
  laid out as up to four segments — slot-0 copy, slot-0 literal, slot-1 copy,
  slot-1 literal — described by extending the `cp_end`/`li_end` thermometer
  scheme to a 4-boundary layout within the line.
- Both elements' history reads come from the (single) 32-byte window read this
  cycle; the literal-bypass bytes come from registers (Phase 3), not the SRL
  (an SRL write this cycle is not yet readable).
- Output assembly: up to 16 valid bytes/cycle into the holding register / output
  FIFO; handle 0/1/2-element retirement and the partial-line `cnt`/`last`
  bookkeeping (extend the existing `s3_*` holding-register and `s3_last_pend`
  logic).

## Phase 5 — control, fallback, edge cases

- **Fallback path**: `fallback` from Phase 3 (or long-literal slot-0 from Phase
  2) splits the pair → slot-0 this cycle, slot-1 next cycle (a bubble). Same
  throughput as today's single-issue model for those pairs.
- **Backpressure / handshakes**: extend `co_ready`, `de_ready`, the command FIFO
  and the `s1_valid`/`s2_valid` start logic for two-wide issue; verify the
  `lt_rd_*` outstanding-request bound (< 32 minus pipeline depth, per the
  `vhsnunzip_pipeline` comment) still holds with the extra resolution stage.
- **Last-command handling**: the inserted stall cycle for the output holding
  register (`s3_last_pend`) must account for a pair landing on the chunk end.
- **Run-length copies** (`cp_rle`): unchanged within an element, but as slot-1
  they hazard with slot-0 (offset 1 reads the previous byte) → resolved as
  literal-bypass (slot-0 literal) or `off1+off0` (slot-0 copy), else fallback.

## Phase 6 — verification and bring-up

1. **Python model first**: extend the model to dual decode + resolution +
   fallback, bit-exact against `snzip`-produced streams. This is the reference
   for the `*.tv` vectors.
2. **Regenerate `*.tv`** and run all `*_tc.sim.08.vhd` testbenches via
   `vhdeps` (GHDL/Questa), as the README describes.
3. **Directed tests** for the hazard cases: full/partial/residual overlap,
   slot-0 literal straddling the split, two copies in a pair, RLE in slot-1,
   pair on chunk boundary, fallback bubble.
4. **Randomized cosim** of the Python model vs RTL over many compressed pages
   (the `test.py` flow).
5. Synthesize `vhsnunzip_unbuffered` (xcvu5p, as in the README table) to confirm
   f_max is unchanged (the whole point of the registered resolution stage) and
   to capture LUT/URAM cost.

## Risks and expectations

- **Critical path**: the 16:16 rotator / per-byte mux (Phase 1) is the timing
  risk, not the dual-issue control (which is registered). If f_max drops,
  retime the rotator before adding lanes.
- **Resource**: ~2× the datapath logic + 4 URAMs/core (was 2). Per the README
  the unbuffered core is ~1844 LUTs / 2 URAMs, so the absolute cost is small.
- **Literal FIFO at 2 push/2 pop** (Phase 2) and the **4-segment line layout**
  (Phase 4) are the subtlest new logic — budget verification time there.
- **Throughput expectation**: decode rate → ~2× where `resid%` (and
  `offset_partial`, pending Phase 0.5) are small; realized throughput =
  min(2 × element-rate, 16 B/cycle) × f_max. On the measured columns this is a
  genuine ~2×; on already-long-match columns Phase 1 alone gets most of it.

## Suggested sequencing

1. Phase 0.5 (analyzer refinement) — decides whether the slot-1 splitter is
   needed.
2. Phase 1 (wide single-issue) — ship and verify independently.
3. Phases 2–5 (dual issue) — layer on, gated by the Phase 6 verification flow.
