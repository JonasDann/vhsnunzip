# R2-lite: 32 B/cycle, 4-slot vhsnunzip — implementation plan

## Goal, target and evidence

Replace today's 16 B dual-issue unbuffered core (`vhsnunzip_pipeline_dual`) with a
32 B/cycle core that retires up to **4 element slots per cycle**. The target is
end-to-end ParCore throughput on the columns where the decompressor is the
bottleneck. These are the columns whose decoded output does not already saturate
the 64 B/cycle decoder stream, mostly PLAIN pages and dictionary pages with wide ids.

The evaluation behind this plan used real Snappy pages sampled from every column of
tpch-30, tpcds-30 and ClickBench (591 columns, 5,084 pages, 1.05 GB):

- **Baseline:** today's RTL, measured in GHDL on every sampled page, with output
  checked bit-exact. It averages 11.77 B/cycle.
- **R2-lite:** a cycle model of this design, which gives 29.0 B/cycle on the same pages.
- **End-to-end:** each page costs max(input/64, decompressor, decoded output/64,
  ids/16) plus the per-page drain/reset (~20 cycles today, ~25 assumed for R2-lite).

Time-weighted speedup vs. today's RTL:

| Column set                                              |  n  | R2-lite | ideal 32 B |
|---------------------------------------------------------|-----|---------|------------|
| decompressor-bound + ParCore-supported types, TPC-H     |  22 |  2.69   |  2.82      |
| decompressor-bound + ParCore-supported types, TPC-DS    | 159 |  2.20   |  2.30      |
| decompressor-bound + ParCore-supported types, ClickBench|  24 |  1.74   |  1.76      |
| **decompressor-bound + ParCore-supported types, all**   | 205 | **2.21**|  2.28      |
| all 591 columns                                         | 591 |  1.51   |  1.57      |

**Resource target:** URAMs unchanged at 8. LUTs are estimated at 2.2–2.4× today's
8,578, which is over the 2× budget. Phase 0 confirms this and decides on area
reduction (see *Area levers*).

## Architecture summary

| Parameter | Value |
|---|---|
| Line / datapath width | `C_BYTES = 32` (input, output, short-term and long-term lines) |
| Element slots per cycle | 4 (`C_SLOTS`), each with one copy path and one literal path |
| Decoder elements per cycle | 4, from a two-line (64 B) window |
| Element FIFO | 32 entries between decoder and command generator |
| Short-term history | 32 lines (1 KiB), SRL, 4 read banks |
| Long-term history | even/odd banks of 1024 × 32 B lines, 4 URAM288 each (8 total), **both URAM ports** |
| Long-term reads | up to 2 per bank per cycle; writes go through an 8-line queue per bank |
| Group invariant | at most 32 output bytes per cycle, so at most one line completes and each lane is written at most once |
| Same-cycle hazards | resolved by copy-source resolution in the generator; **no data forwarding** |
| Overlapping copies | power-of-two offsets (1, 2, 4, 8, 16) replicated in slot 0; other offsets use today's offset doubling |
| Stride scan | copies with offset 4 or 8 filled per lane after the merge (`out[x] = out[x−d]`); they use no copy rotator |

Pipeline:

```
co FIFO → pre-dec → D1 decode-all → D2 boundary loop → D3 field select → element FIFO (4-in/4-out)
        → G1 head precompute → G2 group formation (loop) → group FIFO (+ long-term read issue)
        → S1 per-lane control → S2 data (rotators, merge, stride fill, ST push, hold/out) → de FIFO
                                                                          ↘ long-term write queue
```

### Why each mechanism is there (measured on the corpus)

| Mechanism | Effect on the 205 target columns |
|---|---|
| 4 slots + copy-source resolution (vs. 4 slots, no forwarding) | 1.77× → 1.98× (model baseline) |
| Stride scan for offsets 4 and 8 | +0.07× (more on sorted keys, e.g. `l_orderkey` 2.3 → 2.9×) |
| Two-line decoder window (vs. line-local) | +0.15× |
| Decoder 4 elements/cycle (vs. 3) | +0.17× |
| Both URAM ports with write queue (8 URAM) vs. 3 replicas (12 URAM) | −0.01× |
| Deeper short-term memory (64 lines) | +0.01× (not done) |
| Literal compaction/expansion network | +0.02× (not done) |
| Chained data forwarding instead of source resolution | ±0.01× (not done; long timing chain) |

## Block specifications

### 1. Widths and packages (`vhsnunzip_utils_pkg`, `vhsnunzip_int_pkg`)
- Set `C_BYTES = 32`. This makes `C_IDX = 5`, `C_CNT = 6`, `C_WIN = 6` and `C_AW = 10`.
- Add `C_SLOTS = 4`, `C_DEC_EL = 4`, `C_EL_FIFO = 32`, `C_WQ = 8` and `C_SMAX = 12`.
- New records:
  - `quad_element_stream`: 4 × `element_stream` plus a per-slot valid bit.
  - `slot_cmd`: one slot of a group. Fields:
    - `kind_cp`: none / short-term / long-term / literal-sourced / replicate / stride.
    - `cp_lane_lo`, `cp_lane_hi`: output lanes, 0..32.
    - `cp_rot` (5 bits).
    - `cp_src_line`: short-term relative line or long-term line address.
    - `lt_port`: bank and A/B port.
    - `rep_per`: 1/2/4/8/16.
    - `stride_d`: 4/8.
    - `src_slot`: for literal-sourced copies.
    - `li_lane_lo`, `li_lane_hi`, `li_rot`, `li_line`, `ld_pop`.
  - `group_cmd`: 4 × `slot_cmd`, plus `last`, `out_start` (lane of the first byte)
    and the long-term read bundle (≤ 4 line reads with bank/port/address).
- Keep the 16 B single-issue and dual datapaths building (generic-selected) until R2-lite is verified.

### 2. Long-term memory: `vhsnunzip_lt_mem` (new) + `vhsnunzip_ram.syn.vhd`
- `vhsnunzip_ram` already exposes two read/write ports. Use them as:
  - port B: always reads;
  - port A: a read when the scheduler grants it, otherwise the next queued write.
- One instance per bank (even and odd), with `NU = C_BYTES/8 = 4` URAMs each.
  This replaces today's mirrored banks: the same 8 URAMs, but 4 read ports instead of 2.
- **Write queue:** completed output lines go to the bank given by line parity, in a
  queue of `C_WQ` lines (a 32 B × 8 LUTRAM FIFO per bank). The queue drains into
  port A on cycles without a granted read.
  - When occupancy reaches `C_WQ`, the bank asserts `wq_full` and the generator must
    leave port A free next cycle.
  - This is safe: long-term reads only target lines ≥ 31 lines old, while queued lines
    are at most `C_WQ` + pipeline depth (< 31) old. Assert this in simulation.
- **Read interface:** per cycle, up to 4 read requests tagged {bank, port, address}
  from the generator. Responses return with fixed latency (3 cycles, as today) on a
  per-port `rdat` with a `valid_next` per group. This keeps today's lockstep scheme:
  issue at the group FIFO write side, consume exactly on arrival in S1, and stop issuing
  under backpressure.
- The `RAM_STYLE = "block"` variant needs the same two-port treatment (16 BRAM36 per bank at 32 B).

### 3. Decoder: `vhsnunzip_decoder_quad` (new; replaces `vhsnunzip_decoder_dual`)
- **Pre-decoder:** unchanged structure at 32 B lines; it emits the double-line window.
- **D1, decode-all** (off the loop): for each line offset j (0..31), compute:
  - the element's compressed size `sz[j]` (copy tag 2/3/5 bytes, plus a following
    literal header and its data);
  - `small[j] = sz[j] ≤ C_SMAX`;
  - a slim field record: cp_len 6, cp_off 16, li_len 17 bits (literals ≤ 64 KiB
    because Snappy works in 64 KiB blocks), header length.

  Keep records for the current and lookahead lines (today's depth-2 `cdd` FIFO).
  Also compute `next2[j] = next[next[j]]` for small elements; since `next[j] ∈ j+2..j+12`,
  that is an 11:1 mux per entry.
- **D2, loop:** from `off`, the slot starts are `e0 = off`, `e1 = next[e0]`,
  `e2 = next2[e0]`, `e3 = next[e2]`.
  - Continue to slot k+1 only while slot k is small and its header ends inside the
    two-line window.
  - Loop-carried: `off' = next2[next2[off]]` (or the first stop point). That is two
    serial lookups, the same depth as today's dual decoder.
  - The window advances at most one line per cycle.
- **D3, field select:** gather each slot's record.
  - Baseline: 4 independent 64:1 record muxes (about 0.8K LUTs per slot).
  - Area option: rotate the record window once by `e0`, then select slots 1–3
    relative to it (11:1, 21:1, 31:1).
- **Element FIFO:** 32 entries, 4 pushes and 4 pops per cycle, as 4 LUTRAM banks with
  rotating read/write pointers.
- **Literal SRL credit:** stall the decoder when (decoder input line − oldest
  unconsumed literal line) ≥ 32 − margin, so the literal SRL cannot be overrun now that
  the element FIFO is deep.
- Literal-continuation lines no longer produce "empty element" transfers. They only
  advance the window; the literal SRL is fed from the input stream as today.

### 4. Command generator: `vhsnunzip_cmd_gen_quad` (new; fuses `cmd_gen_1` and `cmd_gen_2`)
The generator implements exactly the reference model's rules. The RTL/model comparison
showed today's coupled front end loses up to 1.9× on some columns against its idealised
rules, so any extra rule here must also go into the model.

- **G1, precompute** (off the loop): for the head remainder and the next 4 elements,
  register lengths, offsets, power-of-two/stride flags, and `off ≥ 32×31`
  (always long-term) / `off < 32` (always within recent lines).
- **G2, group formation** (loop-carried state: output position `p mod 32`, head
  remainder, offset-doubling state, write-queue reservations). Each cycle starts with
  a 32-byte budget and fills slots 0..3. In slot k:
  1. **Copy piece**, of length ≤ remaining budget:
     - Offset 4 or 8, matching the group's stride d (if one is set): mark it as a stride
       piece. It uses no rotator and no memory port.
     - Self-overlapping (offset < piece length):
       - k = 0 and power-of-two offset: replicate.
       - k = 0, other offset: cut the piece to `off` bytes and double the offset (today's rule).
       - k > 0: end the group.
     - k > 0 and the source overlaps this group's output (`src + len > p0`):
       **copy-source resolution**. Find the earlier piece j containing `src`, and cut
       the piece at the end of j (and at its replication period, if j is a replicated
       piece). Then:
       - j is a history copy: `src' = src_j + (src − dst_j)`.
       - j is a literal: source = slot j's literal bank, with a different rotation.
       - j is a stride piece: end the group.
       If the source straddles `p0`, take only the committed prefix.
     - Long-term reads (source line < current line − 31): needs lines l (and l+1 if the
       piece spans two lines) on banks parity(l) / parity(l+1). Grant port B, then A
       (A only if `wq_full` is clear). If only the first line can be granted, cut the
       piece to that line; otherwise end the group.
  2. **Literal piece** (same slot): up to the remaining budget, from this slot's literal SRL bank.
  3. If the element is unfinished, its remainder becomes the next slot (or slot 0 next cycle).
  4. The chunk's `last` element ends the group and adds today's one-cycle flush stall.
- The resolution comparisons are ≤ 4 range checks of 6-bit lane positions plus one
  16-bit add per slot.
  - If G2 misses timing, split it: G2a decides piece boundaries using only lane
    arithmetic; G2b registers and does the source rewriting, long-term addresses and
    port assignment. Port grants then need the per-bank counts from G2a.
- **Output:** a `group_cmd` into the group FIFO. Long-term reads are issued at its write
  side, as today. `cm_ready` follows today's rules plus the read-port grants.

### 5. Datapath: `vhsnunzip_pipeline_quad` (new; replaces `vhsnunzip_pipeline_dual`)
- **S1, per-lane control** (registered):
  - lane ownership: which slot owns each output lane, and whether as copy, literal or
    stride, from the slot lane ranges;
  - per-slot short-term SRL read addresses per source lane (today's `st_addr` ± hold/ahead
    logic, per slot);
  - per-slot even/odd select per lane and port select;
  - per-slot literal SRL addresses (`li_level` ± ahead, with `li_level` threaded through
    the slots' `ld_pop`s);
  - stride source index per lane (j ∈ 1..8 for d = 4, 1..4 for d = 8, or "previous line");
  - the `hold_valid` / internal and external strobes (today's code, merged over 4 slots).
- **S2, data**:
  - Per slot:
    - copy source select: short-term bank, long-term even/odd × port A/B, or slot j's
      literal bank (literal-sourced copies);
    - copy rotator: slot 0 has power-of-two replicate + 32-lane rotator (984 LUTs);
      slots 1–3 have plain rotators (640 LUTs each);
    - literal rotator (640 LUTs each) from its literal SRL bank.
  - Per-lane 8:1 merge (512 LUTs).
  - Stride fill: `out[x] = merged[x − d·j]` or the previous output line (held register).
  - Short-term SRL push (per lane at most once), holding register and output line —
    today's `s2_reg_proc`, unchanged.
  - The stride fill stays in S2 so the next group's short-term reads see final bytes.
    If that misses timing, move it to S2b and add a "previous group line" bypass input
    to each slot's copy source select.
- **Memories:** 4 short-term SRL banks × 32 lanes and 4 literal SRL banks × 32 lanes, all 32 deep.
- Remove the forward overlay (`s2_fwd1`, bank B substitution).

### 6. Toplevel and ParCore integration
- `vhsnunzip_unbuffered`:
  - replace `DUAL_ISSUE` with a `DATAPATH` generic ("single" | "dual" | "quad");
  - instantiate `vhsnunzip_lt_mem`;
  - widen the `co_*` / `de_*` ports to 32 B.
- ParCore `VHSNUnzipWrapper`: set `DECOMP_DATA_BYTES = 32`; the 64 B beat now packs two lines.
- **Optional, ParCore side:** pipeline pages instead of draining and resetting the
  decompressor after each page (vhsnunzip already handles chunk boundaries via `last`).
  This doesn't change the time-weighted number but removes small-page slowdowns:
  about 100 tiny-table columns are slightly slower without it.

## Verification

- **Reference models (`tests/emu`):** add `decoder_quad`, `cmd_gen_quad` and
  `datapath_quad` as bit-exact, SRL/URAM-faithful Python generators producing `*.tv`
  per stage, like today's flow. `cmd_gen_quad` asserts every resolved copy reads
  committed (or same-group literal) bytes, mirroring the C model check, which saw
  0 violations over 1.5–11 million resolved copies.
- **Throughput model (`tests/perf/`):** check in the C cycle model (`sim.c`), the corpus
  extraction/sampling scripts, the ParCore end-to-end model and the GHDL throughput
  bench (`perf_tc`, with the fast-SRL simulation model).
  - Regression: RTL cycles per page must be within ±2% of the model (stricter than
    today's 9.6% gap, because the front end now follows the model's rules by construction).
- **Directed tests:**
  - copy offsets 1, 2, 3, 4, 8, 16, 31, 32, 33 and 64 at every lane alignment;
  - source resolution through a copy, a literal, a replicated piece and a stride piece;
  - long-term reads exactly at the 31/32-line boundary, and pieces spanning two lines;
  - both banks' ports exhausted, and the write queue reaching `C_WQ`;
  - the literal SRL credit stall; literals ≥ 64 KiB − 1; chunks > 64 KiB;
  - `last` on every slot position; empty and one-byte chunks.
- **Randomised:** synthetic Snappy streams (an offset/length/literal-mix generator) plus
  all 5,084 corpus pages, bit-exact through RTL.

## Phases and gates

0. **Infrastructure and area prototype (about 1 week).**
   - Check in the model, corpus scripts and throughput bench.
   - Add the literal SRL credit and group-pipeline latency to the model; re-run the corpus.
   - Synthesise out-of-context (xcu55c, 250 MHz): D3 field select, the G2 loop, and one
     S2 slot plus the merge and stride fill.
   - **Gate:** model ≥ 2.15× on target columns; projected LUTs ≤ 2.4×, otherwise apply
     the area levers or fall back to R3 (3 slots, 2.02×).
1. **32 B single-issue + two-port long-term memory (1–2 weeks).**
   - `C_BYTES = 32` on today's single-issue pipeline; `vhsnunzip_lt_mem` with both ports
     and the write queue; regenerate vectors with `WI = 32`.
   - **Gate:** all single-issue testbenches pass; synthesis gives f_max and 32-lane area.
2. **Command generator (2–3 weeks).**
   - `cmd_gen_quad` + its Python model, fed by an element-at-a-time decoder adapter.
   - **Gate:** bit-exact `group_cmd` vectors; per-page group counts match the C model
     (with the decoder rate taken out).
3. **Datapath (2–3 weeks).**
   - `pipeline_quad` S1/S2 and the memories.
   - **Gate:** `unbuffered_quad_tc` bit-exact on the corpus subset.
4. **Decoder (2 weeks).**
   - `decoder_quad`, the element FIFO and the literal credit.
   - **Gate:** element stream matches the model; end-to-end cycles within ±2% of the model on all pages.
5. **Integration (1–2 weeks).**
   - Full corpus through RTL; ParCore wrapper at 32 B; optional page pipelining.
   - **Gate:** ≥ 2.1× time-weighted on the target columns (same ParCore model, measured
     baseline), bit-exact on all pages.
6. **Timing closure and area (1–2 weeks).** 250 MHz on xcu55c-2L (today's core has only +17 ps of slack).

## Resource estimate (before Phase 0 synthesis)

| Block | Today (measured, dual 16 B) | R2-lite (estimate) |
|---|---|---|
| Pre-decoder + decoder | ~2.8K LUT, 3.6K FF | 4.5–5.5K LUT, 2–3K FF |
| Command generation | 0.7K | 2.0–2.5K |
| S1 control | ~0.9K | ~2.2K |
| S2 copy + literal rotators, merge, stride fill | ~3.3K (with SRLs) | ~7.8K |
| Short-term + literal SRL banks | (included above) | ~2.0K |
| FIFOs, write queues | ~0.5K | ~1.3K |
| **Total LUTs** | **8,578** | **~19–21K (2.2–2.4×)** |
| URAM | 8 | **8** |

### Area levers, in order of throughput cost
1. Rotate-then-relative D3 field select: saves about 0.8K. No throughput cost.
2. Share literal SRL banks between slot pairs (a two-line literal window): saves about
   0.5K. Needs a model check.
3. Drop the stride scan (R2-lite without stride, 2.14×): saves about 0.8K.
4. Fall back to R3: 3 slots, 3-element decoder, 2.02×, about 16.5–18K LUTs.

## Risks
- **Area:** above the 2× budget by the current estimate. Phase 0 decides.
- **Timing:**
  - G2 loop (4-slot running sum + source resolution + port grants): split into G2a/G2b.
  - S2 path (SRL → source select → rotator → merge → stride fill → SRL D): move the
    stride fill to S2b with a bypass.
  - D2 loop: two serial lookups, as today.
- **Model fidelity:** today's core is 9.6% slower than its idealised model. R2-lite's
  front end must implement the model's rules exactly; the ±2% per-page regression enforces it.
- **Literal SRL overrun** with the deep element FIFO: the credit stall, modelled in Phase 0.
- **Write-queue starvation:** handled by the port A reservation at `C_WQ` (modelled; −0.01×).
