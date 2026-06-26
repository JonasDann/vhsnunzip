#!/usr/bin/env python3

import random
import sys
import itertools
from emu.operators import *
from emu.snappy import compress

# Parse command line.
def usage():
    print('Usage: %s <test-data-file> [config=value [...]] \\\n'
          '  [-- <vhdeps target> [vhdeps options...]]' % sys.argv[0], file=sys.stderr)
    sys.exit(2)
try:
    args = iter(sys.argv)
    next(args)
    fname = next(args)
    keys = {key.lower(): value for key, value in map(
        lambda x: tuple(x.split('=', maxsplit=1)),
        itertools.takewhile(lambda x: x != '--', args))}
    vhdeps_target, *vhdeps_args = args
except ValueError:
    vhdeps_target = None
    vhdeps_args = []
except StopIteration:
    usage()

# Seed the random generator.
random.seed(keys.pop('seed', 0))

# Read uncompressed file into memory.
print('Reading input...')
with open(fname, 'rb') as fin:
    data = fin.read()

# Chunk it up randomly.
print('Compressing input...')
chunk_size = int(keys.pop('chunk', '65536'), 0)
min_chunk_size=int(keys.pop('min_chunk', str(chunk_size)), 0)
max_chunk_size=int(keys.pop('max_chunk', str(chunk_size)), 0)
compressed, uncompressed = compress(
    data, 'tools/bin',
    min_chunk_size=min_chunk_size,
    max_chunk_size=max_chunk_size,
    max_prob=float(keys.pop('max_prob', '0')),
    verify=keys.pop('verify', None) != None)

print('Write expected input and output...')
drain(writer(wide_data_source(compressed), '../vhdl/in.tv'))
drain(writer(wide_data_source(uncompressed), '../vhdl/out.tv'))

print('Simulating decompression in Python...')
cs = Counter(writer(data_source(compressed), '../vhdl/cs.tv'))
cd = Counter(writer(pre_decoder(cs), '../vhdl/cd.tv'))
el = Counter(writer(decoder(cd), '../vhdl/el.tv'))
c1 = Counter(writer(cmd_gen_1(el), '../vhdl/c1.tv'))
cm = Counter(writer(cmd_gen_2(c1), '../vhdl/cm.tv'))
cme = writer_exec(cm, '../vhdl/cmde.tv')
de = Counter(writer(datapath(cme), '../vhdl/de.tv'))
drain(verifier(de, uncompressed))

# Verify the speculative dual-issue datapath model against the same data, and
# measure the throughput gain over single-issue. This rebuilds the command
# stream (the single-issue generators above were consumed) and runs the
# dual-issue execution model, which co-issues up to two commands per cycle.
print('Simulating dual-issue decompression in Python...')
dual_counters = {}
drain(verifier(datapath_dual(
    cmd_gen_2(cmd_gen_1(decoder(pre_decoder(data_source(compressed))))),
    dual_counters), uncompressed))

# Verify the SRL-faithful contained-fold model (the bit-exact reference the fold
# datapath RTL mirrors): same short-term-SRL + holding-register scheme as the
# single-issue datapath, doubled, with the same-cycle forward falling out of the
# shared immediate-push SRL.
fold_counters = {}
drain(verifier(datapath_fold(
    cmd_gen_2(cmd_gen_1(decoder(pre_decoder(data_source(compressed))))),
    fold_counters), uncompressed))

# Run vhdeps if requested.
if vhdeps_target is not None:
    print('Checking that VHDL and Python streams match...')
    test_cases = [
        'vhsnunzip_pre_decoder_tc', 'vhsnunzip_decoder_long_tc',
        'vhsnunzip_decoder_dual_tc', 'vhsnunzip_execute_dual_tc',
        'vhsnunzip_cmd_gen_1_tc', 'vhsnunzip_cmd_gen_1_dual_tc',
        'vhsnunzip_cmd_gen_2_tc', 'vhsnunzip_cmd_gen_2_dual_tc',
        'vhsnunzip_cmd_gen_dual_tc',
        'vhsnunzip_pipeline_tc', 'vhsnunzip_pipeline_dual_tc',
        'vhsnunzip_unbuffered_tc', 'vhsnunzip_unbuffered_dual_tc',
    ]
    # The buffered and multicore cores have hardcoded 8-byte-line geometry, so
    # they only build/simulate at the default datapath width (WI == 8). The
    # wide single-issue datapath (WI > 8) targets the unbuffered core only.
    if max_chunk_size <= 65536 and WI == 8:
        test_cases.append('vhsnunzip_tc')
        test_cases.append('vhsnunzip_decoder_tc')
    elif WI != 8:
        print('NOTE: not simulating buffered/multicore core; WI != 8')
    else:
        print('NOTE: not simulating buffered core; chunk size > 64kiB')
    import vhdeps
    code = vhdeps.run_cli(
        [vhdeps_target] + test_cases + ['-i', '..'] + vhdeps_args)
    if code != 0:
        sys.exit(code)

print()
print('Statistics:')
print('  Uncompressed size=%d, compressed size=%d, chunk count=%d' % (
    len(data), sum(map(len, compressed)), len(compressed)))
print('  Stream transfer counts: cs=%d, cd=%d, el=%d, c1=%d, cm=%d, de=%d' % (
    cs.count, cd.count, el.count, c1.count, cm.count, de.count))
print('  Approx. bytes/cycle: %.3f' % (
    len(data) / cm.count))
if dual_counters.get('cycles'):
    print('  Dual-issue (contained fold): single=%d cycles, dual=%d cycles, %d folded (%.1f%%), %d blocked' % (
        dual_counters['single'], dual_counters['cycles'], dual_counters['pairs'],
        100.0 * dual_counters['pairs'] / max(1, dual_counters['single']),
        dual_counters['blocked']))
    print('  Dual-issue forwarding: %d/%d folds hit the RAW hazard, max %d bytes forwarded' % (
        dual_counters['fwd'], dual_counters['pairs'], dual_counters['fwd_max']))
    print('  Dual-issue bytes/cycle: %.3f (%.2fx single-issue)' % (
        len(data) / dual_counters['cycles'],
        dual_counters['single'] / dual_counters['cycles']))
if fold_counters.get('cycles'):
    print('  SRL-fold (RTL reference): single=%d, dual=%d cycles, %d folded -> %.3f B/cyc (%.2fx)' % (
        fold_counters['single'], fold_counters['cycles'], fold_counters['pairs'],
        len(data) / fold_counters['cycles'],
        fold_counters['single'] / fold_counters['cycles']))
    print('  SRL-fold forward overlay: %d cm1 short-term reads cross-checked, %d via forward mux' % (
        fold_counters.get('chk_reads', 0), fold_counters.get('chk_fwd', 0)))

print()
print('All good!')
