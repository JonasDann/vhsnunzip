#!/bin/bash
# Regression harness for vhsnunzip.
#
# Generates test vectors with the Python reference model and simulates all
# testbenches with GHDL (via the dockerised shim ./ghdl-docker).
#
# Prerequisites (one-time):
#   - submodules initialised: git submodule update --init
#   - test tools built:       make -C tools
#   - vhdeps installed:        pip3 install --user vhdeps
#   - docker access + image:   docker pull ghdl/ghdl:ubuntu20-mcode
#
# Usage: ./run_sim.sh <input-file> [test.py key=value ...]
set -euo pipefail
cd "$(dirname "$0")"

INPUT="${1:?usage: run_sim.sh <input-file> [key=value ...]}"
shift || true

# Put the dockerised ghdl shim first on PATH and keep temp dirs under /tmp so the
# container mount sees them.
export PATH="$PWD:$PATH"
export TMPDIR=/tmp
ln -sf ghdl-docker ghdl   # vhdeps invokes a binary literally named "ghdl"

python3 test.py "$INPUT" "$@" -- ghdl
