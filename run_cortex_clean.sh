#!/usr/bin/env bash
# Tear down core.state, run a full bootstrap, then run_cortex.
# Forces the full (re-parse-from-USD) bootstrap path so PhysX picks
# up current physxArticulation:fixedBase and excludeFromArticulation
# values — fast-reset can't update those after PhysX has cached them.
#
# Isaac Sim's vscode socket executor returns the response BEFORE async
# scripts finish, so we poll bootstrap.log for the completion sentinel
# before sending run_cortex (otherwise run_cortex starts while USD is
# still loading and probes against a None stage).
#
# Usage:
#   ./run_cortex_clean.sh                     # teardown → bootstrap → run_cortex
#   ./run_cortex_clean.sh --tail              # ... then tail nav_diag.stream.log
#   ./run_cortex_clean.sh --wait              # wait for Isaac Sim to become reachable first
#   ./run_cortex_clean.sh --boot-timeout 240  # raise bootstrap completion timeout (default 180s)
set -euo pipefail

cd "$(dirname "$0")"

WAIT_FLAG=""
TAIL_AFTER=0
BOOT_TIMEOUT=180
while [[ $# -gt 0 ]]; do
    case "$1" in
        --wait) WAIT_FLAG="--wait"; shift ;;
        --tail) TAIL_AFTER=1; shift ;;
        --boot-timeout)
            shift
            [[ $# -gt 0 ]] || { echo "--boot-timeout requires a value" >&2; exit 2; }
            BOOT_TIMEOUT="$1"; shift ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done

LOG_DIR=cache/isaac-sim/logs
BOOT_LOG="$LOG_DIR/bootstrap.log"
DIAG="$LOG_DIR/nav_diag.stream.log"

TEARDOWN_SCRIPT="$(mktemp /tmp/teardown.XXXXXX.py)"
trap 'rm -f "$TEARDOWN_SCRIPT"' EXIT
cat >"$TEARDOWN_SCRIPT" <<'PY'
import os
import sys
sys.path.insert(0, "/workspace/midterm_project")
from core import state
state.teardown()
# bootstrap.log is owned by root (Kit runs as root in the container) so the
# host user can't truncate it. Do it here from inside the kit process — the
# next bootstrap's make_logger("bootstrap") will rewrite it from scratch
# anyway, but truncating now lets the host poll for new completion text
# without race confusion from the previous run's content.
for path in ("/root/.nvidia-omniverse/logs/bootstrap.log",
             "/root/.nvidia-omniverse/logs/run_cortex.log"):
    try:
        open(path, "w").close()
    except Exception as e:
        print(f"could not truncate {path}: {e}")
print("core.state torn down — next bootstrap will take the full path")
PY

echo "=== [1/5] Tearing down core.state + truncating logs ==="
./run_in_isaac.py $WAIT_FLAG "$TEARDOWN_SCRIPT"

echo "=== [2/5] Full bootstrap (re-parses USD, re-inits PhysX) ==="
./run_in_isaac.py midterm_project/apps/bootstrap.py

echo "=== [3/5] Waiting for bootstrap to finish (timeout ${BOOT_TIMEOUT}s) ==="
# Full path ends with "Bootstrap complete.", fast-reset with "Scene reset to defaults".
# After teardown the path is full, so we expect the former.
deadline=$((SECONDS + BOOT_TIMEOUT))
booted=0
while (( SECONDS < deadline )); do
    if [[ -f "$BOOT_LOG" ]] && grep -qE "Bootstrap complete\.|Scene reset to defaults" "$BOOT_LOG"; then
        booted=1
        echo "bootstrap reported complete after $SECONDS s"
        break
    fi
    sleep 2
done
if (( booted == 0 )); then
    echo "ERROR: bootstrap did not complete within ${BOOT_TIMEOUT}s." >&2
    echo "Last bootstrap.log content:" >&2
    [[ -f "$BOOT_LOG" ]] && tail -40 "$BOOT_LOG" >&2 || echo "  (log missing)" >&2
    exit 1
fi

echo "=== [4/5] Cleaning phantom finger prims left by an earlier draft ==="
# Idempotent — only deletes Xforms that don't carry a PhysicsRigidBody /
# ArticulationRoot schema (so the real Franka fingers are never touched).
# See midterm_project/apps/cleanup_phantom_finger_prims.py for the rationale.
./run_in_isaac.py midterm_project/apps/cleanup_phantom_finger_prims.py

echo "=== [5/5] run_cortex ==="
./run_in_isaac.py midterm_project/apps/run_cortex.py

# run_cortex is also async-detached; let it accumulate diag entries before we read.
# 8000 ticks at 60 Hz ≈ 130 s per episode; we don't need to wait that long, just
# enough for the probe + first state-machine tick to land in the diag stream.
echo
echo "Waiting 10s for run_cortex first ticks..."
sleep 10

echo
echo "=== Last 30 lines of $DIAG ==="
tail -30 "$DIAG" 2>/dev/null || echo "(diag file not present yet)"

if [[ $TAIL_AFTER -eq 1 ]]; then
    echo
    echo "=== Following $DIAG (Ctrl-C to stop) ==="
    tail -f "$DIAG"
fi
