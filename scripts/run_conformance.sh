#!/usr/bin/env bash
# Conformance suite: does the simulator still behave the way the testbed assumes?
#
# These are the probes that established the testbed's ground truth — capture rates, the
# grounding transform, the post-reset runaway, the frame offsets. Run them after a
# simulator upgrade, a settings.json change, or any time an episode result looks wrong:
# they answer "did the simulator move?" before you go looking for a bug in your own code.
#
#   ./scripts/run_sim.sh              # first: bring the simulator up
#   ./scripts/run_conformance.sh      # then: this  (~15 min of real-time flight)
#
# Three probes are EXPECTED to fail. They encode defects that are real and unfixed:
#   p09_hover_hold      the vehicle runs away after reset() unless commanded
#   p06_air_ground_sync traffic-manager vehicles stall intermittently
#   p07_ros2_interop    Jazzy (3.12) cannot load the cpython-310 CARLA module
# If any of those three starts PASSING, the simulator changed and the testbed's
# workarounds should be revisited.
#
# Each probe resets the vehicle, so they are independent and order does not
# matter — but they are slow (flights are real-time), budget ~15 minutes.
set -uo pipefail

PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY="$PROJ/.venv/bin/python"
mkdir -p "$PROJ/out"

PROBES=(
    p01_rpc_connect
    p09_hover_hold
    p02_image_throughput
    p03_modalities
    p04_velocity_loop
    p10_join_convergence
    p05_pixel_to_waypoint
    p06_air_ground_sync
    p08_determinism
    p07_ros2_interop
)

if ! ss -tln 2>/dev/null | grep -q ":41451 "; then
    echo "AirSim RPC (41451) is not listening — run ./scripts/run_sim.sh first" >&2
    exit 1
fi

declare -A RESULT
for probe in "${PROBES[@]}"; do
    echo
    "$PY" -u "$PROJ/tests/conformance/$probe.py" 2>&1 |
        grep -v "^Connected\|^Client Ver\|^$\|^WARNING: Version\|^WARNING: Client\|^WARNING: Simulator"
    RESULT[$probe]=${PIPESTATUS[0]}
done

echo
echo "===================================================================="
echo "  summary"
echo "===================================================================="
fails=0
for probe in "${PROBES[@]}"; do
    if [ "${RESULT[$probe]}" -eq 0 ]; then
        printf "  %-24s all checks passed\n" "$probe"
    else
        printf "  %-24s FAILED (exit %s)\n" "$probe" "${RESULT[$probe]}"
        fails=$((fails + 1))
    fi
done
echo "  per-probe JSON in out/"
exit $((fails > 0))
