#!/usr/bin/env bash
# What is actually running, and at what rate.
#
#   ./scripts/status.sh            # processes + GPU
#   ./scripts/status.sh --rates    # also measure topic rates (~15 s)
#
# Lives in a script rather than being typed inline for the same reason stop.sh does: any
# command line containing these process names becomes a target for `pkill -f`, and typing
# a check that matches the killer is how you SIGTERM your own shell.
set -uo pipefail
PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_DOMAIN_ID="${TESTBED_ROS_DOMAIN_ID:-42}"   # must match scripts/bringup.sh

# Count by ABSOLUTE INSTALL PATH, never by node name.
#
# `pgrep -f node_name` matches the full command line of every process — including the shell
# that invoked this script, if that shell's command line happens to mention the name. That
# reported 1 of every node while the whole stack was demonstrably down. Paths under this
# repo's install tree do not appear in ordinary shell command lines, and we drop our own
# process ancestry as well, so the count is what is actually running.
ancestry() {
    local pid=$$
    while [ "$pid" -gt 1 ]; do
        echo "$pid"
        pid="$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')"
        [ -z "$pid" ] && break
    done
}
MINE="$(ancestry | tr '\n' '|')0"

count() {  # count() <substring of the process's command line>
    ps -eo pid,cmd --no-headers 2>/dev/null |
        grep -F -- "$1" | grep -v " grep " |
        awk -v mine="^($MINE)$" '$1 !~ mine' | wc -l
}

sim_count() { local n; n="$(pgrep -xc 'CarlaUE4-Linux-' 2>/dev/null)" || true; echo "${n:-0}"; }

echo "== processes =="
printf "  %-18s %s\n" "simulator" "$(sim_count)"
printf "  %-18s %s\n" "sim_bridge" "$(count "$PROJ/sim_bridge/server.py")"
for n in carla_air_bridge vlm_client grounding control evaluation; do
    printf "  %-18s %s\n" "$n" "$(count "$PROJ/ros2_ws/install/$n/lib")"
done
# Everything else stop.sh kills, reported here too. A status that checks LESS than stop
# removes will say "clean" while something is still up: an orphaned web console from an
# earlier session was listening on the mesh for days before `stop.sh` learned to find it,
# and this screen said every count was 0 the whole time.
printf "  %-18s %s\n" "web console" "$(count "webui/server.py")"
printf "  %-18s %s\n" "nav example"  "$(count "navigation/nav.launch.py")"
printf "  %-18s %s\n" "trace recorder" "$(count "bag record")"
printf "  %-18s %s\n" "vlm example"  "$(count "vlm_navigation/vlm.launch.py")"
printf "  %-18s %s\n" "episode/sweep" \
    "$(( $(count "$PROJ/scripts/run_episode.py") + $(count "$PROJ/scripts/run_sweep.sh") ))"
echo "  (anything above 1 means a stacked run — ./scripts/stop.sh, then bring up again)"

echo
echo "== gpu =="
nvidia-smi --query-gpu=index,name,utilization.gpu,memory.used --format=csv,noheader 2>/dev/null |
    sed 's/^/  /'
# Check the card the simulator was actually asked for, not a fixed index.
#
# This read `-i 0` until 2026-08-04, while the shipped config renders on GPU 1. That is the
# exact mistake run_sim.sh warns about beside its own copy of this check: a fixed index is
# wrong the moment TESTBED_GPU points elsewhere, and wrong in BOTH directions. Here it cried
# SOFTWARE RENDERING on every correct run (GPU 0 idle at 115 MiB while the sim held 3.8 GB on
# GPU 1) — and a warning that fires when nothing is wrong is one nobody reads when something
# is. Reverse the GPUs and it would have stayed silent through a genuine lavapipe run, which
# is the failure rule 5 exists for.
GPU_TARGET="${TESTBED_GPU:-$("$PROJ/.venv/bin/python" -c "
import yaml
try:
    d = yaml.safe_load(open('${TESTBED_CONFIG:-$PROJ/configs/testbed.yaml}')) or {}
    g = (d.get('simulator') or {}).get('gpu')
    print('' if g is None else g)
except Exception:
    print('')" 2>/dev/null)}"
case "${GPU_TARGET:-}" in
    ''|None) GPU_TARGET=0 ;;                 # driver's choice, historically device 0
    *:*)     GPU_TARGET="" ;;                # raw vendor:device — no index to query
esac
if [ -n "${GPU_TARGET:-}" ]; then
    vram="$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits \
            -i "$GPU_TARGET" 2>/dev/null | tr -d ' ')"
    if [ -n "${vram:-}" ] && [ "$vram" -lt 1000 ] && [ "$(sim_count)" -gt 0 ]; then
        echo "  WARNING: simulator running but GPU $GPU_TARGET is under 1 GB (${vram} MiB) —"
        echo "           probably SOFTWARE rendering. See docs/architecture.md."
    fi
fi

echo
echo "== sockets =="
ss -tln 2>/dev/null | grep -E ':(2000|41451) ' | sed 's/^/  /' || echo "  no sim ports"
[ -S /tmp/carla_air_testbed.sock ] && echo "  sim_bridge socket present" || echo "  sim_bridge socket MISSING"

if [ "${1:-}" = "--rates" ]; then
    echo
    echo "== topic rates (ROS_DOMAIN_ID=$ROS_DOMAIN_ID) =="
    set +u
    source /opt/ros/jazzy/setup.bash
    source "$PROJ/ros2_ws/install/setup.bash"
    set -u
    for t in /fmu/out/vehicle_odometry /camera/rgb/image_raw /camera/depth/image_raw \
             /vlm/annotation /control/waypoint /fmu/in/trajectory_setpoint; do
        hz="$(timeout 12 ros2 topic hz "$t" 2>/dev/null | grep -m1 'average rate' | awk '{print $3}')"
        printf "  %-34s %s Hz\n" "$t" "${hz:-NO DATA}"
    done
fi
