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
echo "  (anything above 1 means a stacked run — ./scripts/stop.sh, then bring up again)"

echo
echo "== gpu =="
nvidia-smi --query-gpu=index,name,utilization.gpu,memory.used --format=csv,noheader 2>/dev/null |
    sed 's/^/  /'
vram="$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits -i 0 2>/dev/null | tr -d ' ')"
if [ -n "${vram:-}" ] && [ "$vram" -lt 1000 ] && [ "$(sim_count)" -gt 0 ]; then
    echo "  WARNING: simulator running but GPU 0 under 1 GB — probably SOFTWARE rendering."
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
             /vlm/annotation /vlm/grounded_waypoint /fmu/in/trajectory_setpoint; do
        hz="$(timeout 12 ros2 topic hz "$t" 2>/dev/null | grep -m1 'average rate' | awk '{print $3}')"
        printf "  %-34s %s Hz\n" "$t" "${hz:-NO DATA}"
    done
fi
