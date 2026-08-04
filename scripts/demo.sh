#!/usr/bin/env bash
# One command: simulator up, VLM flies a scenario, video out, everything stopped.
#
#   ./scripts/demo.sh
#   ./scripts/demo.sh --scenario cross_the_plaza --backend geometric --seed 2
#
# Prints the path of a single combined MP4 at the end: the chase camera with the drone's own
# view and the depth buffer inset, and the model's reasoning drawn on the frame.
#
# This exists because the sequence is five commands across three terminals - bring up, start
# the example, run the episode, combine, stop - and the fifth one is the one people forget.
# `stop.sh --all` runs from a trap here, so the simulator does not survive a Ctrl-C.
set -uo pipefail

PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG="$PROJ/configs/testbed.yaml"
SCENARIO="street_level"
BACKEND="claude"
SEED=5
KEEP=0

usage() {
    cat <<'__HELP__'
demo.sh - run the simulator, fly one scenario with a VLM, and produce one video.

SYNOPSIS
  ./scripts/demo.sh [--scenario NAME] [--backend NAME] [--seed N] [--config PATH] [--keep-up]

OPTIONS
  --scenario NAME   default street_level   (see ros2_ws/src/evaluation/scenarios/default.yaml)
  --backend NAME    default claude         (claude | geometric | oracle | mock | scripted)
  --seed N          default 5
  --config PATH     default configs/testbed.yaml
  --keep-up         leave the simulator running afterwards (default: stop everything)
  -h, --help        this text

OUTPUT
  out/demo/<episode-id>.mp4   chase camera, with the drone view and depth inset

NOTES
  * `claude` needs ANTHROPIC_API_KEY. Put it in .env (git-ignored, mode 0600); the VLM
    example sources it.
  * Everything is stopped on exit, including on Ctrl-C, unless --keep-up is given.
__HELP__
}

need_val() { [ "$2" -ge 2 ] || { echo "ERROR: $1 needs a value" >&2; exit 2; }; }

while [ $# -gt 0 ]; do
    case "$1" in
        --scenario=*) SCENARIO="${1#*=}"; shift ;;
        --scenario)   need_val "$1" $#; SCENARIO="$2"; shift 2 ;;
        --backend=*)  BACKEND="${1#*=}"; shift ;;
        --backend)    need_val "$1" $#; BACKEND="$2"; shift 2 ;;
        --seed=*)     SEED="${1#*=}"; shift ;;
        --seed)       need_val "$1" $#; SEED="$2"; shift 2 ;;
        --config=*)   CONFIG="${1#*=}"; shift ;;
        --config)     need_val "$1" $#; CONFIG="$2"; shift 2 ;;
        --keep-up)    KEEP=1; shift ;;
        -h|--help)    usage; exit 0 ;;
        *) echo "unknown argument: $1" >&2; echo >&2; usage >&2; exit 2 ;;
    esac
done
[ -f "$CONFIG" ] || { echo "ERROR: no such config: $CONFIG" >&2; exit 2; }

mkdir -p "$PROJ/out/demo"
LOGS="$PROJ/out/demo"

CLEANED=0
cleanup() {
    # Once only. Signals used to run this and then let the script continue: an interrupted
    # run tore the simulator down, printed "verified clean", then walked into step 3 and
    # tried to fly an episode against a stack that was no longer there.
    [ "$CLEANED" -eq 1 ] && return
    CLEANED=1
    if [ "$KEEP" -eq 1 ]; then
        echo
        echo "left running (--keep-up). Stop it with: ./scripts/stop.sh --all"
        return
    fi
    echo
    echo "stopping everything..."
    "$PROJ/scripts/stop.sh" --all || true
    # stop.sh reports what it signalled; status.sh reports what is actually left. They
    # disagree when something ignores TERM, and the whole point of stopping is that the next
    # bringup starts clean - so verify rather than trust, and say so if it is not clean.
    local left
    left="$("$PROJ/scripts/status.sh" 2>/dev/null)"
    if printf '%s\n' "$left" | grep -qE '^\s+\S.*\s[1-9][0-9]*$'; then
        echo
        echo "WARNING: something survived the stop —"
        printf '%s\n' "$left" | sed 's/^/  /'
        echo "  investigate before the next run; a leftover graph stacks on the next bringup."
    else
        printf '%s\n' "$left" | grep -E "^  [01], NVIDIA" | sed 's/^/  gpu /'
        echo "  verified clean"
    fi
}
# EXIT does the work; INT/TERM only stop the script, which then reaches EXIT. Trapping
# cleanup directly on INT would tear down and RESUME at the next line.
trap cleanup EXIT
trap 'echo; echo "interrupted."; exit 130' INT TERM

wait_for() {   # wait_for <file> <pattern> <seconds> <what>
    for _ in $(seq 1 "$3"); do
        grep -q "$2" "$1" 2>/dev/null && return 0
        sleep 1
    done
    echo "ERROR: $4 did not start within $3 s — see $1" >&2
    tail -15 "$1" >&2
    return 1
}

echo "=== 1/5  simulator + ROS 2 graph ==="
setsid "$PROJ/scripts/bringup.sh" --config "$CONFIG" > "$LOGS/bringup.log" 2>&1 &
wait_for "$LOGS/bringup.log" "hardware rendering confirmed" 180 "the simulator" || exit 1
grep -m1 "hardware rendering confirmed" "$LOGS/bringup.log" | sed 's/^/  /'
wait_for "$LOGS/bringup.log" "bridged to CARLA-Air" 60 "the graph" || exit 1

echo "=== 2/5  navigation example (controller + episode runner) ==="
# Not started by bringup since 2026-08-04: waypoint following and episode scoring are out of
# scope for the simulator. Without this an episode has nothing to score.
setsid "$PROJ/examples/navigation/run.sh" > "$LOGS/nav.log" 2>&1 &
wait_for "$LOGS/nav.log" "offboard_control\|episode_runner" 60 "the navigation example" || exit 1

echo "=== 3/5  VLM example (backend=$BACKEND) ==="
setsid "$PROJ/examples/vlm_navigation/run.sh" --backend "$BACKEND" > "$LOGS/vlm.log" 2>&1 &
wait_for "$LOGS/vlm.log" "VLM backend" 90 "the VLM example" || exit 1
sleep 5

echo "=== 4/5  flying $SCENARIO seed $SEED ==="
"$PROJ/scripts/run_episode.sh" --scenario "$SCENARIO" --seeds "$SEED" 2>&1 \
    | tee "$LOGS/episode.log" | grep -E "^  |^=== " || true

EPISODE="$(grep -oE 'episode running \[[^]]+\]' "$LOGS/episode.log" | head -1 | tr -d '[]' | awk '{print $3}')"
if [ -z "${EPISODE:-}" ]; then
    echo "ERROR: no episode id in the run output — nothing to combine" >&2
    exit 1
fi

CHASE="$PROJ/out/chase/$EPISODE.mp4"
ONBOARD="$PROJ/out/videos/$EPISODE.mp4"
DEPTH="$PROJ/out/videos/$EPISODE-depth.mp4"
OUT="$PROJ/out/demo/$EPISODE.mp4"

echo "=== 5/5  combining ==="
for f in "$CHASE" "$ONBOARD"; do
    [ -s "$f" ] || { echo "ERROR: missing or empty $f" >&2; exit 1; }
done
# Depth is optional: it only exists when graph.recorder.record_depth is on.
"$PROJ/.venv/bin/python" "$PROJ/scripts/combine_views.py" \
    "$CHASE" "$ONBOARD" "$OUT" $([ -s "$DEPTH" ] && echo "$DEPTH") || exit 1

echo
echo "=============================================================="
grep -oE '\-> (SUCCESS|FAILURE \([a-z_]+\)).*' "$LOGS/episode.log" | tail -1 | sed 's/^/  /'
echo "  video: $OUT"
ls -la "$OUT" | awk '{printf "         %.1f MB\n", $5/1e6}'
echo "=============================================================="
