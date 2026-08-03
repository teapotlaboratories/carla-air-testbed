#!/usr/bin/env bash
# E-01: turn single-seed markers into success rates.
#
#   TESTBED_GPU=1 ./scripts/run_sweep.sh                    # default: 5 seeds, oracle+geometric
#   SEEDS="1 2 3" BACKENDS="oracle" ./scripts/run_sweep.sh
#
# Runs every scenario against every backend, then writes a results table. Budget roughly
# (episode timeout x seeds x scenarios x backends) of WALL CLOCK — sweeps run in real time
# and ClockSpeed cannot help, because it accelerates AirSim while CARLA stays at 1x and the
# two halves of the world desync.
#
# The backend is a launch parameter, so the ROS graph is restarted between backends rather
# than reconfigured. Everything is stopped at the end, per the project rule.
set -uo pipefail

PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SEEDS="${SEEDS:-1 2 3 4 5}"
BACKENDS="${BACKENDS:-oracle geometric}"
SCENARIOS="${SCENARIOS:-cross_the_plaza follow_the_avenue rain_descent avoid_the_block}"
OUT="$PROJ/out/sweep-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$OUT"
START_EPOCH="$(date +%s)"

RELEASE="$("$PROJ/scripts/release_path.sh")"
[ -d "$RELEASE" ] || { echo "no release at $RELEASE - run ./scripts/install.sh" >&2; exit 1; }
export CARLAAIR_RELEASE="$RELEASE"

cleanup() { "$PROJ/scripts/stop.sh" --all >/dev/null 2>&1 || true; }
trap cleanup EXIT INT TERM

n_total=$(( $(echo $SEEDS | wc -w) * $(echo $SCENARIOS | wc -w) * $(echo $BACKENDS | wc -w) ))
echo "sweep: $n_total episodes — backends [$BACKENDS] x scenarios [$SCENARIOS] x seeds [$SEEDS]"
echo "       results -> $OUT"
echo

for backend in $BACKENDS; do
    echo "=== bringing the graph up with backend=$backend ==="
    "$PROJ/scripts/stop.sh" --all >/dev/null 2>&1
    sleep 3
    nohup "$PROJ/scripts/bringup.sh" --backend "$backend" > "$OUT/bringup-$backend.log" 2>&1 &
    # Wait for the whole graph, not just the simulator: the episode service is the last
    # thing to appear and is what run_episode.py actually needs.
    ok=0
    for _ in $(seq 1 60); do
        sleep 5
        if grep -q "episode runner ready" "$OUT/bringup-$backend.log" 2>/dev/null; then ok=1; break; fi
    done
    if [ "$ok" -ne 1 ]; then
        echo "  ERROR: graph did not come up for $backend — see $OUT/bringup-$backend.log" >&2
        tail -5 "$OUT/bringup-$backend.log" >&2
        continue
    fi
    grep -m1 "hardware rendering confirmed" "$OUT/bringup-$backend.log" | sed 's/^/  /'

    for scenario in $SCENARIOS; do
        echo "  --- $backend / $scenario ---"
        "$PROJ/.venv/bin/python" "$PROJ/scripts/run_episode.py" \
            --scenario "$scenario" --seeds $SEEDS \
            > "$OUT/$backend-$scenario.log" 2>&1
        grep -E "^  -> |succeeded" "$OUT/$backend-$scenario.log" | sed 's/^/    /'
    done
done

echo
echo "=== collating ==="
# Only episodes written during THIS sweep. out/episodes/ accumulates across runs, and
# globbing all of it would silently fold the earlier single-seed results into the rates.
"$PROJ/.venv/bin/python" - "$OUT" "$START_EPOCH" <<'PY'
import json, os, sys, glob, statistics

out, start = sys.argv[1], float(sys.argv[2])
epdir = os.path.join(os.path.dirname(out), "episodes")

eps = []
for f in glob.glob(os.path.join(epdir, "*.json")):
    if os.path.getmtime(f) < start:
        continue
    try:
        eps.append(json.load(open(f)))
    except (OSError, json.JSONDecodeError):
        pass

if not eps:
    print("no episodes from this sweep — check the per-scenario logs")
    raise SystemExit(1)

by = {}
for e in eps:
    by.setdefault((e.get("backend") or "?", e.get("scenario")), []).append(e)

print(f"\n{'backend':<11}{'scenario':<21}{'N':>3}{'ok':>4}{'rate':>7}{'median final':>14}  failure modes")
print("-" * 84)
table = []
for (backend, scen), g in sorted(by.items()):
    n, ok = len(g), sum(1 for x in g if x.get("success"))
    dists = [x["final_distance_m"] for x in g
             if isinstance(x.get("final_distance_m"), (int, float)) and x["final_distance_m"] == x["final_distance_m"]]
    med = statistics.median(dists) if dists else float("nan")
    modes = {}
    for x in g:
        if not x.get("success"):
            modes[x.get("failure_mode", "?")] = modes.get(x.get("failure_mode", "?"), 0) + 1
    print(f"{backend:<11}{scen:<21}{n:>3}{ok:>4}{ok/n*100:>6.0f}%{med:>13.1f} m  "
          + (", ".join(f"{k}x{v}" for k, v in sorted(modes.items())) or "-"))
    table.append(dict(backend=backend, scenario=scen, n=n, successes=ok, rate=ok / n,
                      median_final_distance_m=med, failure_modes=modes))

for backend in sorted({t["backend"] for t in table}):
    rows = [t for t in table if t["backend"] == backend]
    n = sum(t["n"] for t in rows); ok = sum(t["successes"] for t in rows)
    print(f"{backend:<11}{'ALL':<21}{n:>3}{ok:>4}{ok/n*100:>6.0f}%")

json.dump(table, open(os.path.join(out, "summary.json"), "w"), indent=2)
print(f"\nwrote {os.path.join(out, 'summary.json')}")
PY

echo
echo "sweep complete — everything stopped"
