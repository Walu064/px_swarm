#!/usr/bin/env bash
set -euo pipefail

COUNT=3
WORLD="default"
WORLD_NAME=""
MODEL="x500"
SPACING=6
Z=0.3
HEADLESS=0
DRY_RUN=0
PX4_DIR="${PX4_AUTOPILOT_DIR:-$HOME/PX4-Autopilot}"
MODEL_STORE="${PX_SWARM_GZ_MODEL_STORE:-${HIVE_GZ_MODEL_STORE:-$PX4_DIR/Tools/simulation/gz}}"

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Start Gazebo (gz sim) and spawn N drone models in a grid.

Options:
  --count N               Number of drones to spawn (default: $COUNT)
  --world NAME            World SDF name without extension (default: $WORLD)
  --world-name NAME       Gazebo world entity name (default: same as --world)
  --model NAME            Model directory name in model store (default: $MODEL)
  --spacing M             XY spacing between drones in meters (default: $SPACING)
  --z M                   Spawn altitude in world frame (default: $Z)
  --px4-dir PATH          PX4-Autopilot directory (default: $PX4_DIR)
  --model-store PATH      Model store root (default: $MODEL_STORE)
  --headless              Run gz sim server-only
  --dry-run               Print actions, do not execute
  -h, --help              Show help

Example:
  $(basename "$0") --count 5 --world default --model x500 --spacing 8
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --count) COUNT="$2"; shift 2 ;;
    --world) WORLD="$2"; shift 2 ;;
    --world-name) WORLD_NAME="$2"; shift 2 ;;
    --model) MODEL="$2"; shift 2 ;;
    --spacing) SPACING="$2"; shift 2 ;;
    --z) Z="$2"; shift 2 ;;
    --px4-dir) PX4_DIR="$2"; MODEL_STORE="${PX_SWARM_GZ_MODEL_STORE:-${HIVE_GZ_MODEL_STORE:-$PX4_DIR/Tools/simulation/gz}}"; shift 2 ;;
    --model-store) MODEL_STORE="$2"; shift 2 ;;
    --headless) HEADLESS=1; shift ;;
    --dry-run) DRY_RUN=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "[gazebo] Unknown arg: $1"; usage; exit 2 ;;
  esac
done

if [[ -z "$WORLD_NAME" ]]; then
  WORLD_NAME="$WORLD"
fi

if ! [[ "$COUNT" =~ ^[0-9]+$ ]] || (( COUNT < 1 )); then
  echo "[gazebo] Invalid --count: $COUNT"
  exit 2
fi

if ! command -v gz >/dev/null 2>&1; then
  echo "[gazebo] 'gz' command not found. Install Gazebo Harmonic/Garden tools."
  exit 1
fi

MODEL_DIR="$MODEL_STORE/models/$MODEL"
MODEL_SDF="$MODEL_DIR/model.sdf"
WORLD_SDF="$MODEL_STORE/worlds/$WORLD.sdf"
SERVER_CONFIG="$MODEL_STORE/server.config"

if (( DRY_RUN == 0 )); then
  if [[ ! -f "$MODEL_SDF" ]]; then
    echo "[gazebo] Missing model SDF: $MODEL_SDF"
    echo "[gazebo] Expected PX4 Gazebo model at: $PX4_DIR/Tools/simulation/gz/models/$MODEL/model.sdf"
    echo "[gazebo] Override with --model-store PATH or PX_SWARM_GZ_MODEL_STORE (or legacy HIVE_GZ_MODEL_STORE) if your PX4 tree is elsewhere."
    exit 1
  fi
  if [[ ! -f "$WORLD_SDF" ]]; then
    echo "[gazebo] Missing world SDF: $WORLD_SDF"
    echo "[gazebo] Expected PX4 Gazebo world at: $PX4_DIR/Tools/simulation/gz/worlds/$WORLD.sdf"
    exit 1
  fi
fi

export GZ_SIM_RESOURCE_PATH="$MODEL_STORE/models:$MODEL_STORE/worlds"
export GZ_SIM_SERVER_CONFIG_PATH="$SERVER_CONFIG"

SIM_CMD=(gz sim -r "$WORLD_SDF")
if (( HEADLESS == 1 )); then
  SIM_CMD+=(-s)
fi

echo "[gazebo] model_store=$MODEL_STORE"
echo "[gazebo] world=$WORLD_NAME ($WORLD_SDF)"
echo "[gazebo] model=$MODEL count=$COUNT spacing=${SPACING}m z=${Z}m"
echo "[gazebo] Starting: ${SIM_CMD[*]}"

if (( DRY_RUN == 1 )); then
  for ((i=0; i<COUNT; i++)); do
    row=$(( i / 5 ))
    col=$(( i % 5 ))
    x=$(( col * SPACING ))
    y=$(( row * SPACING ))
    echo "[dry-run] spawn drone_$i at x=$x y=$y z=$Z"
  done
  exit 0
fi

"${SIM_CMD[@]}" >/tmp/px_swarm_gz_sim.log 2>&1 &
GZ_PID=$!
echo "[gazebo] gz sim pid=$GZ_PID (log: /tmp/px_swarm_gz_sim.log)"

cleanup() {
  if kill -0 "$GZ_PID" 2>/dev/null; then
    kill "$GZ_PID" 2>/dev/null || true
  fi
}
trap cleanup INT TERM

# Give gz some time to initialize world services.
sleep 3

for ((i=0; i<COUNT; i++)); do
  row=$(( i / 5 ))
  col=$(( i % 5 ))
  x=$(( col * SPACING ))
  y=$(( row * SPACING ))
  name="${MODEL}_${i}"

  sdf_str="<sdf version=\"1.6\"><include><uri>file://${MODEL_SDF}</uri><pose>${x} ${y} ${Z} 0 0 0</pose></include></sdf>"
  req="name: \"${name}\", allow_renaming: false, sdf: '${sdf_str}'"

  echo "[gazebo] Spawning ${name} at (${x}, ${y}, ${Z})"
  gz service -s "/world/${WORLD_NAME}/create" \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req "$req" >/tmp/px_swarm_gz_spawn_${i}.log 2>&1 || {
      echo "[gazebo] Failed to spawn ${name}. Check /tmp/px_swarm_gz_spawn_${i}.log"
      exit 1
    }
done

echo "[gazebo] Spawned $COUNT models. Gazebo is running."
echo "[gazebo] Note: this starts Gazebo models only. Start PX4 SITL separately to get MAVLink on 14540+."
echo "[gazebo] Press Ctrl+C to stop this script and terminate gz sim."
wait "$GZ_PID"
