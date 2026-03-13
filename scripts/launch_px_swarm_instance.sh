#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN_DEFAULT="$PROJECT_ROOT/build/px_swarm_node"

COUNT=3
BASE_PORT=14540
SWARM_IP="239.255.0.1"
SWARM_PORT=9000
MISSION="$PROJECT_ROOT/configs/mission_simple.yaml"
LEADER_MAX_SPEED=""
START_PX4=1
PX4_DIR="${PX4_AUTOPILOT_DIR:-$HOME/PX4-Autopilot}"
PX4_BUILD_DIR=""
GZ_WORLD="default"
GZ_MODEL="x500"
TERMINAL="auto"
TITLE_PREFIX="PX-SWARM"
SWARM_SIZE=""
LOG_DIR="$PROJECT_ROOT/logs"
FLIGHT_TAG=""

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Options:
  --count N                      Number of drones (leader + followers), default: $COUNT
  --base-port PORT               Base MAVSDK port, default: $BASE_PORT
  --swarm-ip IP                  Swarm UDP IP, default: $SWARM_IP
  --swarm-port PORT              Swarm UDP port, default: $SWARM_PORT
  --bin PATH                     Path to px_swarm_node binary, default: $BIN_DEFAULT
  --mission PATH                 Mission file for leader (optional), default: $MISSION
  --leader-max-speed MPS         Leader mission speed cap (optional)
  --swarm-size N                 Value passed to --swarm-size (default: --count)
  --start-px4                    Start PX4 SITL instances for Gazebo gz (default)
  --no-start-px4                 Do not start PX4 SITL instances
  --px4-dir PATH                 PX4-Autopilot directory, default: $PX4_DIR
  --px4-build-dir PATH           PX4 SITL build dir, default: <px4-dir>/build/px4_sitl_default
  --gz-world NAME                Gazebo world entity name to attach to, default: $GZ_WORLD
  --gz-model NAME                Gazebo model prefix, default: $GZ_MODEL
  --log-dir PATH                 Base log directory, default: $LOG_DIR
  --flight-tag TAG               Explicit log run tag (default: auto-generated)
  --terminal auto|gnome|konsole|xterm
  --title-prefix PREFIX          Terminal title prefix, default: $TITLE_PREFIX
  -h, --help                     Show help

Examples:
  $(basename "$0") --count 5 --mission configs/mission_simple.yaml --leader-max-speed 6
  $(basename "$0") --count 10 --no-start-px4 --terminal gnome
EOF
}

BIN="$BIN_DEFAULT"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --count) COUNT="$2"; shift 2 ;;
    --base-port) BASE_PORT="$2"; shift 2 ;;
    --swarm-ip) SWARM_IP="$2"; shift 2 ;;
    --swarm-port) SWARM_PORT="$2"; shift 2 ;;
    --bin) BIN="$2"; shift 2 ;;
    --mission) MISSION="$2"; shift 2 ;;
    --leader-max-speed) LEADER_MAX_SPEED="$2"; shift 2 ;;
    --swarm-size) SWARM_SIZE="$2"; shift 2 ;;
    --start-px4) START_PX4=1; shift ;;
    --no-start-px4) START_PX4=0; shift ;;
    --px4-dir) PX4_DIR="$2"; shift 2 ;;
    --px4-build-dir) PX4_BUILD_DIR="$2"; shift 2 ;;
    --gz-world) GZ_WORLD="$2"; shift 2 ;;
    --gz-model) GZ_MODEL="$2"; shift 2 ;;
    --log-dir) LOG_DIR="$2"; shift 2 ;;
    --flight-tag) FLIGHT_TAG="$2"; shift 2 ;;
    --terminal) TERMINAL="$2"; shift 2 ;;
    --title-prefix) TITLE_PREFIX="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "[launch] Unknown arg: $1"; usage; exit 2 ;;
  esac
done

if [[ -z "$SWARM_SIZE" ]]; then
  SWARM_SIZE="$COUNT"
fi

if [[ -z "$PX4_BUILD_DIR" ]]; then
  PX4_BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
fi

if [[ ! "$COUNT" =~ ^[0-9]+$ ]] || (( COUNT < 1 )); then
  echo "[launch] Invalid --count: $COUNT"
  exit 2
fi
if [[ ! "$BASE_PORT" =~ ^[0-9]+$ ]] || (( BASE_PORT < 1024 )); then
  echo "[launch] Invalid --base-port: $BASE_PORT"
  exit 2
fi
if [[ ! "$SWARM_PORT" =~ ^[0-9]+$ ]] || (( SWARM_PORT < 1 || SWARM_PORT > 65535 )); then
  echo "[launch] Invalid --swarm-port: $SWARM_PORT"
  exit 2
fi

if [[ ! -x "$BIN" ]]; then
  echo "[launch] Binary not found or not executable: $BIN"
  echo "Build first: cmake -S . -B build && cmake --build build -j"
  exit 1
fi

if [[ -n "$MISSION" ]] && [[ ! -f "$MISSION" ]]; then
  echo "[launch] Mission file not found: $MISSION"
  exit 1
fi

sanitize_tag() {
  local s="$1"
  s="${s//[^[:alnum:]]/_}"
  s="${s##_}"
  s="${s%%_}"
  if [[ -z "$s" ]]; then
    s="flight"
  fi
  echo "$s"
}

timestamp_tag_now() {
  date +"%Y%m%d_%H%M%S"
}

if [[ -z "$FLIGHT_TAG" ]]; then
  if [[ -n "$MISSION" ]]; then
    FLIGHT_TAG="$(sanitize_tag "$(basename "${MISSION%.*}")")_$(timestamp_tag_now)"
  else
    FLIGHT_TAG="manual_$(timestamp_tag_now)"
  fi
fi

mkdir -p "$LOG_DIR"
printf '%s\n' "$FLIGHT_TAG" >"$LOG_DIR/.active_flight"

choose_terminal() {
  if [[ "$TERMINAL" != "auto" ]]; then
    echo "$TERMINAL"
    return
  fi
  if command -v gnome-terminal >/dev/null 2>&1; then echo "gnome"; return; fi
  if command -v konsole >/dev/null 2>&1; then echo "konsole"; return; fi
  if command -v xterm >/dev/null 2>&1; then echo "xterm"; return; fi
  echo ""
}

TERM_KIND="$(choose_terminal)"
if [[ -z "$TERM_KIND" ]]; then
  echo "[launch] No supported terminal emulator found (gnome-terminal/konsole/xterm)."
  exit 1
fi

build_cmd() {
  local out=""
  local arg q
  for arg in "$@"; do
    printf -v q "%q" "$arg"
    out+="$q "
  done
  echo "${out% }"
}

launch_terminal() {
  local title="$1"
  shift
  local cmd
  cmd="$(build_cmd "$@")"

  case "$TERM_KIND" in
    gnome)
      gnome-terminal --title "$title" -- bash -lc "$cmd; echo; echo '[terminal] Process finished.'; exec bash"
      ;;
    konsole)
      konsole --new-tab -p tabtitle="$title" -e bash -lc "$cmd; echo; echo '[terminal] Process finished.'; exec bash"
      ;;
    xterm)
      xterm -T "$title" -e bash -lc "$cmd; echo; echo '[terminal] Process finished.'; exec bash" &
      ;;
    *)
      echo "[launch] Unsupported terminal kind: $TERM_KIND"
      return 1
      ;;
  esac
}

start_px4_gz_instances() {
  local px4_bin="$PX4_BUILD_DIR/bin/px4"
  local px4_etc="$PX4_BUILD_DIR/etc"
  local instance_dir
  local model_name

  if [[ ! -x "$px4_bin" ]]; then
    echo "[launch] PX4 binary missing or not executable: $px4_bin"
    exit 1
  fi

  if [[ ! -d "$px4_etc" ]]; then
    echo "[launch] PX4 etc dir missing: $px4_etc"
    exit 1
  fi

  echo "[launch] Restarting PX4 SITL instances for gz: count=$COUNT world=$GZ_WORLD model=$GZ_MODEL"
  pkill -x px4 >/dev/null 2>&1 || true
  pkill -f px4-simulator_mavlink >/dev/null 2>&1 || true
  sleep 1

  for ((i=0; i<COUNT; i++)); do
    instance_dir="$PX4_BUILD_DIR/instance_$i"
    model_name="${GZ_MODEL}_${i}"
    mkdir -p "$instance_dir"
    pushd "$instance_dir" >/dev/null
    echo "[launch] Starting PX4 instance $i attached to ${model_name}"
    PX4_SYS_AUTOSTART=4001 \
    PX4_SIMULATOR=gz \
    PX4_GZ_STANDALONE=1 \
    PX4_GZ_WORLD="$GZ_WORLD" \
    PX4_GZ_MODEL_NAME="$model_name" \
    "$px4_bin" -i "$i" -d "$px4_etc" >out.log 2>err.log &
    popd >/dev/null
    sleep 1
  done
}

if (( START_PX4 == 1 )); then
  start_px4_gz_instances
fi

echo "[launch] Starting PX-Swarm nodes in separate terminals..."
echo "[launch] terminal=$TERM_KIND count=$COUNT base_port=$BASE_PORT swarm=$SWARM_IP:$SWARM_PORT"
echo "[launch] flight_tag=$FLIGHT_TAG log_dir=$LOG_DIR"

for ((i=1; i<COUNT; i++)); do
  port=$((BASE_PORT + i))
  title="$TITLE_PREFIX follower#$i :$port"
  cmd=( "$BIN" --port "$port" --role follower --id "$i" --swarm-ip "$SWARM_IP" --swarm-port "$SWARM_PORT" --swarm-size "$SWARM_SIZE" --log-dir "$LOG_DIR" --flight-tag "$FLIGHT_TAG" )
  launch_terminal "$title" "${cmd[@]}"
  sleep 0.15
done

leader_port="$BASE_PORT"
leader_title="$TITLE_PREFIX leader#0 :$leader_port"
leader_cmd=( "$BIN" --port "$leader_port" --role leader --id 0 --swarm-ip "$SWARM_IP" --swarm-port "$SWARM_PORT" --swarm-size "$SWARM_SIZE" --log-dir "$LOG_DIR" --flight-tag "$FLIGHT_TAG" )
if [[ -n "$MISSION" ]]; then
  leader_cmd+=( --mission "$MISSION" )
fi
if [[ -n "$LEADER_MAX_SPEED" ]]; then
  leader_cmd+=( --mission-leader-max-speed "$LEADER_MAX_SPEED" )
fi
launch_terminal "$leader_title" "${leader_cmd[@]}"

echo "[launch] Done. Open terminals should now be running all nodes."
echo "[launch] Stop nodes by closing terminals (or Ctrl+C in each)."
