#!/usr/bin/env bash
# Robust 15-drone PX4 SITL + Gazebo launcher.
# Keep this file anywhere (e.g. ~/Downloads). It finds PX4 explicitly.

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_ROOT="${PX4_ROOT:-$HOME/PX4-Autopilot}"
PX4_BIN="${PX4_BIN:-$PX4_ROOT/build/px4_sitl_default/bin/px4}"
CONTROLLER="${FORMATION_CONTROLLER:-$SCRIPT_DIR/x500_l_formation_controller_v15.py}"
WORLD="${PX4_GZ_WORLD:-default}"
STARTUP_DELAY="${STARTUP_DELAY:-4}"
POST_SPAWN_WAIT="${POST_SPAWN_WAIT:-40}"
RUN_DIR="${X500_SWARM_LOG_DIR:-$SCRIPT_DIR/x500_swarm_logs_v5}"

mkdir -p "$RUN_DIR"
PIDS=()

cleanup() {
    echo
    echo "Stopping 15 PX4 instances..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    echo "Swarm stopped. Logs: $RUN_DIR"
}
trap cleanup INT TERM EXIT

if [[ ! -d "$PX4_ROOT" ]]; then
    echo "ERROR: PX4 root not found: $PX4_ROOT"
    echo "Set PX4_ROOT if your PX4-Autopilot directory is elsewhere."
    exit 1
fi

if [[ ! -x "$PX4_BIN" ]]; then
    echo "ERROR: PX4 binary not found: $PX4_BIN"
    echo "This launcher does NOT run 'make px4_sitl'."
    echo "Build/check PX4 from: $PX4_ROOT"
    exit 1
fi

if [[ ! -f "$CONTROLLER" ]]; then
    echo "ERROR: Controller not found: $CONTROLLER"
    exit 1
fi

if ! python3 -c 'import pymavlink' >/dev/null 2>&1; then
    echo "ERROR: pymavlink is not installed for python3."
    echo "Run: python3 -m pip install pymavlink"
    exit 1
fi

# Do NOT use 'set -u' here. PX4's generated gz_env.sh may reference variables
# that are not exported after a reboot. That was the source of the previous
# 'unbound variable' failure.
GZ_ENV="$PX4_ROOT/build/px4_sitl_default/rootfs/gz_env.sh"
if [[ -f "$GZ_ENV" ]]; then
    # shellcheck disable=SC1090
    source "$GZ_ENV"
else
    echo "WARNING: $GZ_ENV not found; continuing with current Gazebo environment."
fi

# Fixed initial ground poses. Formation is commanded later by Python.
declare -A POSE
POSE[1]="-6.00,-4.50,0.00,0,0,0.00"
POSE[2]="-6.00,-1.50,0.00,0,0,0.00"
POSE[3]="-6.00,1.50,0.00,0,0,0.00"
POSE[4]="-6.00,4.50,0.00,0,0,0.00"
POSE[5]="-9.00,-4.50,0.00,0,0,0.00"
POSE[6]="-9.00,-1.50,0.00,0,0,0.00"
POSE[7]="-9.00,1.50,0.00,0,0,0.00"
POSE[8]="-9.00,4.50,0.00,0,0,0.00"
POSE[9]="-12.00,-4.50,0.00,0,0,0.00"
POSE[10]="-12.00,-1.50,0.00,0,0,0.00"
POSE[11]="-12.00,1.50,0.00,0,0,0.00"
POSE[12]="-3.00,-4.50,0.00,0,0,0.00"
POSE[13]="-3.00,1.50,0.00,0,0,0.00"
POSE[14]="-3.00,-1.50,0.00,0,0,0.00"
POSE[15]="-3.00,4.50,0.00,0,0,0.00"

echo "============================================================"
echo "PX4 / Gazebo x500 SWARM"
echo "Exactly 15 vehicles: instances 0..14 / UIDs 1..15"
echo "Model: gz_x500"
echo "World: $WORLD"
echo "PX4 root: $PX4_ROOT"
echo "PX4 binary: $PX4_BIN"
echo "Controller: $CONTROLLER"
echo "============================================================"

echo "Starting UID 1 / PX4 instance 0 (starts Gazebo)..."
(
    export PX4_SYS_AUTOSTART=4001
    export PX4_SIM_MODEL=gz_x500
    export PX4_GZ_WORLD="$WORLD"
    export PX4_GZ_MODEL_POSE="${POSE[1]}"
    exec "$PX4_BIN" -i 0
) >"$RUN_DIR/px4_uid_1.log" 2>&1 &
PIDS+=("$!")

sleep 10

for uid in $(seq 2 15); do
    instance=$((uid - 1))
    echo "Starting UID $uid / PX4 instance $instance..."
    (
        export PX4_SYS_AUTOSTART=4001
        export PX4_SIM_MODEL=gz_x500
        export PX4_GZ_WORLD="$WORLD"
        export PX4_GZ_STANDALONE=1
        export PX4_GZ_MODEL_POSE="${POSE[$uid]}"
        exec "$PX4_BIN" -i "$instance"
    ) >"$RUN_DIR/px4_uid_${uid}.log" 2>&1 &
    PIDS+=("$!")
    sleep "$STARTUP_DELAY"
done

echo
echo "All 15 PX4 instances launched."
echo "Waiting ${POST_SPAWN_WAIT}s for Gazebo bridges/sensors..."
sleep "$POST_SPAWN_WAIT"

echo
echo "Launching 15-drone controller..."
python3 "$CONTROLLER" "$@"
STATUS=$?
echo "Controller exited with status $STATUS."
exit "$STATUS"
