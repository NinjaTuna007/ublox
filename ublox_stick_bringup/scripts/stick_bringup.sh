#! /bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_SHARE="$(ros2 pkg prefix ublox_stick_bringup)/share/ublox_stick_bringup"
ENV_FILE="${RTK_CREDENTIALS_FILE:-${PKG_SHARE}/config/rtk_credentials.env}"

STICK_NUMBER="${1:-${STICK_NUMBER:-1}}"
ROBOT_NAME=stick_${STICK_NUMBER}
SESSION=${ROBOT_NAME}_bringup
USE_SIM_TIME=False

AGENT_TYPE=surface
PULSE_RATE=1.0
CONTEXT=waraps
DOMAIN=surface
BROKER_ADDR="${MQTT_BROKER_ADDR:-20.240.40.232}"
BROKER_PORT="${MQTT_BROKER_PORT:-1884}"
WARAPS_CONFIG="${WARAPS_CONFIG:-${PKG_SHARE}/config/waraps_level1.yaml}"

if [ ! -f "$ENV_FILE" ]; then
    echo "ERROR: RTK credentials file not found: $ENV_FILE"
    echo "Copy config/rtk_credentials.env.example to config/rtk_credentials.env and fill in credentials."
    exit 1
fi
set -a
# shellcheck source=/dev/null
source "$ENV_FILE"
set +a

NTRIP_HOST="${NTRIP_HOST:-nrtk-swepos.lm.se}"
NTRIP_PORT="${NTRIP_PORT:-80}"
NTRIP_MOUNTPOINT="${NTRIP_MOUNTPOINT:-MSM_GNSS}"

case "$STICK_NUMBER" in
    1)
        NTRIP_USERNAME="${NTRIP_USERNAME_1:?NTRIP_USERNAME_1 not set in $ENV_FILE}"
        NTRIP_PASSWORD="${NTRIP_PASSWORD_1:?NTRIP_PASSWORD_1 not set in $ENV_FILE}"
        ;;
    2)
        NTRIP_USERNAME="${NTRIP_USERNAME_2:?NTRIP_USERNAME_2 not set in $ENV_FILE}"
        NTRIP_PASSWORD="${NTRIP_PASSWORD_2:?NTRIP_PASSWORD_2 not set in $ENV_FILE}"
        ;;
    *)
        echo "ERROR: Invalid STICK_NUMBER '$STICK_NUMBER'. Must be 1 or 2."
        exit 1
        ;;
esac

if [ "$USE_SIM_TIME" = "True" ]; then
    REALSIM=simulation
else
    REALSIM=real
fi

tmux -2 new-session -d -s "$SESSION" -n 'gps'
tmux select-window -t "$SESSION:0"
tmux send-keys "ros2 launch ublox_stick_bringup stick_gps_stack.launch.py robot_name:=$ROBOT_NAME frame_id:=${ROBOT_NAME}_gps username:=$NTRIP_USERNAME password:=$NTRIP_PASSWORD host:=$NTRIP_HOST port:=$NTRIP_PORT mountpoint:=$NTRIP_MOUNTPOINT" C-m

tmux new-window -t "$SESSION:1" -n 'waraps'
tmux select-window -t "$SESSION:1"
tmux send-keys "ros2 launch wasp_bt wasp_mqtt_agent.launch robot_name:=$ROBOT_NAME agent_type:=$AGENT_TYPE pulse_rate:=$PULSE_RATE use_sim_time:=$USE_SIM_TIME" C-m

tmux new-window -t "$SESSION:2" -n 'mqtt'
tmux select-window -t "$SESSION:2"
tmux send-keys "ros2 launch str_json_mqtt_bridge waraps_bridge.launch broker_addr:=$BROKER_ADDR broker_port:=$BROKER_PORT robot_name:=$ROBOT_NAME domain:=$DOMAIN realsim:=$REALSIM use_sim_time:=$USE_SIM_TIME context:=$CONTEXT mqtt_params_file:=$WARAPS_CONFIG" C-m

if [ "$STICK_NUMBER" = "1" ]; then
    tmux new-window -t "$SESSION:3" -n 'pvptu'
    tmux select-window -t "$SESSION:3"
    tmux send-keys "ros2 launch pvptu_driver pvptu_driver.launch.py robot_name:=$ROBOT_NAME" C-m

    tmux new-window -t "$SESSION:4" -n 'sound_vel'
    tmux select-window -t "$SESSION:4"
    tmux send-keys "ros2 launch ublox_stick_bringup sound_velocity_relay.launch.py robot_name:=$ROBOT_NAME" C-m

    tmux new-window -t "$SESSION:5" -n 'succorfish'
    tmux new-window -t "$SESSION:6" -n 'serial_ping'
    tmux new-window -t "$SESSION:7" -n 'spare'
else
    tmux new-window -t "$SESSION:3" -n 'succorfish'
    tmux new-window -t "$SESSION:4" -n 'serial_ping'
    tmux new-window -t "$SESSION:5" -n 'spare'
fi

tmux select-window -t "$SESSION:0"
if [ "${SKIP_ATTACH:-0}" != "1" ]; then
    tmux -2 attach-session -t "$SESSION"
fi
