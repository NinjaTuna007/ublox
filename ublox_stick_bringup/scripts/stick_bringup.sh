#! /bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_SHARE="$(ros2 pkg prefix ublox_stick_bringup)/share/ublox_stick_bringup"
SOURCE_ENV="${SCRIPT_DIR}/../config/rtk_credentials.env"
if [ -n "${RTK_CREDENTIALS_FILE}" ]; then
    ENV_FILE="${RTK_CREDENTIALS_FILE}"
elif [ -f "${SOURCE_ENV}" ]; then
    ENV_FILE="${SOURCE_ENV}"
else
    ENV_FILE="${PKG_SHARE}/config/rtk_credentials.env"
fi

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
FOXGLOVE_PORT="${FOXGLOVE_PORT:-8765}"
GPS_BACKEND="${GPS_BACKEND:-serial_ubx}"
SERIAL_BAUD="${SERIAL_BAUD:-115200}"
ENABLE_NTRIP="${ENABLE_NTRIP:-true}"
DEVICE_SERIAL_STRING="${DEVICE_SERIAL_STRING:-}"
WARAPS_CONFIG="${WARAPS_CONFIG:-${PKG_SHARE}/config/waraps_level1.yaml}"

# OWTT acoustic setup: stick N <-> modem id 00N.
# Sticks 1-2 are broadcasters (owtt_leader), sticks 3-4 are receivers (owtt_follower).
ENABLE_OWTT="${ENABLE_OWTT:-true}"
MODEM_ID="$(printf '%03d' "$STICK_NUMBER")"
OWTT_ROLE="${OWTT_ROLE:-auto}"
# TDMA schedule: 001 free-runs every 4 PPS epochs (~4 s); 002 broadcasts 2
# epochs after each heard 001 frame, stacking one acoustic frame every 2 s.
if [ -z "${BROADCAST_INTERVAL_S:-}" ]; then
    if [ "$MODEM_ID" = "002" ]; then
        BROADCAST_INTERVAL_S=2
    else
        BROADCAST_INTERVAL_S=4
    fi
fi
OWTT_WORLD_FRAME="${OWTT_WORLD_FRAME:-utm}"
# How often the leader pushes fresh $G to the Teensy. Synced to the 40 Hz TF
# publisher by default; set 0.02 for 50 Hz.
GPS_SEND_PERIOD_S="${GPS_SEND_PERIOD_S:-0.025}"
# Field-tunable OWTT freshness gates (leaders / transmitters):
#   PAYLOAD_TTL_EPOCHS — Teensy stops acoustic TX if no new $G for N PPS epochs
#                        (default 1 ≈ 1 s). 0 = never expire (bench only).
#   MAX_POSITION_AGE_S — ROS leader stops pushing $G if /smarc/latlon (or
#                        latlon_topic) has been silent this long (default 5 s).
PAYLOAD_TTL_EPOCHS="${PAYLOAD_TTL_EPOCHS:-1}"
MAX_POSITION_AGE_S="${MAX_POSITION_AGE_S:-5.0}"
LEADER1_NAME="${LEADER1_NAME:-stick_1}"
LEADER1_MODEM_ID="${LEADER1_MODEM_ID:-001}"
LEADER2_NAME="${LEADER2_NAME:-stick_2}"
LEADER2_MODEM_ID="${LEADER2_MODEM_ID:-002}"
# TDMA: broadcaster 001 goes first, 002 waits for 001's epoch.
LISTEN_FOR_MODEM_ID="${LISTEN_FOR_MODEM_ID:-$([ "$MODEM_ID" = "002" ] && echo "001" || echo "000")}"
# OCXO holdover experiment: seconds since Teensy boot after which it ignores
# PPS and free-runs on the OCXO. Stick 4 defaults to 10 minutes; others never.
IGNORE_PPS_AFTER_S="${IGNORE_PPS_AFTER_S:-$([ "$STICK_NUMBER" = "4" ] && echo 600 || echo 0)}"
# Sound velocity sensor (Valeport ultraSV, pvptu_driver) lives on ONE stick —
# stick 3 by default. It publishes Float64 on /<robot>/smarc/sound_velocity;
# followers on the same computer read it across the shared ROS graph, so
# SVS_SOURCE_ROBOT tells each follower where the sensor lives.
HAS_SVS_SENSOR="${HAS_SVS_SENSOR:-$([ "$STICK_NUMBER" = "3" ] && echo true || echo false)}"
SVS_SOURCE_ROBOT="${SVS_SOURCE_ROBOT:-stick_3}"

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
    1|2|3|4)
        NTRIP_USER_VAR="NTRIP_USERNAME_${STICK_NUMBER}"
        NTRIP_PASS_VAR="NTRIP_PASSWORD_${STICK_NUMBER}"
        NTRIP_USERNAME="${!NTRIP_USER_VAR:?${NTRIP_USER_VAR} not set in $ENV_FILE}"
        NTRIP_PASSWORD="${!NTRIP_PASS_VAR:?${NTRIP_PASS_VAR} not set in $ENV_FILE}"
        ;;
    *)
        echo "ERROR: Invalid STICK_NUMBER '$STICK_NUMBER'. Must be 1, 2, 3 or 4."
        exit 1
        ;;
esac

# --- Teensy port resolution --------------------------------------------------
# Each Teensy box has a unique USB serial, and /dev/serial/by-id symlinks
# survive ACM renumbering. With two boxes on one computer, pin each stick to
# its box via TEENSY_USB_SERIAL_<stick> in rtk_credentials.env (or export
# TEENSY_USB_SERIAL). if00 = LoLo/succorfish, if02 = GNSS bridge.
TEENSY_SERIAL_VAR="TEENSY_USB_SERIAL_${STICK_NUMBER}"
TEENSY_USB_SERIAL="${TEENSY_USB_SERIAL:-${!TEENSY_SERIAL_VAR:-}}"
# Escape hatch for box swaps: TEENSY_USB_SERIAL=any ignores the per-stick pin
# and matches whatever single box is plugged in (still refuses to guess when
# more than one box is present).
if [ "$TEENSY_USB_SERIAL" = "any" ]; then
    TEENSY_USB_SERIAL=""
fi

resolve_teensy_port() {
    local iface="$1" fallback="$2" pattern
    if [ -n "$TEENSY_USB_SERIAL" ]; then
        pattern="/dev/serial/by-id/usb-Teensyduino_Dual_Serial_${TEENSY_USB_SERIAL}-${iface}"
    else
        pattern="/dev/serial/by-id/usb-Teensyduino_Dual_Serial_*-${iface}"
    fi
    local matches count
    matches=$(compgen -G "$pattern" || true)
    count=$(printf '%s\n' "$matches" | grep -c . || true)
    if [ "$count" -eq 0 ]; then
        if [ -n "$TEENSY_USB_SERIAL" ]; then
            echo "ERROR: pinned Teensy usb serial '$TEENSY_USB_SERIAL' not found: $pattern" >&2
            echo "Is the box for stick $STICK_NUMBER plugged in?" >&2
            exit 1
        fi
        printf '%s\n' "$fallback"
        return
    fi
    if [ "$count" -gt 1 ]; then
        echo "ERROR: $count Teensy boxes detected but TEENSY_USB_SERIAL is not set." >&2
        echo "Add TEENSY_USB_SERIAL_${STICK_NUMBER}=<usb serial> to $ENV_FILE. Candidates:" >&2
        printf '%s\n' "$matches" >&2
        exit 1
    fi
    printf '%s\n' "$matches"
}

LOLO_PORT="${LOLO_PORT:-$(resolve_teensy_port if00 /dev/ttyACM0)}"
SERIAL_PORT="${SERIAL_PORT:-$(resolve_teensy_port if02 /dev/ttyACM1)}"

if [ "$USE_SIM_TIME" = "True" ]; then
    REALSIM=simulation
else
    REALSIM=real
fi

tmux -2 new-session -d -s "$SESSION" -n 'gps'
tmux select-window -t "$SESSION:0"
GPS_LAUNCH_ARGS="robot_name:=$ROBOT_NAME gps_backend:=$GPS_BACKEND serial_port:=$SERIAL_PORT serial_baud:=$SERIAL_BAUD enable_ntrip:=$ENABLE_NTRIP"
if [ -n "$DEVICE_SERIAL_STRING" ]; then
    GPS_LAUNCH_ARGS="$GPS_LAUNCH_ARGS device_serial_string:=$DEVICE_SERIAL_STRING"
fi
GPS_LAUNCH_ARGS="$GPS_LAUNCH_ARGS username:=$NTRIP_USERNAME password:=$NTRIP_PASSWORD host:=$NTRIP_HOST port:=$NTRIP_PORT mountpoint:=$NTRIP_MOUNTPOINT"
tmux send-keys "ros2 launch ublox_stick_bringup stick_gps_stack.launch.py $GPS_LAUNCH_ARGS" C-m

tmux new-window -t "$SESSION:1" -n 'foxglove'
tmux select-window -t "$SESSION:1"
tmux send-keys "ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=$FOXGLOVE_PORT" C-m

tmux new-window -t "$SESSION:2" -n 'waraps'
tmux select-window -t "$SESSION:2"
tmux send-keys "ros2 launch wasp_bt wasp_mqtt_agent.launch robot_name:=$ROBOT_NAME agent_type:=$AGENT_TYPE pulse_rate:=$PULSE_RATE use_sim_time:=$USE_SIM_TIME" C-m

tmux new-window -t "$SESSION:3" -n 'mqtt'
tmux select-window -t "$SESSION:3"
tmux send-keys "ros2 launch str_json_mqtt_bridge waraps_bridge.launch broker_addr:=$BROKER_ADDR broker_port:=$BROKER_PORT robot_name:=$ROBOT_NAME domain:=$DOMAIN realsim:=$REALSIM use_sim_time:=$USE_SIM_TIME context:=$CONTEXT mqtt_params_file:=$WARAPS_CONFIG" C-m

if [ "$ENABLE_OWTT" = "true" ]; then
    if [ "$OWTT_ROLE" = "auto" ]; then
        if [ "$STICK_NUMBER" = "1" ] || [ "$STICK_NUMBER" = "2" ]; then
            OWTT_ROLE=leader
        else
            OWTT_ROLE=follower
        fi
    fi
    # Namespace the driver under /<robot>. OWTT launch files default their own
    # namespace to robot_name, so they share /<robot>/succorfish/* automatically.
    SUCCORFISH_CMD="ros2 launch succorfish_driver succorfish_driver.launch profile:=teensy serial_port:=$LOLO_PORT namespace:=$ROBOT_NAME"
    if [ "$OWTT_ROLE" = "leader" ]; then
        SERIAL_PING_CMD="ros2 launch serial_ping_pkg owtt_leader_node.launch own_modem_id:=$MODEM_ID listen_for_modem_id:=$LISTEN_FOR_MODEM_ID broadcast_interval_s:=$BROADCAST_INTERVAL_S robot_name:=$ROBOT_NAME world_frame:=$OWTT_WORLD_FRAME send_period_s:=$GPS_SEND_PERIOD_S payload_ttl_epochs:=$PAYLOAD_TTL_EPOCHS max_position_age_s:=$MAX_POSITION_AGE_S"
    else
        SERIAL_PING_CMD="ros2 launch serial_ping_pkg owtt_follower_node.launch.py own_modem_id:=$MODEM_ID leader1_name:=$LEADER1_NAME leader1_modem_id:=$LEADER1_MODEM_ID leader2_name:=$LEADER2_NAME leader2_modem_id:=$LEADER2_MODEM_ID ignore_pps_after_s:=$IGNORE_PPS_AFTER_S robot_name:=$ROBOT_NAME sound_velocity_topic:=/$SVS_SOURCE_ROBOT/smarc/sound_velocity"
    fi
fi

if [ "$HAS_SVS_SENSOR" = "true" ]; then
    tmux new-window -t "$SESSION:4" -n 'pvptu'
    tmux select-window -t "$SESSION:4"
    tmux send-keys "ros2 launch pvptu_driver pvptu_driver.launch.py robot_name:=$ROBOT_NAME topic:=smarc/sound_velocity" C-m

    tmux new-window -t "$SESSION:5" -n 'sound_vel'
    tmux select-window -t "$SESSION:5"
    tmux send-keys "ros2 launch ublox_stick_bringup sound_velocity_relay.launch.py robot_name:=$ROBOT_NAME input_topic:=smarc/sound_velocity" C-m

    tmux new-window -t "$SESSION:6" -n 'succorfish'
    tmux new-window -t "$SESSION:7" -n 'serial_ping'
    tmux new-window -t "$SESSION:8" -n 'spare'
    SUCCORFISH_WIN=6
    SERIAL_PING_WIN=7
else
    tmux new-window -t "$SESSION:4" -n 'succorfish'
    tmux new-window -t "$SESSION:5" -n 'serial_ping'
    tmux new-window -t "$SESSION:6" -n 'spare'
    SUCCORFISH_WIN=4
    SERIAL_PING_WIN=5
fi

if [ "$ENABLE_OWTT" = "true" ]; then
    tmux select-window -t "$SESSION:$SUCCORFISH_WIN"
    tmux send-keys "$SUCCORFISH_CMD" C-m
    tmux select-window -t "$SESSION:$SERIAL_PING_WIN"
    tmux send-keys "$SERIAL_PING_CMD" C-m
fi

tmux select-window -t "$SESSION:0"
if [ "${SKIP_ATTACH:-0}" != "1" ]; then
    tmux -2 attach-session -t "$SESSION"
fi
