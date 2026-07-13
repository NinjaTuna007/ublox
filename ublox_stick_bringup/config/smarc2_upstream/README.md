# Upstream smarc2 PR reference copies

This directory mirrors proposed changes for the smarc2 repository:

- Trimmed `waraps_level1.yaml` (Level-1 topics only; no exec/tst/bt)
- Optional `waraps_vehicle.py` change to include `sound_velocity` in `sensor_info`

Apply these changes in `str_json_mqtt_bridge` and `wasp_bt` on smarc2, then point
`stick_bringup.sh` at the installed smarc2 config if preferred over the local copy
in `ublox_stick_bringup/config/waraps_level1.yaml`.
