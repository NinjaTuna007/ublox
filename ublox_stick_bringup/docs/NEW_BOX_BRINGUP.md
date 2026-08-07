# New Stick box bring-up drill

Checklist for a **brand-new Stick PCB** (Teensy 4.1 Dual Serial + u-blox ZED-X20P [+ Succorfish modem]).  
Use this for humans and AI agents. Do the steps in order; stop and fix before advancing.

Known-good Dual Serial map (after firmware is flashed):

| USB interface | by-id suffix | Role |
|---|---|---|
| `if00` | `…-if00` → usually `/dev/ttyACM0` | LoLo / host protocol @ 115200 |
| `if02` | `…-if02` → usually `/dev/ttyACM1` | Transparent GNSS bridge (UBX/NMEA/RTCM) |

Firmware repo: `~/Arduino/succor-sketches/owtt_stick_v1`  
ROS bringup: `ros2 run ublox_stick_bringup stick_bringup.sh <N>`

---

## 0. Preconditions

- [ ] Operator in `dialout` (`groups | grep dialout`)
- [ ] Workspace built / sourced: `source ~/colcon_ws/install/setup.bash`
- [ ] `rtk_credentials.env` exists (NTRIP users/passwords). **Do not commit it.**
- [ ] Latest `owtt_stick_v1` sketch available (GNSS opens at **38400**, RAM-bumps to **921600**)
- [ ] Prefer **one** Stick box on USB while flashing / first probe (avoids ACM mix-ups)

**Sandbox note (AI agents):** never conclude a `/dev/ttyACM*` or `by-id` path is missing from a sandboxed `ls`. Re-check with full host permissions (`required_permissions: ["all"]`).

---

## 1. Enumerate USB / serial

```bash
lsusb | grep -Ei '16c0|1546|Teensy|u-blox'
ls -la /dev/serial/by-id/
ls -la /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
fuser -v /dev/ttyACM* 2>&1 || true
journalctl -k -n 40 --no-pager | grep -Ei 'ttyACM|16c0|1546|teensy|cdc_acm' | tail -20
```

**Pass criteria**

- [ ] Exactly one Dual Serial Teensy when doing a single-box drill: `16c0:048b`
- [ ] Two by-id links: `usb-Teensyduino_Dual_Serial_<SERIAL>-if00` and `…-if02`
- [ ] Record the Teensy USB **serial string** (e.g. `18445230`) — needed for multi-box pins
- [ ] Ports not held by a stale ROS/tmux process (`fuser` empty), or kill that session first

If `16c0:0478` (HalfKay) appears mid-flash, wait for Dual Serial to return.

---

## 2. Stop conflicting software

```bash
tmux list-sessions 2>/dev/null
# If an old bringup owns this Teensy:
tmux kill-session -t stick_N_bringup   # or: tmux kill-server
```

- [ ] No tmux/ROS node holding `if00` / `if02`

---

## 3. Flash Teensy firmware (`owtt_stick_v1`)

```bash
cd ~/Arduino/succor-sketches
arduino-cli compile --fqbn teensy:avr:teensy41:usb=serial2 owtt_stick_v1

# HalfKay reboot (replace SERIAL)
python3 - <<'PY'
import serial, time
p='/dev/serial/by-id/usb-Teensyduino_Dual_Serial_SERIAL-if00'
s=serial.Serial(p,134); time.sleep(0.3); s.close()
time.sleep(2)
PY

arduino-cli board list   # note the usb… teensy upload path
cd owtt_stick_v1
arduino-cli upload --fqbn teensy:avr:teensy41:usb=serial2 --port <UPLOAD_PORT> .

ls /dev/serial/by-id/ | grep Teensy
lsusb -d 16c0:0478 || echo "not left in bootloader"
```

**Pass criteria**

- [ ] Compile succeeds
- [ ] Upload succeeds; Dual Serial returns with the **same** USB serial
- [ ] Not stuck in HalfKay (`16c0:0478`)

---

## 4. Probe LoLo + GNSS (no ROS yet)

```bash
# LoLo: enable GNSS status (replace SERIAL)
python3 - <<'PY'
import serial, time
LOLO='/dev/serial/by-id/usb-Teensyduino_Dual_Serial_SERIAL-if00'
s=serial.Serial(LOLO,115200,timeout=0.05)
s.write(b'$ZGNSSDEBUG=1\r\n')
t0=time.time()
while time.time()-t0 < 12:
    line=s.readline()
    if not line: continue
    t=line.decode('ascii','replace').strip()
    if t.startswith('#GNSS'):
        print(t)
s.write(b'$ZGNSSDEBUG=0\r\n'); s.close()
PY
```

**Pass criteria (GNSS UART / bridge)**

- [ ] `#GNSS,ACTIVE` with climbing `rx_bytes`
- [ ] `uart_baud=921600` (after the 38400→921600 bump) — or transient `#GNSS,RELINK,…` then success
- [ ] `ubx_ok` and/or `nmea` increasing; prefer `ubx_bad` staying near 0
- [ ] Bridge dump is **not** all `0x00` (all-nulls ⇒ wiring / wrong baud / stuck TX)

**PPS / UTC (may need outdoor sky or warm-up)**

- [ ] `timing=PPS_LOCKED` (or briefly `WAIT_PPS` while warming)
- [ ] `utc_valid=1` and a sensible `epoch_unix_s` once PPS+TIMEUTC bind

Indoor `WAIT_PPS` / `utc_valid=0` with healthy `ubx_ok`/`nmea` is **not** a flash failure.

Optional: open `…-if02` and confirm non-null traffic / UBX `B5 62` / `$G…` NMEA.

---

## 5. Pin the box (only if ≥2 Teensies on one PC)

Edit `ublox_stick_bringup/config/rtk_credentials.env` (gitignored):

```bash
TEENSY_USB_SERIAL_<N>=<SERIAL>    # e.g. TEENSY_USB_SERIAL_1=18445230
```

Or one-shot:

```bash
TEENSY_USB_SERIAL=<SERIAL> ros2 run ublox_stick_bringup stick_bringup.sh <N>
```

Single box + unset pin → bringup wildcards the only Dual Serial present.

---

## 6. ROS full-stack bringup

```bash
source ~/colcon_ws/install/setup.bash
SKIP_ATTACH=1 ros2 run ublox_stick_bringup stick_bringup.sh <N>
tmux list-windows -t stick_<N>_bringup
```

Stick roles (default):

| Stick | OWTT role | Modem id |
|------:|-----------|----------|
| 1–2 | leader (TX) | `001` / `002` |
| 3–4 | follower (RX) | `003` / `004` |

**Pass criteria**

- [ ] `/stick_<N>/ublox_gps_node/fix` publishing (fix status ≥0; outdoor RTK may take time)
- [ ] `succorfish` window open on LoLo; config commands visible
- [ ] Leader: eventually `#Y,OK,<id>` then `$G` / acoustic TX — **not** endless `#E,Y,ADDR_TIMEOUT`
- [ ] Follower (when TX exists): `#I…` and `/stick_<leader>/distance` ~physical range

`#E,Y,ADDR_TIMEOUT` = Teensy↔Succorfish modem path (power / UART / modem), not GNSS.

---

## 7. Two-stick OWTT sanity (optional)

1. Bring up TX stick (1 or 2) and RX stick (3 or 4) on correct boxes.
2. Confirm both `PPS_LOCKED` / `utc_valid=1`.
3. Confirm RX ranges stable (order of metres on the bench), no whole-second outliers.
4. Restart follower once: ranges must keep flowing (e-ink refresh must **not** freeze the timing loop — current firmware paints on a TeensyThreads worker).

---

## Failure quick-reference

| Symptom | Likely cause | Next check |
|---|---|---|
| No `by-id` Teensy links | Cable / power / not Dual Serial FW | `lsusb`; reflash Dual Serial FQBN |
| LoLo silent | Wrong port / held by ROS | `fuser`; use `…-if00` @ 115200 |
| `#GNSS` all zeros / `ubx_ok=0` forever | X20P UART wiring or baud | Probe wiring; watch `#GNSS,RELINK`; confirm INITIAL=38400 sketch |
| `WAIT_PPS` forever outdoors | PPS pin / antenna | Hardware PPS to Teensy pin 40; sky view |
| `ADDR_TIMEOUT` on `$Y…` | Modem path | Modem power, Teensy↔modem UART, baud 9600 |
| Ranges off by ~330 m / ±1 s | UTC label slip | Both sticks PPS+UTC bind; see `#UTC,BOUND/REBIND` |
| Topic silence during e-ink paint | Old firmware blocking `display()` | Flash sketch with TeensyThreads e-ink worker |

---

## Agent summary (copy-paste)

1. Enumerate Dual Serial `by-id`; note USB serial; free ports.  
2. Kill conflicting tmux.  
3. Compile + HalfKay + upload `owtt_stick_v1` (`usb=serial2`).  
4. `$ZGNSSDEBUG=1` → expect `uart_baud=921600`, rising `ubx_ok`/`nmea`.  
5. Wait for `PPS_LOCKED` + `utc_valid=1` before OWTT claims.  
6. Pin serial if multi-box; `SKIP_ATTACH=1 stick_bringup.sh <N>`.  
7. Verify fix topic; then modem `#Y,OK` / ranging as applicable.
