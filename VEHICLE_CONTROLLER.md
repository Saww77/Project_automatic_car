# Vehicle controller and MQTT telemetry

## What is implemented

`app_program/vehicle_controller/` coordinates the vehicle. It is the normal application owner of Buddy 2 movement requests. Buddy 3 samples the line and submits a copied latest-value recommendation; Buddy 1 forwards MQTT START/STOP intent instead of writing motor speed. The controller preserves the existing line_decide algorithm and motor PID.

The current states that actually execute are IDLE(0), FOLLOW_LINE(1), and FAULT(5). TURNING(2), AVOIDING(3), RECOVER_LINE(4) are reserved for future mission actions. Barcode decoding is still disabled, the first-junction stop remains enabled, and Buddy 5 remains a hardware-driver template. This does not claim full-course navigation or corrected physical curve tracking.

Controller task: priority 15, core 0 (TP_PRC1), nominal 10 ms sleep, approximately 20 ms with this kernel's guard tick. Motor owner remains priority 14 on core 1. A priority-inheritance mutex protects copied input/state snapshots. START is a pending intent; STOP is a priority latch that cancels START. No backlog of old steering samples is kept. All subsystem objects and the controller mutex are created before application tasks start.

The line task now requests a direct 10 ms cyclic release. Junction detection is armed only while FOLLOW_LINE is running, so idle/startup samples cannot latch a junction stop before START. Previously it requested 1 ms, discarded coalesced wakeups, then counted ten releases, despite CNF_TIMER_PERIOD=10 ms. It now reads once per release; missed deadlines are not treated as ten successful samples. This corrects intended sampling cadence without changing the steering decision function.

While following, line recommendations older than 150 ms, invalid ADC reads, motor faults, or loss of MQTT connectivity stop the mission. Fault recovery or reconnection never automatically resumes it. START requires fresh valid line input, motor readiness and no pending STOP/interlock. Wheel targets renew every 100 ms or when changed, preserving the motor watchdog. Enqueue failures retry; motor faults remain latched.

## Startup policy

- Normal build (`MQTT=0`): autostart once after motor readiness and a valid fresh line recommendation, preserving the former driving workflow. A STOP cancels autostart. Subsequent starts use `vehicle_request_start()`.
- MQTT build: boot IDLE. Wait for Wi-Fi, broker connection, command subscription and the 5-second motor startup delay, then send a binary START. STOP, faults and connection loss require a new explicit START after recovery.
- `VC_AUTO_START` may override the default at compile time. Hardware tests should initially be done with the wheels raised.

A junction publishes a persistent stop recommendation. The existing junction test stays halted until firmware restart. It does not yet consume a barcode and execute a turn.

## MQTT configuration and build

The missing template is now supplied at `config/wifi_secrets.example.h`. Copy it to `config/wifi_secrets.h` only if the latter does not already exist. Set your 2.4 GHz Wi-Fi SSID/password, numeric broker IPv4 address, broker port and unique MQTT_CLIENT_ID. The real header is ignored by Git. Do not commit credentials. This cloud validation reused the ignored header from the supplied project archive without printing its values.

Use Pico SDK 2.2.0 (or the README's supported SDK) and initialise:

```sh
git -C "$PICO_SDK_PATH" submodule update --init lib/tinyusb lib/cyw43-driver lib/lwip
cd build_make
make -j2 WIFI=cyw43 NET=lwip MQTT=1
```

Output: `build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2`.
Normal build: `make -j2`, producing `mtk3pico_smp1_usb_cdc_wifi.uf2`.

Broker transport currently uses plain MQTT TCP on the local network, numeric IPv4, QoS0, with no configured broker username/password or TLS. Use a reachable LAN broker configured accordingly; this implementation does not add cloud-broker authentication. Topic root defaults to `robot/<MQTT_CLIENT_ID>/v2`, overridable by MQTT_TOPIC_ROOT. Give each robot a unique client ID.

## Publishing path and wire format

The Buddy 1 command task publishes snapshots at intervals of at least 200 ms (roughly200–250 ms with RTOS scheduling). Application code uses s1_publish_* or mqtt_bridge_send, which copy bounded messages into the existing bridge. Only the Wi-Fi service calls lwIP/MQTT. Publishers never wait for the broker or block the time-critical motor task. Success means locally queued, not broker-acknowledged. Full queues drop the newest update and increment the controller snapshot's telemetry_drops counter; the Wi-Fi bridge also exposes its drop flags.

All payloads are BINARY, not JSON. Bit0 is the least-significant bit of byte0; multi-byte fields use the existing little-endian bit packing. Telemetry is not retained. `/status` is a retained online/offline heartbeat, emitted every 5 seconds by the network service with its existing reconnect and will handling.

| Topic suffix | Bytes | Contents |
|---|---:|---|
| `/vehicle` |1| bits 0–2 mission state; bits 3–7 S1 fault mask |
| `/motion` |9| Existing S2_STATUS: mode0:2, level2:3, motor faults5:4, L RPM9:9, R RPM18:9, L propulsion PWM27:7, R PWM34:7, L progress41:13, R progress54:13. Notation offset:width. |
| `/line` |1| L/C/R black flags bits 0/1/2; barcode bits 3–5 (0 none;1..4 A..D). Current barcode producer reports0 because decoding is disabled. |
| `/terrain` |2| height bits 0–10 (mm); phase bits11–13 |
| `/obstacle` |6| nearest0:12, width12:11, left clearance23:12, right clearance35:12 (mm) |
| `/command` |1|0x00 STOP,0x01 START; text `START` is invalid |
| `/ack` |1| bit 0 requested command; bit 1 rejection. Positive ACK means accepted for processing, not motor movement/completion. |

Vehicle S1 fault bits: motor1, encoder2, IMU4, no-echo8, Wi-Fi16. Raw S2 motor faults remain separately available in `/motion` (count1, timing2, stall4, timeout8). In the current controller only real motor/link faults are mapped; missing IMU/ultrasonic measurements are not fabricated as valid zero measurements.

Line telemetry is omitted if invalid or older than 150 ms. Terrain and obstacle topics are omitted until real producers submit valid samples, and omitted again after 1000 ms without an update. These topics are currently hooks, not invented sensor drivers. A blocked-obstacle safety interlock remains blocked until its producer explicitly clears it; clearing does not restart motion.

`vehicle_get_snapshot()` also exposes timestamp, stop/fault reason, freshness flags and telemetry drop count locally. These fields are available on the additive `/controller` diagnostics topic; the existing binary-v2 topics and one-byte ACK stay compatible. The monitor timestamps receipt; it does not infer sender-time freshness from the compact payload.

## Buddy 4 / Buddy 5 integration

From the owning sensor task, after checking its measurement validity:

```c
#include "../vehicle_controller/vehicle_controller.h"
/* Units/ranges: height0..2000  mm, terrain phase0..5. */
vehicle_submit_terrain(real_height_ mm, real_terrain_phase);
/* nearest/clearance0..4000  mm, width0..2000 mm. */
vehicle_submit_obstacle(real_nearest_ mm, real_width_ mm,
                        real_left_clearance_ mm, real_right_clearance_ mm,
                        obstacle_blocks_path);
```

Do not enable an unvalidated height estimate: Buddy 4's processor already marks height invalid without odometry. No valid height producer is connected in the normal build, so `/terrain` is intentionally absent. Buddy 5 obstacle detection/bypass must be supplied by that subsystem. TURNING/AVOIDING/RECOVER_LINE are not executable mission states yet.

## Laptop monitor and commands

On a laptop that can reach the same broker:

```sh
python3 -m venv .venv
. .venv/bin/activate
python3 -m pip install 'paho-mqtt>=2,<3'
python3 tools/mqtt_monitor.py --host BROKER_IP --root robot/YOUR_CLIENT_ID/v2
```

Send a one-shot START or STOP while displaying decoded telemetry:

```sh
python3 tools/mqtt_monitor.py --host BROKER_IP --root robot/YOUR_CLIENT_ID/v2 --command start
python3 tools/mqtt_monitor.py --host BROKER_IP --root robot/YOUR_CLIENT_ID/v2 --command stop
```

Replace placeholders with the values in your local configuration. Commands are never retained or automatically replayed after reconnection. The monitor prints decoded JSON on the laptop; the on-wire payload remains binary. Watch `/ack` and `/vehicle`, not just successful publication to `/command`. Close a command monitor before opening another if duplicate console output is confusing; each monitor uses a unique client ID.

## Validation and limits

Executed in this cloud workspace:

```sh
python3 testing/vehicle_controller/test_vehicle.py
python3 testing/vehicle_controller/test_monitor.py
python3 testing/subsystem3/test_line_follow.py
python3 testing/subsystem2/test_motion_core_host.py
```

Host tests compile actual controller, runtime/command routing, telemetry and wire-format code against mocked RTOS/motors/MQTT transport. They cover latest-value steering, STOP priority, START/STOP/START batches, link/fault/stale stops, no automatic resume, obstacle interlocks, clock wrap, queue retry, non-MQTT autostart, packed motion/line/terrain/obstacle payloads, missing/stale data suppression, drop accounting and monitor decoding. Existing motor and line regression tests pass.

Both normal and MQTT-enabled firmware profiles build and link with the pinned SDK dependencies. No live Pico, Wi-Fi association or physical broker delivery was available here; those are unverified. On hardware, subscribe first, check heartbeat/telemetry, START with wheels raised, verify asymmetric curve recommendations survive, STOP, disconnect Wi-Fi and confirm stopping/no automatic restart after reconnection. Full-track navigation and physical curve performance are separate unverified outcomes.

## START-rejection diagnostics update

The old dashboard only displayed `START rejected`; it did not report the controller guard. The new matching dashboard and firmware add `/controller`, a25-byte binary payload with version1. Offsets: version0, mission state1, stop reason2, current START blocker3, readiness flags4, motor faults5, left/centre/right darkness6/8/10 (uint16 LE), target RPM12/13, line age14 (uint16 LE;65535 unavailable), uptime16 (uint32 LE), local telemetry drops20 (uint32 LE), last START rejection24.

Blockers:0 ready,1 motor startup,2 network/subscription,3 motor fault,4 STOP pending,5 invalid/missing ADC,6 stale ADC,7 line/junction stop,8 obstacle interlock. Readiness flag bits:0 motor ready,1 valid line,2 line stop,3 obstacle blocked,4 START pending,5 STOP pending,6 network ready. This topic is additive; original topic layouts and ACK bytes are unchanged. Internal MQTT bridge route IDs now need4 bits for9 routes.

A concrete startup bug was corrected: the existing junction test could latch during the five-second motor startup or IDLE and block the first START. Its timer now runs only during FOLLOW_LINE. Once a real running junction stop occurs, the original test remains latched until restart. Do not bypass motor faults or invalid/stale line guards.

The dashboard shows a specific START blocker, normalized IR darkness, target RPM, line age and local drop count. It disables START on a fresh nonzero blocker and keeps STOP available. If using older firmware without diagnostics, it reports diagnostics unavailable rather than inventing a reason. The screenshot displaying `Request forward` and the Buddy2 direct-motion note identifies the older dashboard.
