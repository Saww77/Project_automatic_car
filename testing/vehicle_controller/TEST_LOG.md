# Vehicle controller integration validation

Executed in the cloud workspace; all commands below passed.

- `python3 testing/vehicle_controller/test_vehicle.py`: actual controller/runtime/command-task/telemetry C compiled with GCC (`-Wall -Wextra -Werror`). Tests include single-owner asymmetric wheel recommendations; latest-value replacement; STOP priority and START/STOP/START batch ACKs; line-stop latching; invalid/stale line stopping; link and motor faults; explicit restart; obstacle interlock; wraparound and time sampled after locking; queue-full retry; non-MQTT readiness/autostart; binary telemetry, expiry, unavailable sensor omission and drop accounting.
- `python3 testing/vehicle_controller/test_monitor.py`: packed motion/line/vehicle/terrain decoding, ACK semantics and malformed-length rejection.
- `python3 testing/subsystem3/test_line_follow.py`: existing controller regression tests in both sensor configurations, including 1331 bounded output combinations per configuration.
- `python3 testing/subsystem2/test_motion_core_host.py`: existing motor/PWM/stall/command-transport regression tests, including 4095 fine-RPM command combinations.
- Normal Pico firmware `make -j2`: compiled and linked with Arm GCC 14.2.1 and Pico SDK 2.2.0. No compiler warnings/errors.
- MQTT Pico firmware `make -j2 NET=lwip MQTT=1`: compiled and linked with the SDK's pinned TinyUSB, cyw43 and lwIP submodules. No compiler warnings/errors. Ignored local Wi-Fi configuration reused without changing or publishing credentials.
- Changed tracked files passed `git -c core.whitespace=cr-at-eol diff --check`.

Not run: live Pico execution, real Wi-Fi association/reconnection, broker delivery and dashboard against the physical car, motor timing under on-car load, curve/loop completion, barcode-directed navigation and obstacle bypass. Host mocks do not establish those outcomes. See VEHICLE_CONTROLLER.md for the on-car sequence and remaining subsystem hooks.
