> Current steering implementation: see [LINE_FOLLOWING_GUIDE.md](LINE_FOLLOWING_GUIDE.md). The notes below describe historical baseline/rollback behaviour, superseded by PD TRACK/TIGHT/RECOVER/LOST control. Motor timing and manual fault recovery are retained.

> Vehicle-controller integration: line_decide() and motor tuning remain at the restored baseline. The line task now submits recommendations to the central controller using a direct 10 ms release, rather than writing motor commands. MQTT builds wait for START. See VEHICLE_CONTROLLER.md for the new ownership, telemetry and startup behaviour. Physical curve performance remains unverified.

# Rollback after v4 failed to turn

The driver reported that v4 no longer turned sufficiently into curves. The curve controller, header and regression tests have been restored exactly from b41f50c, the earlier version reported as improved. This restores immediate steering, 30 RPM straights and 24 RPM outer-wheel turns. It removes subsequent turn holds and steering ramps. It does not resolve the previously reported tight-loop limitation.

Flash `SE09_rollback_improved_b41f50c.uf2`. Build and host tests were rerun after restoration. No physical car validation was performed. Further tuning needs sensor darkness and actual RPM/PWM readings at the point of failure. The existing junction stop remains enabled.

---

# Very slow line and curve following

## Current behaviour
- Straight target: 30 RPM on both wheels, approximately 10.5 cm/s with the configured 67 mm wheel diameter. This is a requested speed, not a measured road speed.
- Gentle curve: outer wheel 24 RPM, inner wheel 12 RPM. Both wheels keep moving.
- Sharp outer-only reading (left/right difference >=750): outer wheel 24 RPM and inner wheel coasting. No reverse wheel command is used.
- Steering requires a calibrated black reading >=600 and left/right difference >=150. White-surface offsets below the black threshold do not stop a wheel.
- The new forward-only API accepts even wheel RPM targets 0..126, encoded in a 16-bit queue command. Existing level-based APIs retain their 0..4 meaning and existing speeds.
- At targets below 60 RPM, the integral divisor is 50 rather than 1000, allowing up to 40 percentage points of correction instead of 2. This addresses the old inability to increase duty sufficiently at low speed. Existing >=60 RPM gain is unchanged. These starting gains still need physical tuning; higher PWM corrects speed error and does not change the requested RPM.
- Encoder, timing, stall and command timeout protections remain enabled. Faults stay latched instead of being silently cleared and retried.

## Sensor configuration and limitations
The uploaded configuration enables only left GP26 and right GP28 (S3_IR_MASK=0x5), which is preserved. If GP27 centre is physically connected and calibrated, change the mask to 0x7. Do not enable an unconnected analogue input.

With two outer sensors, white/white means either the line is correctly between the sensors or the line is completely lost. The firmware restores both wheels to 30 RPM on white/white to avoid the previous latched-left-stop regression. It cannot reliably distinguish those two physical situations. Three sensors permit steering memory on line loss and a stop after 1000 ms without reacquisition. Three sensors improve detection; they do not remove the need for track calibration.

Measure white_raw and black_raw for each installed sensor in task_line_follow.c. Values remain the uploaded assumptions (130 white, 3200 black), not measured calibration. Black must read higher than white with this conversion. Sensor placement and tape width affect when an outer sensor detects a curve.

The existing STOP-at-first-junction test remains enabled, and barcode navigation remains disabled. The photographed course has crossings and barcodes, so completing the full route requires that separate integration. A junction halt is intentional in this firmware.

## Build and flash
Follow README.md for the Pico SDK and Arm compiler. From build_make run `make -j2`. Flash the NEW `mtk3pico_smp1_usb_cdc_wifi.uf2` using BOOTSEL. Older firmware from the original ZIP does not contain these changes.

In this cloud workspace the verified tools are available with:
```
PATH=/workspace/tooling/arm/usr/bin:$PATH PICO_SDK_PATH=/workspace/tooling/pico-sdk make -C /workspace/Project_automatic_car/build_make -j2
```
This builds the default dual-core USB-console/Wi-Fi-driver profile, without MQTT. Wi-Fi credentials are excluded from Git.

## Validation completed
- Actual firmware built and linked with Arm GCC 14.2.1, Pico SDK 2.2.0, TinyUSB and cyw43 dependencies; a new UF2 was generated.
- `python3 testing/subsystem3/test_line_follow.py`: actual pure line controller compiled and checked in two- and three-sensor modes, including slow straights, both gentle and sharp curve directions, recovery, timeout, white-surface imbalance and 1331 bounded output combinations per mode.
- `python3 testing/subsystem2/test_motion_core_host.py`: actual motor core, command transport and owner decoder checked with mocked time/PWM/encoders. All 4095 nonzero fine-RPM command pairs round-trip, invalid commands reject, zero requests stop, a stopped left wheel restarts, and a stalled wheel stops both motors. Legacy level-1 behaviour is preserved.
These are build and simulated checks; no physical car or track was available. Curve tracking, actual speed, low-speed motor start, stopping distance and PID stability need on-car verification.

## On-car check
Test both wheels raised first, then a straight and each curve separately. USB output every 500 ms shows `[S3] dark ... target RPM ...` and `[MOTION] actual rpm ... PWM ... faults ...`. If one wheel has nonzero target but zero actual RPM, use its PWM and fault evidence to tune low-speed control rather than increasing cruise RPM. Tune KI_LOW_SPEED_DEN only using measured response; a larger divisor reduces integral action.

Fault bits: 1 encoder count, 2 timing, 4 stall, 8 command timeout. Inspect the cause before explicitly clearing a fault or power cycling. A low-speed stall must not be hidden by repeatedly resetting faults.
