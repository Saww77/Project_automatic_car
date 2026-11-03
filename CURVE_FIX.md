# Very slow line and curve following

## Current behaviour (anti-zigzag revision)
- Straight and outer-wheel turning targets are both 20 RPM (about 7 cm/s with configured 67 mm wheels). This avoids accelerating every time a sensor returns to white.
- Ordinary corrections progressively reduce the inner target from 18 to 8 RPM according to sensor error instead of immediately stopping a wheel.
- A sharp pivot requires at least 80 ms of consecutive strong outer-only evidence (difference >=750) in the same direction. An isolated hit, direction reversal, or weaker reading resets that confirmation.
- A confirmed tight turn may continue gently for 40 ms through white/white. BOTH wheels move during that hold: inner target at least 12 RPM. A stopped-wheel pivot is never held after line loss.
- Existing low-speed PID, fault latching and motor protections are unchanged from the prior 30 RPM revision.
- Forward wheel RPM API accepts even targets 0..126. Legacy level-based APIs retain their original meaning.

## Sensor configuration and limitations
The uploaded configuration enables only left GP26 and right GP28 (S3_IR_MASK=0x5), which is preserved. If GP27 centre is physically connected and calibrated, change the mask to 0x7. Do not enable an unconnected analogue input.

With two outer sensors, white/white means either the line is correctly between the sensors or the line is completely lost. With two sensors, a confirmed strong turn gets a maximum 40 ms gentle hold through white/white, then returns to equal 20 RPM targets. Centre/balanced visible readings cancel steering immediately. A single outer-sensor hit cannot arm the hold. The firmware still cannot distinguish centred white/white from complete line loss with only two sensors. With three installed sensors enabled, complete line loss stops after 1000 ms. Three sensors improve detection; they do not remove the need for track calibration.

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
- `python3 testing/subsystem3/test_line_follow.py`: actual pure line controller compiled and checked in two- and three-sensor modes, including slow straights, earlier sharp-turn entry, left/right curves, confirmed turn holds, immediate wheel release, 40 ms soft-hold expiry, alternating-edge rejection, direction reversal, noise rejection, recovery timeout and 1331 bounded output combinations per mode.
- `python3 testing/subsystem2/test_motion_core_host.py`: actual motor core, command transport and owner decoder checked with mocked time/PWM/encoders. All 4095 nonzero fine-RPM command pairs round-trip, invalid commands reject, zero requests stop, a stopped left wheel restarts, and a stalled wheel stops both motors. Legacy level-1 behaviour is preserved.
These are build and simulated checks; no physical car or track was available. Curve tracking, actual speed, low-speed motor start, stopping distance and PID stability need on-car verification.

## On-car check
Test both wheels raised first, then a straight and each curve separately. USB output every 500 ms shows `[S3] dark ... target RPM ...` and `[MOTION] actual rpm ... PWM ... faults ...`. If one wheel has nonzero target but zero actual RPM, use its PWM and fault evidence to tune low-speed control rather than increasing cruise RPM. Tune KI_LOW_SPEED_DEN only using measured response; a larger divisor reduces integral action.

Fault bits: 1 encoder count, 2 timing, 4 stall, 8 command timeout. Inspect the cause before explicitly clearing a fault or power cycling. A low-speed stall must not be hidden by repeatedly resetting faults.

## On-car regression feedback
The driver reported improvement, then running wide on the bottom-right loop and top-left circle with two sensors. The 60 ms pivot hold / 120 ms turn hold revision then produced more zigzagging. This revision removes held pivots, reduces and softens the hold, requires sustained strong readings for sharp pivots, and uses a constant slower outer speed. These are tested control changes, not physical proof of stable tracking.

Flash SE09_smooth_slow_v3.uf2. Test a straight, then each loop separately. If it still zigzags, record the USB darkness/target-RPM and actual-RPM/PWM lines plus a short video showing the sensor positions; further changes need that evidence to distinguish sensor calibration, steering oscillation and motor PID oscillation. The junction-stop test remains enabled.
