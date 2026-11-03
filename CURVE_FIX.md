# Very slow line and curve following

## Current behaviour (curve controller v4)
- Straight and outer-wheel targets stay at 20 RPM (approximately 7 cm/s with configured 67 mm wheels).
- Ordinary turns progressively reduce the inner wheel target while keeping it above zero.
- Strong outer-only readings (difference >=650, centre below400) must persist in the same direction for40 ms before requesting a stopped inner wheel for tight bends.
- The signed steering correction changes by at most2 RPM per10 ms cycle. Direction reversals ramp through straight travel instead of instantly swapping a stopped wheel.
- When the line is centred or both outer sensors return to white, steering decays by at most4 RPM per10 ms cycle. A stopped inner wheel therefore restarts progressively and both wheel targets are equal again within50 ms.
- Threshold hysteresis enters line detection at600 darkness and releases below400, reducing rapid state changes near one threshold. These thresholds require valid per-sensor calibration.
- Existing low-speed PID, fault latching and motor protections are unchanged. Legacy level-based APIs retain their original meaning.

## Sensor configuration and limitations
The uploaded configuration enables only left GP26 and right GP28 (S3_IR_MASK=0x5), which is preserved. If GP27 centre is physically connected and calibrated, change the mask to 0x7. Do not enable an unconnected analogue input.

With two outer sensors, white/white means either the line is correctly between the sensors or the line is completely lost. With two sensors, white/white may mean either centred travel or complete line loss. This controller gradually releases steering on white/white; it does not claim to resolve that ambiguity. With three physically installed and aligned sensors enabled, complete line loss stops after1000 ms. Three sensors improve detection; they do not remove the need for track calibration.

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
- `python3 testing/subsystem3/test_line_follow.py`: actual pure line controller compiled and checked in two- and three-sensor modes, including sustained left/right tight bends, S-curve reversal, bounded wheel-target changes, hysteresis, alternating noise, gradual release, timeout and1331 output combinations per mode.
- `python3 testing/subsystem2/test_motion_core_host.py`: actual motor core, command transport and owner decoder checked with mocked time/PWM/encoders. All 4095 nonzero fine-RPM command pairs round-trip, invalid commands reject, zero requests stop, a stopped left wheel restarts, and a stalled wheel stops both motors. Legacy level-1 behaviour is preserved.
These are build and simulated checks; no physical car or track was available. Curve tracking, actual speed, low-speed motor start, stopping distance and PID stability need on-car verification.

## On-car check
Test both wheels raised first, then a straight and each curve separately. USB output every 500 ms shows `[S3] dark ... target RPM ...` and `[MOTION] actual rpm ... PWM ... faults ...`. If one wheel has nonzero target but zero actual RPM, use its PWM and fault evidence to tune low-speed control rather than increasing cruise RPM. Tune KI_LOW_SPEED_DEN only using measured response; a larger divisor reduces integral action.

Fault bits: 1 encoder count, 2 timing, 4 stall, 8 command timeout. Inspect the cause before explicitly clearing a fault or power cycling. A low-speed stall must not be hidden by repeatedly resetting faults.

## On-car regression feedback and next test
The driver reported running wide on tight loops, then increased zigzagging after stronger pivot/hold tuning. The uploaded14.4-second video shows the straight and a left bend, but does not establish full tight-loop completion. The sensor photo shows two front tracking modules and a separate third module; GPIO mapping, sensor height and actual white/black readings remain unverified.

Version4 replaces abrupt pivot/hold transitions with a rate-limited signed correction and line-detection hysteresis. Test `SE09_curve_control_v4.uf2` on a straight and left/right loops separately. Compare target RPM with actual RPM/PWM when a curve fails. A new build and simulated sequences do not prove physical track performance.

The existing first-junction stop and disabled barcode handling remain unchanged. This revision addresses curve control, not complete mission navigation.
