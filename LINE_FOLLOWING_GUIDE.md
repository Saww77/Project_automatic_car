# PD line following: setup, calibration and tuning

This implementation uses the existing two front sensors (left AO → GP26, right AO → GP28),20 mm tape and the vehicle controller/motor PID already in the project. It adds PD steering, confirmed tight-curve handling and bounded recovery. It has been built and tested in software, not validated on the physical car. Default calibration is still the old130/3200 assumption until you measure it.

## 1. Keep the hardware setup fixed

Measure between the actual underside sensing points, not the circuit-board edges. For20 mm tape,25–30 mm separation is a starting point, not a guarantee: when centred, both sensing points should be just outside the black edges. Verify this using actual readings. Mount both at the same height and distance ahead of the wheel axle. Use the sensor's specified sensing distance and ensure stable contrast at that height. Record the forward offset; it affects tight curves.

Your separate third/barcode sensor is not treated as a centre tracking sensor. S3_IR_MASK stays0x5. Do not change it to0x7 unless a sensor is physically aligned between the front tracking pair and wired to GP27. Use AO (analogue output), not DO, for this controller, with signals compatible with the Pico ADC.

## 2. Flash the motor-disabled calibration image FIRST

Image: `SE09_line_calibration_mqtt.uf2`. Hold BOOTSEL, connect USB, then copy that UF2 to RPI-RP2. Open USB serial at115200 baud (PuTTY/Serial Monitor on Windows; the COM port may change after flashing).

Calibration mode prints, for example:

```
[CALIBRATION] raw L=... C=0 R=... mask=0x5 MOTORS DISABLED
```

This build refuses START and disables autostart even if a fresh movement recommendation is submitted. The matching dashboard reports “Sensor calibration mode — motors disabled.” C=0 is expected with your two-sensor mask. Calibration does not drive the motors.

Collect four measurements while preserving the mounted height:

| Measurement | What to do |
|---|---|
| Left white | Put white track surface under the left sensing point; record5–10 stable L samples |
| Left black | Put black tape directly under that point; record5–10 stable L samples |
| Right white | Put white surface under the right point; record5–10 stable R samples |
| Right black | Put black tape directly under that point; record5–10 stable R samples |

Take a median/representative value for each. Send me those FOUR numbers if you want me to apply the measured calibration. Black can legitimately read lower than white; the code supports either polarity independently for each sensor. A span below200 ADC counts is rejected rather than dividing by a near-zero calibration. If contrast is poor, check AO selection, height, wiring and surface before tuning steering. The trim potentiometer on some modules changes only DO's comparator threshold, not AO.

Values go in `app_program/subsystem3/line_config.h`:

- S3_WHITE_LEFT / S3_BLACK_LEFT
- S3_WHITE_RIGHT / S3_BLACK_RIGHT

You can produce replacement values from comma-separated samples with:

```sh
python3 tools/line_calibration.py --left-white WHITE_L_SAMPLES --left-black BLACK_L_SAMPLES --right-white WHITE_R_SAMPLES --right-black BLACK_R_SAMPLES
```

Replace each sample placeholder with raw integers separated by commas. The helper prints four defines; it does not overwrite code or move the car. Rebuild the driving firmware after updating calibration. The supplied driving candidate uses the existing default calibration, not invented measurements from your hardware.

## 3. Build / flash the DRIVING profile

With the README's Arm compiler and Pico SDK configured:

```sh
cd build_make
make -j2 NET=lwip MQTT=1 LINE_CALIBRATE=1  # calibration; motors disabled
make -j2 NET=lwip MQTT=1 LINE_CALIBRATE=0  # driving PD + MQTT; waits for START
```

The calibration output has `_line_calibration` in its name. Build-profile tracking includes this option, so switching profiles invalidates old objects. Calibration cannot be combined with TEST=1 or BUDDY1_DEMO=1. The non-MQTT normal build (`make -j2`) retains autostart; use the MQTT profile for controlled START/STOP testing.

The driving artifact is `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2`; the supplied candidate is named `SE09_PD_tracking_mqtt.uf2`. Keep existing Wi-Fi/broker credentials in ignored config/wifi_secrets.h. The dashboard's normal steering/motor diagnostics remain compatible. The dashboard update accompanying this change also recognises the calibration-only blocker.

## 4. Verify direction with wheels RAISED

After flashing driving firmware, wait at least five seconds, then START from the dashboard. With both tracking sensors over white, wheel targets should both be24 RPM. Confirm actual motors turn forward.

Move tape under LEFT only: error should become negative, the left target should fall relative to the right, and sustained strong detection should enter TIGHT with target0/20. Under RIGHT only: positive error, right target lower, sustained TIGHT target20/0. If the physical wheel response is reversed, correct the sensor/wheel mapping before any gain tuning. STOP is always the next action before repositioning.

Normalized white readings should be near0, black near1000. Defaults are acceptable for a first raised-wheel check only if those endpoints and polarity are actually verified; otherwise apply the measured calibration first. Raw measurements are printed on USB so a normalization assumption cannot hide a swapped/inverted sensor.

## 5. Understand the TWO state machines

The vehicle controller is the final authority. Its active states are IDLE → FOLLOW_LINE → IDLE/FAULT. START requires ready motors, fresh valid line recommendations and no interlock; STOP, faults and a running junction halt stop motion. Explicit Reset fault clears a stopped motor fault to IDLE, never automatic movement. Vehicle TURNING/AVOIDING/RECOVER_LINE enum values are reserved; those complete-mission actions are not implemented.

The line controller recommends wheel RPM while FOLLOW_LINE is active:

| Line mode | USB code | Behaviour / transition |
|---|---:|---|
| TRACK |0| PD steering. Straight base24 RPM; base reduces with error. Nonzero tracking wheel targets are at least12 RPM; maximum30 RPM. |
| TIGHT |1| Strong one-sided error (>=650) sustained for60 ms in the same direction: outer20 RPM, inner0. Weak/centred readings for20 ms or a visible opposite edge exit to TRACK. |
| RECOVER |2| After a confirmed TIGHT turn loses the line, continue slowly in its last reliable direction for at most180 ms. A visible line exits to TRACK. |
| LOST |3| Recovery timeout recommends0/0 and stop. Place the car back over the line before requesting another START. |

The line state resets while the vehicle is not following. It cannot silently resume the mission after a stop. Junction detection is still armed only while following; a real running junction stop remains latched until reboot. Barcode navigation is still disabled. This implementation does not claim full-route navigation.

Two outer sensors both reading white cannot distinguish centred travel from complete loss. In TRACK, correction decays toward straight rather than inventing a search. Only a previously confirmed tight turn arms recovery. Adding an aligned centre sensor improves observability; PD alone cannot remove this two-sensor ambiguity.

## 6. What PD means in THIS code

`line_control.c` is hardware-independent C, tested directly. Error is normalized R minus L (range approximately-1000..1000). P uses the filtered error; D uses filtered error-change normalized to elapsed time, suppressing derivative kick at initialisation/reacquisition. D damps a correction and is prevented from creating an opposite correction from a fading same-side signal. There is no steering integral. Buddy2's independent wheel-speed PID remains unchanged.

Initial knobs in line_config.h:

| Knob | Initial value | Purpose |
|---|---:|---|
| S3_KP_MILLI |18| Steering RPM correction per1000 error |
| S3_KD_MILLI |12| D correction per1000 error-change per nominal10 ms |
| S3_CRUISE_RPM |24| Straight speed |
| S3_TIGHT_OUTER_RPM |20| Slow outer speed during tight turn/recovery |
| S3_TIGHT_ENTER_ERROR |650| Strong one-sided evidence threshold |
| S3_TIGHT_CONFIRM_MS |60| Reject isolated hits before TIGHT |
| S3_RECOVERY_MS |180| Hard bound on blind search after a confirmed turn |

Do not raise recovery time indefinitely. Targets remain forward-only0..30 and even RPM for the existing compact motor-command transport. Counter-rotation, active braking and direction-settling changes are NOT added by this PD revision; the earlier active-brake experiment was rolled back after your feedback.

## 7. Tune in this order

1. **Straight only.** Start centred on a straight with correct calibration. Verify stable normalized readings and actual wheel RPM tracking equal targets. If actual RPM oscillates while targets stay equal, diagnose the motor loop rather than changing steering gains.
2. **Broad left and right curves.** If it oscillates across the line, first lower Kp by2; test again. If it still overshoots despite correct direction and readings, try increasing Kd by2. Change ONE parameter per run. If it is consistently too weak with a sustained error, increase Kp by2, subject to target saturation.
3. **The failing loop.** Watch mode1/TIGHT, target RPM and actual RPM. If strong calibrated one-sided error persists but never enters TIGHT, inspect threshold/confirmation and sample timing before making changes. After measured evidence, a lower entry threshold (for example650→600) can enter tight mode earlier; verify it does not falsely trigger on straights.
4. **Compare target with actual.** If TIGHT requests0/20 or20/0 but the nominally stopped wheel still rolls quickly, further Kp changes will not fix the motor/geometry response. If the inner wheel is actually stopped yet radius is still too large, measure track radius and wheel spacing. An ideal forward pivot has radius about half wheel spacing; tighter paths may need controlled counter-rotation and proper direction-settling support as separate work.
5. **Recovery.** mode2 should be brief. mode3 means recovery ran out; reposition and inspect readings/placement. Do not extend the blind search to make a test appear to pass.

The USB line printed every250 ms or on a mode/stop transition is:

```
[LINE_PD] mode=... raw=L/C/R dark=L/C/R error=... D=... dt=... target=L/R stop=...
```

Send a short failing-loop video together with these lines and the dashboard's actual wheel RPM. That separates poor line detection, PD tuning, recovery and motor tracking. Motor Control timing faults remain separate and must be addressed using measured gap/limit; the PD code does not disable those protections.

## Validation

Executed: actual pure-C PD tests for both sensor masks, straight noise, direction, derivative damping, tight entry, bounded search/reacquisition, elapsed time, S-curve reversals, both calibration polarities and1331 stateful sensor combinations. Existing controller/motor tests pass. A calibration-policy test attempts START with ready motors and a fresh recommendation and verifies it is refused, with autostart disabled. Driving and motor-disabled calibration profiles build using the pinned SDK/toolchain. Physical loop completion, actual mounted readings and tuned gains remain unverified. These are starting parameters, not a claim of measured tuning on your car.
