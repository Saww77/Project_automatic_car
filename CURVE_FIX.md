# Slow curve-following fix

Reviewed the supplied firmware, project briefing and write-up. The track photo contains tight bends, loops and crossings.

## Changes
- Line-follow wheel requests use level 1 instead of level 2: target 60 RPM instead of 120 RPM with MAX_RPM=240. Actual speed depends on encoder/PID calibration.
- Curve corrections coast the inner wheel and drive the outer wheel at level 1.
- Require an outer sensor to reach LINE_DARK=600 as well as a 150-point left/right difference before steering. Weak white-surface imbalance must not command a stopped inner wheel.
- With two sensors, white/white restores both wheels to level 1 and clears remembered steering. With three sensors enabled, retain steering through line loss and stop after 1000 ms without reacquisition.
- Remove the extra outer-sensor smoothing delay.
- Stop on an ADC read failure and invalidate cached wheel commands so successful reads can resume control.

## Wiring and tuning
The uploaded firmware enables only left GP26 and right GP28 (S3_IR_MASK=0x5); this is preserved. If all three sensors are physically installed, set S3_IR_MASK to 0x7 in app_program/subsystem3/subsystem3.h. The centre sensor then confirms centred travel and allows timeout detection when all three sensors see white. With only two sensors, centred white/white and completely missing the track are physically ambiguous; this software cannot reliably distinguish them. Two-sensor mode prioritises restoring straight travel when neither outer sensor sees black. It cannot reliably recover a completely lost line; a centre sensor is needed for unambiguous loss detection.

Calibrate white_raw and black_raw in task_line_follow.c separately for each installed sensor on YOUR track. Defaults remain 130 white and 3200 black; these are assumptions, not measured calibration. Confirm black gives a higher ADC reading than white. Confirm left/right wiring and motor direction with wheels raised first. If level 1 cannot turn the loaded wheels, calibrate the motion feedforward/PID rather than increasing every line-follow command blindly.

## Test on the photographed track
1. Build using the existing README instructions and flash the newly built UF2, not an older image from the archive.
2. Start on a straight, then test left and right loops individually. Verify the wheel nearer the line slows when an outer sensor sees black.
3. After a bend, confirm reacquisition without excessive pivoting. Tune POS_STRAIGHT (currently 150) and LOST_TIMEOUT_MS (currently 1000) using actual sensor logs; a smaller threshold reacts earlier, a shorter timeout reduces recovery travel.
4. In three-sensor mode, remove the line during a turn and while straight: verify recovery times out and stops. In two-sensor mode, verify both wheels resume when both outer sensors see white; automatic loss detection is unavailable.
5. The existing STOP-at-first-junction test remains enabled, and barcode code remains disabled. Completing the full photographed route requires separate junction/barcode integration. Do not disable this stop and assume crossings are solved.

## Validation
Run: python3 testing/subsystem3/test_line_follow.py
PASS for both 0x5 and 0x7 configurations: straight, left/right curves, loss recovery, timeout stop, reacquisition, and 1331 output-bound combinations per configuration. Tests compile the actual line_decide function using host GCC.
Full firmware build attempted but blocked: arm-none-eabi-gcc is not installed in this cloud machine; Pico SDK is also not configured. No new UF2 was generated. Hardware curve performance, stopping distance, sensor calibration and motor tuning have not been validated.

## Left wheel stopped on straight: diagnostic
The earlier controller could stop the left wheel for a modest left/right sensor offset, then retain that pivot on white/white. The revised controller rejects readings below LINE_DARK before steering and clears turn memory on white/white in two-sensor mode. Every 500 ms, the USB console now prints calibrated darkness and requested wheel levels.
- `wheels L=0 R=1`: the controller sees a left-line correction. Check reported darkness, sensor placement and white/black calibration.
- `wheels L=1 R=1` with the left wheel physically stationary: both wheels are being requested; inspect motion fault output, left encoder/motor wiring and low-speed PID/feedforward. A level-1 target does not guarantee the motor can overcome static friction.
Do not increase speed or change motor polarity without confirming which case applies. Full firmware build and physical verification remain outstanding.
