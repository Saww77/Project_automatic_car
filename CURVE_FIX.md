# Active-brake curve correction

## Concrete motor-control correction
The prior controller requested a zero inner-wheel target during sharp turns, but motion_core.c implemented zero as both motor inputs LOW. That coasts the wheel. A coasting inner wheel can continue rolling forward and widen the actual turn, despite the line controller requesting a pivot.

The Robo Pico vendor demo explicitly uses CircuitPython throttle=0 to brake and throttle=None to coast. The corresponding Adafruit DCMotor implementation sets both inputs HIGH for braking and both LOW for coasting. This revision follows that documented brake mode for a zero-target wheel during velocity steering. It applies independently to left and right wheels. A subsequent forward command clears the brake outputs through the existing drive path. Full stops, initialization and fault stops retain the original all-zero outputs.

Authoritative references:
- https://github.com/CytronTechnologies/Robo-Pico/blob/main/Demo_Code/circuitpython/code.py (GP21 brake example)
- https://github.com/adafruit/Adafruit_CircuitPython_Motor/blob/main/adafruit_motor/motor.py (throttle setter)

S2_BRAKE_ZERO_WHEEL=1 selects this Robo Pico behaviour. It must not be assumed suitable for an unrelated external motor driver. With PWM TOP=999, compare=1000 holds each input HIGH continuously. Status PWM=0 still means zero propulsion demand; it does not mean the electrical brake is off.

## Line controller
The baseline remains b41f50c, the earlier version the driver reported as improved. Straight targets are30 RPM, outer curve targets24 RPM, gentle inner target12 RPM. Sharp turns now engage at an outer-sensor difference of600 instead of750, with the centre below the line threshold. A sharp turn requests zero inner RPM and the motor layer actively brakes that wheel. Subsequent experimental turn holds and slow steering ramps remain removed.

The two front tracking sensors remain GP26 left and GP28 right, S3_IR_MASK=0x5. The separate barcode sensor is not enabled as a centre tracking sensor. GPIO mapping and per-sensor white/black calibration still need physical confirmation; uploaded calibration values130/3200 are assumptions, not measurements. Two white outer readings are still ambiguous and resume straight travel. This revision does not invent a reliable lost-line detector from those two readings.

## Build and tests
Firmware built successfully using Arm GCC14.2.1 and Pico SDK2.2.0, with USB console and cyw43 driver. Run from this workspace:
```
PATH=/workspace/tooling/arm/usr/bin:$PATH PICO_SDK_PATH=/workspace/tooling/pico-sdk make -C /workspace/Project_automatic_car/build_make -j2
python3 testing/subsystem3/test_line_follow.py
python3 testing/subsystem2/test_motion_core_host.py
```
Line-controller tests pass in two-/three-sensor modes, including earlier sharp turns, straight travel, both curve directions and1331 output combinations each. Motor tests compile actual code with mocked PWM/time/encoders and verify BOTH electrical brake outputs on either pivot wheel, releasing the brake before forward drive, full-stop outputs, stall protection and4095 RPM-command encode/decode combinations.

These tests establish code and build behaviour, not physical track completion. The video shows a straight and left bend, not verified completion of the tight loop. Actual braking, sensor readings, loop tracking and stopping distance must be checked on the car. Test briefly with wheels raised first, then on the same failing loop using SE09_active_brake_curves.uf2. USB logs report requested wheel RPM, measured RPM, propulsion PWM and fault flags. Faults remain latched for inspection.

The existing first-junction stop and disabled barcode route handling remain unchanged; full-course navigation remains separate from this curve correction.
