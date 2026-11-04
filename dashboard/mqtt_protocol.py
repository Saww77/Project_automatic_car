"""Pico binary-v2 wire format. No dependency on the firmware or MQTT client."""
STATES = ('IDLE', 'FOLLOW_LINE', 'TURNING', 'AVOIDING', 'RECOVER_LINE', 'FAULT')
MOTOR_MODES = ('IDLE', 'VELOCITY', 'DISTANCE', 'TURN')
MOTOR_FAULTS = ((1, 'Encoder counts'), (2, 'Control timing'), (4, 'Wheel stalled'), (8, 'Command timeout'))
VEHICLE_FAULTS = ((1, 'Motor'), (2, 'Encoder'), (4, 'IMU'), (8, 'Ultrasonic echo'), (16, 'Wi-Fi'))
TELEMETRY_TOPICS = ('vehicle', 'motion', 'line', 'terrain', 'obstacle')

def command_payload(command):
    if command == 'start': return b'\x01'
    if command == 'stop': return b'\x00'
    raise ValueError('command must be start or stop')

def decode_telemetry(kind, payload):
    sizes = dict(vehicle=1, motion=9, line=1, terrain=2, obstacle=6)
    if kind not in sizes or len(payload) != sizes[kind]:
        raise ValueError('Incorrect binary telemetry length')
    word = int.from_bytes(payload, 'little')
    def bits(offset, width): return (word >> offset) & ((1 << width) - 1)
    if kind == 'vehicle':
        state, faults = bits(0, 3), bits(3, 5)
        if state >= len(STATES): raise ValueError('Unknown vehicle state')
        return dict(state=STATES[state], state_code=state, faults=faults,
                    fault_labels=[label for bit, label in VEHICLE_FAULTS if faults & bit])
    if kind == 'motion':
        fields = dict(mode=bits(0, 2), speed_level=bits(2, 3), faults=bits(5, 4),
                      left_rpm=bits(9, 9), right_rpm=bits(18, 9),
                      left_pwm=bits(27, 7), right_pwm=bits(34, 7),
                      left_progress=bits(41, 13), right_progress=bits(54, 13))
        if word >> 67 or fields['speed_level'] > 4 or max(fields['left_rpm'], fields['right_rpm']) > 300 or max(fields['left_pwm'], fields['right_pwm']) > 100:
            raise ValueError('Invalid motion range or reserved bits')
        fields['mode_name'] = MOTOR_MODES[fields['mode']]
        fields['fault_labels'] = [label for bit, label in MOTOR_FAULTS if fields['faults'] & bit]
        return fields
    if kind == 'line':
        ir, barcode = bits(0, 3), bits(3, 3)
        if word >> 6 or barcode > 4: raise ValueError('Invalid line payload')
        return dict(ir_bits=ir, left_black=bool(ir & 1), centre_black=bool(ir & 2),
                    right_black=bool(ir & 4), barcode_code=barcode,
                    barcode='None' if barcode == 0 else 'ABCD'[barcode - 1])
    if kind == 'terrain':
        height, phase = bits(0, 11), bits(11, 3)
        if word >> 14 or height > 2000 or phase > 5: raise ValueError('Invalid terrain payload')
        return dict(height_mm=height, phase=phase)
    fields = dict(nearest_mm=bits(0, 12), width_mm=bits(12, 11),
                  left_mm=bits(23, 12), right_mm=bits(35, 12))
    if word >> 47 or fields['width_mm'] > 2000 or max(fields['nearest_mm'], fields['left_mm'], fields['right_mm']) > 4000:
        raise ValueError('Invalid obstacle payload')
    return fields
