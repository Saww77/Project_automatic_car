#!/usr/bin/env python3
"""Display SE09 binary-v2 telemetry and optionally send a one-shot START/STOP."""
import argparse
import json
import time

STATES = ('IDLE', 'FOLLOW_LINE', 'TURNING', 'AVOIDING', 'RECOVER_LINE', 'FAULT')

def bits(payload, offset, width):
    return (int.from_bytes(payload, 'little') >> offset) & ((1 << width) - 1)

def decode(topic, payload):
    kind = topic.rsplit('/', 1)[-1]
    lengths = {'status': 1, 'motion': 9, 'line': 1, 'terrain': 2,
               'obstacle': 6, 'vehicle': 1, 'ack': 1, 'command': 1, 'controller': 25}
    if kind not in lengths or len(payload) != lengths[kind]:
        return {'error': 'unknown topic or incorrect binary payload length', 'hex': payload.hex()}
    if kind == 'controller':
        if payload[0] != 1:
            return {'error': 'unknown controller diagnostics version'}
        return dict(state=payload[1], stop_reason=payload[2], start_block=payload[3],
                    ready_flags=payload[4], motor_faults=payload[5],
                    left_dark=int.from_bytes(payload[6:8], 'little'),
                    centre_dark=int.from_bytes(payload[8:10], 'little'),
                    right_dark=int.from_bytes(payload[10:12], 'little'),
                    left_target_rpm=payload[12], right_target_rpm=payload[13],
                    line_age_ms=int.from_bytes(payload[14:16], 'little'),
                    uptime_ms=int.from_bytes(payload[16:20], 'little'),
                    telemetry_drops=int.from_bytes(payload[20:24], 'little'),
                    last_start_reject=payload[24])
    if kind == 'motion':
        return dict(mode=bits(payload, 0, 2), speed_level=bits(payload, 2, 3),
                    faults=bits(payload, 5, 4), left_rpm=bits(payload, 9, 9),
                    right_rpm=bits(payload, 18, 9), left_pwm=bits(payload, 27, 7),
                    right_pwm=bits(payload, 34, 7), left_progress=bits(payload, 41, 13),
                    right_progress=bits(payload, 54, 13))
    if kind == 'vehicle':
        state = payload[0] & 7
        return {'state': STATES[state] if state < len(STATES) else 'UNKNOWN', 'faults': payload[0] >> 3}
    if kind == 'line':
        return {'ir_bits': payload[0] & 7, 'barcode': (payload[0] >> 3) & 7}
    if kind == 'terrain':
        return {'height_mm': bits(payload, 0, 11), 'phase': bits(payload, 11, 3)}
    if kind == 'obstacle':
        return dict(nearest_mm=bits(payload, 0, 12), width_mm=bits(payload, 12, 11),
                    left_mm=bits(payload, 23, 12), right_mm=bits(payload, 35, 12))
    if kind == 'ack':
        return {'command': 'START' if payload[0] & 1 else 'STOP',
                'accepted_for_processing': not bool(payload[0] & 2)}
    return {'value': payload[0]}

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--port', type=int, default=1883)
    parser.add_argument('--root', default='robot/se09-car/v2')
    parser.add_argument('--command', choices=['start', 'stop'])
    args = parser.parse_args()
    if any(x in args.root for x in ('#', '+')):
        parser.error('root must not contain MQTT wildcards')
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        parser.error('Install paho-mqtt>=2,<3 in your Python environment first')
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                         client_id=f'se09-monitor-{time.time_ns()}')
    sent = False
    def on_connect(client, userdata, flags, reason, properties):
        if reason.is_failure:
            print('Broker connection failed:', reason)
            return
        client.subscribe(args.root.rstrip('/') + '/#')
    def on_subscribe(client, userdata, mid, reasons, properties):
        nonlocal sent
        if args.command and not sent and not any(code.is_failure for code in reasons):
            sent = True  # Never replay START automatically after reconnect.
            client.publish(args.root.rstrip('/') + '/command',
                           bytes([args.command == 'start']), qos=0, retain=False)
    def on_message(client, userdata, message):
        print(json.dumps({'received_at': time.time(), 'topic': message.topic,
                          'retained': message.retain, **decode(message.topic, message.payload)}), flush=True)
    client.on_connect = on_connect
    client.on_subscribe = on_subscribe
    client.on_message = on_message
    client.connect(args.host, args.port, keepalive=30)
    try:
        client.loop_forever()
    except KeyboardInterrupt:
        client.disconnect()

if __name__ == '__main__':
    main()
