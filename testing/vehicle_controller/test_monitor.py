from pathlib import Path
import importlib.util
import sys
sys.dont_write_bytecode=True
root=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('monitor', root/'tools/mqtt_monitor.py')
monitor=importlib.util.module_from_spec(spec);spec.loader.exec_module(monitor)
value=(30<<9)|(24<<18)|(40<<27)|(35<<34)
result=monitor.decode('robot/car/v2/motion',value.to_bytes(9,'little'))
assert result['left_rpm']==30 and result['right_rpm']==24
assert result['left_pwm']==40 and result['right_pwm']==35
assert monitor.decode('x/vehicle',bytes([1]))['state']=='FOLLOW_LINE'
assert monitor.decode('x/line',bytes([4|(2<<3)]))=={'ir_bits':4,'barcode':2}
assert monitor.decode('x/ack',bytes([3]))['accepted_for_processing'] is False
assert monitor.decode('x/ack',bytes([0]))['command']=='STOP'
assert 'error' in monitor.decode('x/motion',bytes([1]))
assert monitor.decode('x/terrain',(123|(2<<11)).to_bytes(2,'little'))=={'height_mm':123,'phase':2}
print('PASS: monitor decodes packed firmware telemetry and rejects wrong lengths')
