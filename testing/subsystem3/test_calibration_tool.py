"""Exercise calibration-sample input and generated defines without touching files."""
from pathlib import Path
import subprocess
import sys
import unittest
TOOL=Path(__file__).resolve().parents[2]/'tools/line_calibration.py'
class CalibrationToolTests(unittest.TestCase):
    def run_tool(self,left_white='130,4095,131',left_black='3200',right_white='3300',right_black='150'):
        return subprocess.run([sys.executable,str(TOOL),'--left-white',left_white,'--left-black',left_black,
                               '--right-white',right_white,'--right-black',right_black],capture_output=True,text=True)
    def test_median_and_inverse_polarity(self):
        result=self.run_tool();self.assertEqual(result.returncode,0)
        self.assertIn('#define S3_WHITE_LEFT 131',result.stdout)
        self.assertIn('#define S3_WHITE_RIGHT 3300',result.stdout)
        self.assertIn('#define S3_BLACK_RIGHT 150',result.stdout)
    def test_insufficient_contrast(self):
        result=self.run_tool(left_black='150');self.assertNotEqual(result.returncode,0)
        self.assertNotIn('#define',result.stdout)
    def test_out_of_range(self):self.assertNotEqual(self.run_tool(left_white='4096').returncode,0)
    def test_non_numeric(self):self.assertNotEqual(self.run_tool(left_white='white').returncode,0)
    def test_missing_samples(self):self.assertNotEqual(self.run_tool(left_white='').returncode,0)
if __name__=='__main__':unittest.main()
