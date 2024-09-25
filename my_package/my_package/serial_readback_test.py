import unittest
from kinematics_mechanum_wheel import KinematicMechanumWheel
from odometry_publisher import convert_serial_data_to_angular_velocities
import numpy as np


class TestSerialReadback(unittest.TestCase):

    def test_wrong_input(self):
        test_bytes = b'0,0,0'
        velocities = convert_serial_data_to_angular_velocities(test_bytes, None)
        self.assertIsNone(velocities, f"recieved output: {velocities}")

    def test_correct_output(self):
        test_bytes = b'0,0,0,0'
        correct_output = np.array([0., 0., 0., 0.])
        velocities = convert_serial_data_to_angular_velocities(test_bytes, None)
        msg = f"expected: {correct_output}, got {velocities}"
        print(msg)
        self.assertTrue((velocities==correct_output).all(), msg)

    
if __name__ == "__main__":
    unittest.main()
