import unittest
from kinematics_mechanum_wheel import KinematicMechanumWheel
import numpy as np

def setup_wheel() -> KinematicMechanumWheel:
    y_to_wheel = (15/100)
    x_to_wheel =  (15/100)
    radius = ((8/2)/100) 
    angle_from_wheels = np.pi/2
    return KinematicMechanumWheel(x_to_wheel, y_to_wheel, radius, angle_from_wheels)

class TestKinematics(unittest.TestCase):
    wheel = setup_wheel()

    def test_forward_signs(self):
        expected_signs = np.array(
            [
                1, 1, 1, 1
            ]
        )
        input_vel = np.array([
            1.0,
            0.0,
            0.0
        ])
        ang_vel = self.wheel.test_calculate_wheel_velocities(input_vel)
        self.assertTrue((np.sign(ang_vel) == expected_signs).all(), f"{ang_vel} -> {expected_signs}")

    def test_backward_signs(self):
        expected_signs = np.array(
            [
                -1, -1, -1, -1
            ]
        )
        input_vel = np.array([
            -1.0,
            0.0,
            0.0
        ])
        ang_vel = self.wheel.test_calculate_wheel_velocities(input_vel)
        self.assertTrue((np.sign(ang_vel) == expected_signs).all(), f"{ang_vel} -> {expected_signs}")

    def test_left_signs(self):
        expected_signs = np.array(
            [
                -1, 1, 1, -1
            ]
        )
        input_vel = np.array([
            0.0,
            -1.0,
            0.0
        ])

        ang_vel = self.wheel.test_calculate_wheel_velocities(input_vel)
        self.assertTrue((np.sign(ang_vel) == expected_signs).all(), f"{ang_vel} -> {expected_signs}")

    def test_right_signs(self):
        expected_signs = np.array(
            [
                1, -1, -1, 1
            ]
        )
        input_vel = np.array([
            0.0,
            1.0,
            0.0
        ])

        ang_vel = self.wheel.test_calculate_wheel_velocities(input_vel)
        print(ang_vel)
        self.assertTrue((np.sign(ang_vel) == expected_signs).all(), f"{ang_vel} -> {expected_signs}")

    def test_left_forward_signs(self):
        expected_signs = np.array(
            [
                0.0, 1, 1, 0
            ]
        )
        input_vel = np.array([
            1.0,
            1.0,
            0.0
        ])

        ang_vel = self.wheel.test_calculate_wheel_velocities(input_vel)
        self.assertTrue((np.sign(ang_vel) == expected_signs).all(), f"{ang_vel} -> {expected_signs}")

    def test_left_backward_signs(self):
        expected_signs = np.array(
            [
                -1, 0, 0, -1 
            ]
        )
        input_vel = np.array([
            -1.0,
            1.0,
            0.0
        ])

        ang_vel = self.wheel.test_calculate_wheel_velocities(input_vel)
        self.assertTrue((np.sign(ang_vel) == expected_signs).all(), f"{ang_vel} -> {expected_signs}")

    def test_right_forward_signs(self):
        expected_signs = np.array(
            [
                1.0, 0, 0, 1
            ]
        )
        input_vel = np.array([
            1.0,
            -1.0,
            0.0
        ])

        ang_vel = self.wheel.test_calculate_wheel_velocities(input_vel)
        self.assertTrue((np.sign(ang_vel) == expected_signs).all(), f"{ang_vel} -> {expected_signs}")

    def test_right_backward_signs(self):
        expected_signs = np.array(
            [
                0.0, -1, -1, 0
            ]
        )
        input_vel = np.array([
            -1.0,
            -1.0,
            0.0
        ])

        ang_vel = self.wheel.test_calculate_wheel_velocities(input_vel)
        self.assertTrue((np.sign(ang_vel) == expected_signs).all(), f"{ang_vel} -> {expected_signs}")


    
if __name__ == "__main__":
    unittest.main()
