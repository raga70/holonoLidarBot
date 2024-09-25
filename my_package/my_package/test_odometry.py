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

    def test_right(self):
        input_vel = np.array(
            [
                1, 0, 0
            ]
        )
        ang_vel = self.wheel.test_calculate_wheel_velocities(input_vel)
        converted_vels = self.wheel.calculate_robot_velocities(ang_vel)
        self.assertTrue((input_vel == converted_vels).all(), f"{input_vel} -> {converted_vels}")

    def test_left(self):
        input_vel = np.array(
            [
                -1, 0, 0
            ]
        )
        ang_vel = self.wheel.test_calculate_wheel_velocities(input_vel)
        converted_vels = self.wheel.calculate_robot_velocities(ang_vel)
        self.assertTrue((input_vel == converted_vels).all(), f"{input_vel} -> {converted_vels}")
        
    def test_fwd(self):
        input_vel = np.array(
            [
                0, 1, 0
            ]
        )
        ang_vel = self.wheel.test_calculate_wheel_velocities(input_vel)
        converted_vels = self.wheel.calculate_robot_velocities(ang_vel)
        self.assertTrue((input_vel == converted_vels).all(), f"{input_vel} -> {converted_vels}")

    def test_bck(self):
        input_vel = np.array(
            [
                0, -1, 0
            ]
        )
        ang_vel = self.wheel.test_calculate_wheel_velocities(input_vel)
        converted_vels = self.wheel.calculate_robot_velocities(ang_vel)
        self.assertTrue((input_vel == converted_vels).all(), f"{input_vel} -> {converted_vels}")

    def test_rot_left(self):
        input_vel = np.array(
            [
                0, 0, 1 
            ]
        )
        ang_vel = self.wheel.test_calculate_wheel_velocities(input_vel)
        converted_vels = self.wheel.calculate_robot_velocities(ang_vel)
        self.assertTrue((input_vel == converted_vels).all(), f"{input_vel} -> {converted_vels}")

    def test_rot_right(self):
        input_vel = np.array(
            [
                0, 0, -1 
            ]
        )
        ang_vel = self.wheel.test_calculate_wheel_velocities(input_vel)
        converted_vels = self.wheel.calculate_robot_velocities(ang_vel)
        self.assertTrue((input_vel == converted_vels).all(), f"{input_vel} -> {converted_vels}")
if __name__ == "__main__":
    unittest.main()
