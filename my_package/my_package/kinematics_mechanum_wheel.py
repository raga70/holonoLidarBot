import numpy as np

class KinematicMechanumWheel:

    def __init__(self, y_to_wheel, x_to_wheel, wheel_radius, angle_from_wheels):
        self.y_to_wheel = y_to_wheel
        self.x_to_wheel = x_to_wheel
        self.wheel_radius = wheel_radius
        self.beta = angle_from_wheels
        self.alpha = np.arctan(self.y_to_wheel / self.x_to_wheel)
        self.yetas = np.array([np.radians(-45), np.radians(45), np.radians(45), np.radians(-45)])
        self.length = np.sqrt(self.y_to_wheel**2 + self.x_to_wheel**2)

        self.T_wo_r = np.array([
            [np.cos(self.beta - self.yetas[0])/np.sin(self.yetas[0]), np.sin(self.beta - self.yetas[0])/np.sin(self.yetas[0]), self.length * np.sin(self.beta - self.yetas[0] - self.alpha)/np.sin(self.yetas[0])],
            [np.cos(self.beta - self.yetas[1])/np.sin(self.yetas[1]), np.sin(self.beta - self.yetas[1])/np.sin(self.yetas[1]), self.length * np.sin(self.beta - self.yetas[1] - self.alpha)/np.sin(self.yetas[1])],
            [np.cos(self.beta - self.yetas[2])/np.sin(self.yetas[2]), np.sin(self.beta - self.yetas[2])/np.sin(self.yetas[2]), self.length * np.sin(self.beta - self.yetas[2] - self.alpha)/np.sin(self.yetas[2])],
            [np.cos(self.beta - self.yetas[3])/np.sin(self.yetas[3]), np.sin(self.beta - self.yetas[3])/np.sin(self.yetas[3]), self.length * np.sin(self.beta - self.yetas[3] - self.alpha)/np.sin(self.yetas[3])],
        ])

        self.T = 1/self.wheel_radius * self.T_wo_r

        # NOTE(Chris): We should be doing this but the inversion of (T^T @ T) is singular no idea why
        #               Maybe the original T matrix is not completely correctly defined
        self.T_fwd_dir = self.wheel_radius/4 * np.array(
            [
            [1, 1, 1, 1],
            [-1, 1, 1, -1],
            [-1/(self.x_to_wheel + self.y_to_wheel), 1/(self.x_to_wheel + self.y_to_wheel),-1/(self.x_to_wheel + self.y_to_wheel),1/(self.x_to_wheel + self.y_to_wheel)]
        ])

        self.T_fwd = np.linalg.pinv(self.T.transpose() @ self.T) @ self.T.transpose()


    def calculate_wheel_velocities(self, velocities):
        return self.T @ velocities

    def calculate_robot_velocities(self, ang_velocities):
        return  self.T_fwd @ ang_velocities 