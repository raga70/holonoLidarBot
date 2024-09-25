import numpy as np
from math_utils import z_rotation_matrix


class KinematicMechanumWheel:

    def __init__(self, y_to_wheel, x_to_wheel, wheel_radius, angle_from_wheels):
        self.y_to_wheel = y_to_wheel
        self.x_to_wheel = x_to_wheel
        self.wheel_radius = wheel_radius

        self.alphas = np.array([np.pi/4.0, -np.pi/4.0, 3*np.pi/4.0, -3*np.pi/4.0])
        self.betas = np.array([np.pi/2.0, -np.pi/2.0, np.pi/2.0, -np.pi/2.])
        self.yetas = np.array([-np.pi/4, np.pi/4, np.pi/4, -np.pi/4])

        self.length = np.sqrt(self.y_to_wheel**2 + self.x_to_wheel**2)

        self.T_wo_r = np.array([
            [
                (np.cos(self.betas[0] - self.yetas[0]))/np.sin(self.yetas[0]),
                (np.sin(self.betas[0] - self.yetas[0]))/np.sin(self.yetas[0]),
                (self.length*np.sin(self.betas[0]-self.yetas[0]-self.alphas[0]))/np.sin(self.yetas[0])
            ],
            [
                (np.cos(self.betas[1] - self.yetas[1]))/np.sin(self.yetas[1]),
                (np.sin(self.betas[1] - self.yetas[1]))/np.sin(self.yetas[1]),
                (self.length*np.sin(self.betas[1]-self.yetas[1]-self.alphas[1]))/np.sin(self.yetas[1])
            ],
            [
                (np.cos(self.betas[2] - self.yetas[2]))/np.sin(self.yetas[2]),
                (np.sin(self.betas[2] - self.yetas[2]))/np.sin(self.yetas[2]),
                (self.length*np.sin(self.betas[2]-self.yetas[2]-self.alphas[2]))/np.sin(self.yetas[2])
            ],
            [
                (np.cos(self.betas[3] - self.yetas[3]))/np.sin(self.yetas[3]),
                (np.sin(self.betas[3] - self.yetas[3]))/np.sin(self.yetas[3]),
                (self.length*np.sin(self.betas[3]-self.yetas[3]-self.alphas[3]))/np.sin(self.yetas[3])
            ]
        ])
        print(self.T_wo_r)
        self.T_wo_r = np.array([
            [1, -1, -(self.x_to_wheel+self.y_to_wheel)],
            [1, 1, (self.x_to_wheel + self.y_to_wheel)],
            [1, 1, -(self.x_to_wheel + self.y_to_wheel)],
            [1, -1, (self.x_to_wheel + self.y_to_wheel)],
        ])

        self.T = (1/self.wheel_radius) * self.T_wo_r
        

        # NOTE(Chris): We should be doing this but the inversion of (T^T @ T) is singular no idea why
        #               Maybe the original T matrix is not completely correctly defined
        self.T_fwd_dir = self.wheel_radius/4 * np.array(
            [
            [1, 1, 1, 1],
            [-1, 1, 1, -1],
            [-1/(self.x_to_wheel + self.y_to_wheel), 1/(self.x_to_wheel + self.y_to_wheel),-1/(self.x_to_wheel + self.y_to_wheel),1/(self.x_to_wheel + self.y_to_wheel)]
        ])

        self.T_fwd = np.linalg.pinv(self.T.transpose() @ self.T) @ self.T.transpose()

    def test_calculate_wheel_velocities(self, velocities):
        return self.T @ velocities

    def calculate_wheel_velocities(self, velocities):
        rot_vel = z_rotation_matrix(np.radians(180))@velocities[:2]
        rot_vel = np.append(rot_vel, velocities[2])
        return self.T @ rot_vel 

    def calculate_robot_velocities(self, ang_velocities):
        r_vel = self.T_fwd @ ang_velocities
        #rot_vel = z_rotation_matrix(np.radians(-90))@r_vel[:2].T
        return r_vel

def setup_wheel() -> KinematicMechanumWheel:
    y_to_wheel = (15/100)
    x_to_wheel =  (15/100)
    radius = ((8/2)/100) 
    angle_from_wheels = np.pi/2
    return KinematicMechanumWheel(x_to_wheel, y_to_wheel, radius, angle_from_wheels)


if __name__ == "__main__":
    wheel = setup_wheel()
    velocities = np.array([0, 0, 1])
    ang_vel = wheel.calculate_wheel_velocities(velocities)
    print(ang_vel)
