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

    def calculate_wheel_velocities(self, velocities):
        return self.T @ velocities

def main():
    velocities = np.array([
        [0, 1, 0]
    ])
    y_to_wheel = (14/100)
    x_to_wheel =  (14/100)
    radius = ((8/2)/100) 
    angle_from_wheels = np.pi/2
    wheel = KinematicMechanumWheel(y_to_wheel, x_to_wheel, radius, angle_from_wheels)
    angular_velocities = wheel.T @ velocities.T
    print(angular_velocities)



if __name__ == "__main__":
    main()
