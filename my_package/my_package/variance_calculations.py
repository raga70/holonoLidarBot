import math
import numpy as np

def var_resolution(wheel_radius, resolution):
    """ Calculates the variance created by the encoder resolution.
        This is in meters. The calculation is calculating how many radians
        the encoder misses.

    Args:
        wheel_radius (_type_): wheel radius in meters 
        resolution (_type_): Resolution of the encoder in ticks per revolution 
    """
    delta = (2*math.pi*wheel_radius) / resolution

    return (delta/2)**2

def var_gearbox_backlash(encoder_ang_vel, wheelbacklash, wheel):
    """ Calculates the effect of the backlash on the velocities.
        This is done by using the kinematics equation, so that we have
        an accurate estimate of the contribution of the slippage of every wheel.
        We only use the sign of the encoder angular velocities, and the magnitude of 
        the wheelbacklash

    Args:
        encoder_ang_vel (_type_): An array with the angular velocities givenback by the encoders
        wheelbacklash (_type_): the wheel backlash in radians 
        wheel (_type_): Wheel object, used for the kinematics 
    """

    wheel_slippages = np.sign(encoder_ang_vel) * wheelbacklash
    return wheel.calculate_robot_velocities(wheel_slippages)