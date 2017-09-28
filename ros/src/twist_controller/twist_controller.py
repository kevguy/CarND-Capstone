import rospy
import math
from pid import PID
from lowpass import LowPassFilter

DEBUG_MODE = True
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.vehicle_mass = args[0]
        self.fuel_capacity = args[1]
        self.brake_deadband = args[2]
        self.decel_limit = args[3]
        self.accel_limit = args[4]
        self.wheel_radius = args[5]
        self.wheel_base = args[6]
        self.steer_ratio = args[7]
        self.max_lat_accel = args[8]
        self.max_steer_angle = args[9]

        self.pid_velocity = PID(0.6, 0.05, 0.1)
        self.pid_steering = PID(6.0, 0.3, 1.0)
        # pass

    def control(self, *args, **kwargs):
        """
        arguments:
         - proposed linear velocity
         - proposed angular velocity (radians)
         - current linear velocity
         - elapsed time
         - dbw_enabled status
        """
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        target_velocity = args[0]
        target_steering = args[1]
        current_velocity = args[2]
        time_elapsed = args[3]
        dbw_enabled = args[4]

        velocity_error = target_velocity - current_velocity
        velocity = self.pid_velocity.step(velocity_error, time_elapsed)

        steering_error = target_steering
        steer = self.pid_steering.step(steering_error, time_elapsed)

        throttle = max(velocity, 0.0)
        brake = math.fabs(min(0.0, velocity))

        if DEBUG_MODE:
            rospy.logerr('final velocity: {}'.format(velocity))
            rospy.logerr('final steering: {}'.format(steer))

        # return 1., 0., 0.
        return throttle, brake, steer
