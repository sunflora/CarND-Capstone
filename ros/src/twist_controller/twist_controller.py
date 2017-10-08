'''

    2017.09.29  Adjust limiting parameters max linear acceleration
'''

from pid import PID
import rospy
from std_msgs.msg import Bool
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
LINEAR_PID_MIN = -1.0
LINEAR_PID_MAX = 0.2  # limit by max acc as 1m/s^2
ANGULAR_PID_MIN = -0.4
ANGULAR_PID_MAX = 0.35
ACCEL_SENSITIVITY = 0.06
MAX_BRAKE_VALUE = 250

SAMPLE_TIME = 0.1 #TODO: It's related to the publishing frequency of the twist command.

class TwistController(object):
    def __init__(self):
        self.linear_velocity_pid = PID(kp=ACCEL_SENSITIVITY*1.25,ki=0.003,kd=0.0,mn=LINEAR_PID_MIN,mx=LINEAR_PID_MAX)
        self.low_pass_filter = LowPassFilter(8.0, 2.0)  #(b,a) normalized, (0 , 1) = all pass
        self.yaw_controller = None
        pass

    def control(self, current_linear_velocity, target_linear_velocity, steer_sensitivity, target_angular_velocity):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        # if target_angular_velocity > 0.025:
        #     #rospy.logerr('')
        #     #rospy.logerr('target_linear_velocity.org {: f}'.format(target_linear_velocity))
        #     v = self.yaw_controller.get_max_linear_velocity(current_linear_velocity, target_angular_velocity)
        #     #rospy.logerr('max_linear_velocity(tangent) {: f}'.format(v))
        #     target_linear_velocity = min ( v, target_linear_velocity)
        
        
        sample_time = SAMPLE_TIME

        brake = 0.0

        if target_linear_velocity > 0.1:
            cte_linear = target_linear_velocity - current_linear_velocity
            throttle = self.linear_velocity_pid.step(cte_linear, sample_time)
            throttle = self.low_pass_filter.filt(throttle)
            if throttle <= 0.0:
                brake = throttle * -1 * 10000
                throttle = 0.0
        else:
            self.linear_velocity_pid.reset()
            self.low_pass_filter.reset()
            throttle = 0.0
            brake = MAX_BRAKE_VALUE

        brake = min(brake, MAX_BRAKE_VALUE)

        #rospy.logerr('target {: f}, current {: f}, throttle {: f}, brake: {: f}'.format(target_linear_velocity, current_linear_velocity, throttle, brake))
        #rospy.loginfo('target {: f}, current {: f}, throttle {: f}, brake: {: f}'.format(target_linear_velocity, current_linear_velocity, throttle, brake))

        #   normalized steering : -1/+1
        #   normalized steering = steer_angle * 2 / max_steer_angle
        #   steer_angle = wheel_angle * steer_ratio
        #   normalized steering = wheel_angle * { steer_ratio * 2 / max_steer_angle }
        #   steer_sensitivity = steer_ratio * 2 / max_steer_angle
        #
        #   normalized steerting = wheel_angle * steer_sensitivity

        steer = steer_sensitivity * self.yaw_controller.get_steering(target_linear_velocity, target_angular_velocity, current_linear_velocity)

        return throttle, brake, steer
