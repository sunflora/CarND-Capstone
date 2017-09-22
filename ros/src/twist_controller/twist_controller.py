from pid import PID
import rospy
from std_msgs.msg import Bool
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
LINEAR_PID_MIN = -1.0
LINEAR_PID_MAX = 1.0
ANGULAR_PID_MIN = -0.4
ANGULAR_PID_MAX = 0.35
ACCEL_SENSITIVITY = 0.06

SAMPLE_TIME = 0.5 # sample time for PID

class TwistController(object):
    def __init__(self):
        self.linear_velocity_pid = PID(kp=ACCEL_SENSITIVITY*1.25,ki=0.003,kd=0.0,mn=LINEAR_PID_MIN,mx=LINEAR_PID_MAX)
        self.low_pass_filter = LowPassFilter(8.0, 2.0)  #(b,a) normalized, (0 , 1) = all pass
        self.yaw_controller = None
        pass

    def control(self, current_linear_velocity, target_linear_velocity, steer_sensitivity, target_angular_velocity):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        sample_time = SAMPLE_TIME
        cte_linear = target_linear_velocity - current_linear_velocity
        throttle = self.linear_velocity_pid.step(cte_linear, sample_time)
        
        throttle = self.low_pass_filter.filt(throttle)

        brake = 0.0
        if throttle <= 0:
            brake = throttle * -1 * 1000
            throttle = 0.0

        #   normalized steering : -1/+1
        #   normalized steering = steer_angle * 2 / max_steer_angle
        #   steer_angle = wheel_angle * steer_ratio
        #   normalized steering = wheel_angle * { steer_ratio * 2 / max_steer_angle }
        #   steer_sensitivity = steer_ratio * 2 / max_steer_angle
        #
        #   normalized steerting = wheel_angle * steer_sensitivity

        steer = steer_sensitivity * self.yaw_controller.get_steering(target_linear_velocity, target_angular_velocity, current_linear_velocity)

        return throttle, brake, steer
