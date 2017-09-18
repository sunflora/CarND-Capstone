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


class TwistController(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.linear_velocity_pid = PID(kp=1.0,ki=0.1,kd=0.5,mn=LINEAR_PID_MIN,mx=LINEAR_PID_MAX)
        self.low_pass_filter = LowPassFilter(8.0, 2.0)  #(b,a) normalized

        #self.angular_velocity_pid = PID(kp=6.0,ki=0.1,kd=0.5,mn=ANGULAR_PID_MIN,mx=ANGULAR_PID_MAX)
        self.yaw_controller = YawController()

        pass

    def control(self, current_linear_velocity, target_linear_velocity, steer_sensitivity, target_angular_velocity):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        sample_time = 0.1
        cte_linear = target_linear_velocity - current_linear_velocity
        throttle = self.linear_velocity_pid.step(cte_linear, sample_time)
        #rospy.logerr("target: %s and current: %s", target_linear_velocity, current_linear_velocity)
        #rospy.logerr("========  cte: %s, throttle: %s", cte_linear, throttle)

        '''
        cte_angular = target_angular_velocity - current_angular_velocity
        steering = self.angular_velocity_pid.step(cte_angular, sample_time)
        #rospy.logerr("target: %s and current: %s", target_angular_velocity, current_angular_velocity)
        #rospy.logerr("========  cte: %s, steering: %s", cte_angular, steering)
        '''
        
        throttle = self.low_pass_filter.filt(throttle)

        brake = 0.0
        if throttle <= 0.0:
            brake = throttle * -1 * 5
            throttle = 0.0

        steer = self.yaw_controller.get_steering(steer_sensitivity, target_angular_velocity, current_linear_velocity)

        return throttle, brake, steer
