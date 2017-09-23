#!/usr/bin/env python

'''

TLDetector for getting traffic light info via ground truth

/vehicle/traffic_lights helps you acquire an accurate ground truth data source for the traffic light
classifier, providing the location and current color state of all traffic lights in the
simulator. This state can be used to generate classified images or subbed into your solution to
help you work on another single component of the node. This topic won't be available when
testing your solution in real life so don't rely on it in the final submission.

'''


import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import Lane
from std_msgs.msg import Bool
from sensor_msgs.msg import Image

from styx_msgs.msg import TrafficLightArray, TrafficLight

from tl_helper import TLHelper

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self, *args, **kwargs):
        rospy.init_node('tl_detector')
        rospy.logerr("Inside TLDetectorGT")

        self.tlh = TLHelper()

        self.base_lane = None
        self.base_waypoints = None

        self.pose = None
        self.pose_position = None
        self.pose_nearest_waypoint = None

        self.lights = None
        self.lights_waypoints = None


        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size = 1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size = 1)
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size = 1)
        rospy.Subscriber('/image_color',Image, self.image_color_cb, queue_size = 1)
        rospy.Subscriber('/image_signal',Bool, self.image_signal_cb, queue_size = 1)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.has_image = False
        self.state = TrafficLight.UNKNOWN
        self.active_state = TrafficLight.UNKNOWN
        self.active_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        #rospy.logerr("Inside TLDetectorGT.pose_cb")
        self.pose = msg
        self.pose_position = msg.pose.position

    def waypoints_cb(self, msg):
        #rospy.logerr("Inside TLDetectorGT.waypoints_cb")
        self.base_lane = msg
        self.base_waypoints = msg.waypoints


    def traffic_cb(self, msg):
        
        self.lights = msg.lights
        #print ("--- traffic_cb ---", self.lights)

        if self.base_waypoints is None:
            return

        #rospy.logerr("TLDetectorGT.traffic_cb: base_waypoint is not None")

        ## TODO: Note that here we assume light position won't change in the context of base_waypoint
        lights_waypoints = []
        for l in self.lights:
            light_position = l.pose.pose.position
            nearest_waypoint = self.tlh.get_nearest_waypoint(light_position, self.base_waypoints)
            lights_waypoints.append(nearest_waypoint)
        self.lights_waypoints = lights_waypoints

        self.process_traffic_lights()
        
        return

    def image_color_cb(self, msg):
        #rospy.logerr("Inside TLDetectorGT.image_cb")
        """
        Identifies red lights in the incoming camera image
        publishes the index of the waypoint closest to the red light to /traffic_waypoint
        Args:
            msg (Image): image from car-mounted camera
        """
        self.has_image = False
        self.camera_image = msg
        pass


    def image_signal_cb(self, msg):
        
        # okay2run attribute for calling process_traffic_lights()
        okay2run = self.pose is not None and self.base_waypoints is not None and self.lights_waypoints is not None
        if not okay2run:
            return

        self.process_traffic_lights()

        return


    def process_traffic_lights(self):

        light_wp, state = self.get_traffic_light_wp()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state = state
            self.state_count = 0
        
        if self.state_count != 0:
            if self.state_count >= STATE_COUNT_THRESHOLD: 
                self.active_state = self.state
                self.active_wp = light_wp if state == TrafficLight.RED else -1
            self.upcoming_red_light_pub.publish(Int32(self.active_wp))
            
        #rospy.logerr('st:{}\tcnt:{}\twp:{}'.format( self.state, self.state_count, self.active_wp))

        self.state_count += 1

    def get_traffic_light_wp(self):
        """
        Finds closest visible traffic light, if one exists, and determines its location and color

        Returns:
            int: index of waypoint closes to the upcoming traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """


        okay2run = self.pose is not None and self.base_waypoints is not None and self.lights_waypoints is not None
        if not okay2run:
            return

        '''
        for i in range(0, len(self.lights_waypoints)):
            rospy.logerr("lights_waypoints[%s]:%s",i, self.lights_waypoints[i])
        '''

        self.pose_nearest_waypoint = self.tlh.get_nearest_waypoint(self.pose_position, self.base_waypoints)
        self.light_waypoint = self.get_nearest_light_wp( self.pose_nearest_waypoint, self.lights_waypoints)

        ## TODO: Note Assuming all traffic lights have the same state
        state = self.get_light_state(self.lights[0])

        #rospy.logerr("TLDetectorGT.get_traffic_light_wp: car-wp: %s tl-wp:%s state:%s", self.pose_nearest_waypoint, self.light_waypoint, state)
        return self.light_waypoint, state

    def get_nearest_light_wp(self, current_wp, light_wps):
        for i in range(0, len(light_wps)):
            if light_wps[i] > current_wp:
                return light_wps[i]
        return lights_wp[0]

    def get_light_state(self, light):
        #rospy.logerr("TLDetectorGT.get_light_state: %s", light.state)
        return light.state

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
