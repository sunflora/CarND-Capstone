#!/usr/bin/env python

'''

TLDetector for getting traffic light info via ground truth

/vehicle/traffic_lights helps you acquire an accurate ground truth data source for the traffic light
classifier, providing the location and current color state of all traffic lights in the
simulator. This state can be used to generate classified images or subbed into your solution to
help you work on another single component of the node. This topic won't be available when
testing your solution in real life so don't rely on it in the final submission.

LOG:
    2017.09.24  add threading.Lock and copy.deepcopy for data consistency
                add MapZone

'''


import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import Lane
from std_msgs.msg import Bool
from sensor_msgs.msg import Image

import operator

import threading
import copy

from styx_msgs.msg import TrafficLightArray, TrafficLight

from tl_helper import TLHelper

import sys
import os

sys.path.insert(0, os.path.realpath('./../waypoint_updater/'))
from map_zone import MapZone

#STATE_COUNT_THRESHOLD = 3
STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self, *args, **kwargs):
        rospy.init_node('tl_detector')
        #rospy.logerr("Inside TLDetectorGT")

        self.tlh = TLHelper()

        self.map_zone = MapZone() 

        self.base_waypoints = None

        self.pose_position = None
        self.pose_nearest_waypoint = None

        self.lights = None
        self.lights_waypoints = None
        self.lights_waypoints_list = None  #sorted list


        self.pose_lock = threading.Lock()
        self.base_waypoints_lock = threading.Lock()
        self.traffic_lock = threading.Lock()

        self.msg_pose = None
        self.msg_base_waypoints = None
        self.msg_traffic = None

        self.base_waypoints_updated = False

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

        #self.traffic_light_waypoints = []

        #rospy.spin()
        self.loop()

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
 
            if self.msg_pose is None or self.msg_base_waypoints is None or self.msg_traffic is None:
                continue

            self.set_pose()

            if self.base_waypoints_updated:
                self.set_base_waypoints()
                self.lights_waypoints = {}
                self.base_waypoints_updated = False

            self.set_traffic_lights()

            # cache traffic light position (x,y) and vehicle stop line waypoint 
            self.update_traffic_light_waypoints()

            light_wp, state = self.get_next_traffic_light_wp()

            self.process_traffic_lights( light_wp, state)

    def getKey( self, x, y):
        return int(round(x)*10000+round(y))

    def find_traffic_light_stop_wpt( self, wpt, dist=26.0):
        """ go back the waypoints to find the vehicle stop line waypoint"""    
        max_wpt = len(self.base_waypoints)-1
        
        _wpt = wpt 
        pos = self.base_waypoints[wpt].pose.pose.position

        while dist > 0.0:
            prv_wpt = wpt - 1 if wpt>0 else max_wpt

            prv_pos = self.base_waypoints[prv_wpt].pose.pose.position

            dist -= self.tlh.get_distance_2D( pos, prv_pos)

            wpt = prv_wpt
            pos = prv_pos

        #rospy.logerr('tfl:{} stop:{}'.format(_wpt, prv_wpt))

        return prv_wpt

    def update_traffic_light_waypoints(self):
        """  update the traffic light waypoints information """
        sorting = False
        for lgt in self.lights:
            light_position = lgt.pose.pose.position

            key = self.getKey( light_position.x, light_position.y)
 
            if not self.lights_waypoints.has_key(key):

                wpt = self.tlh.get_nearest_waypoint(light_position, self.base_waypoints, self.map_zone)
                
                wpt = self.find_traffic_light_stop_wpt( wpt)

                self.lights_waypoints[key] = [light_position.x, light_position.y, wpt]
                sorting = True

        if sorting:
            self.lights_waypoints_list = sorted( self.lights_waypoints.iteritems(), key=lambda(k,v): operator.itemgetter(2)(v) )
            #rospy.logerr( self.lights_waypoints_list)

    def pose_cb(self, msg):
        self.pose_lock.acquire()
        self.msg_pose = msg.pose
        self.pose_lock.release()

    def set_pose(self):
        self.pose_lock.acquire()
        self.pose_position = copy.deepcopy(self.msg_pose.position) 
        self.pose_lock.release()

    def waypoints_cb(self, msg):
        self.base_waypoints_lock.acquire()
        self.msg_base_waypoints = msg
        self.base_waypoints_lock.release()

        self.base_waypoints_updated = True

    def set_base_waypoints(self):
        self.base_waypoints_lock.acquire()
        self.base_waypoints = copy.deepcopy(self.msg_base_waypoints.waypoints)
        self.base_waypoints_lock.release()
        
        i=0
        for p in self.base_waypoints:
            self.map_zone.addElement( i, p.pose.pose.position.x, p.pose.pose.position.y)
            i += 1

    def traffic_cb(self, msg):
        self.traffic_lock.acquire()
        self.msg_traffic = msg.lights
        self.traffic_lock.release()
        
    def set_traffic_lights(self):
        self.traffic_lock.acquire()
        self.lights = self.msg_traffic
        self.traffic_lock.release()


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
        okay2run = self.msg_pose is not None and self.msg_base_waypoints is not None and self.lights_waypoints is not None
        if not okay2run:
            return

        #TODO: self.process_traffic_lights()

        return


    def process_traffic_lights(self, light_wp, state):

        okay2run = self.msg_pose is not None and self.msg_base_waypoints is not None and self.lights_waypoints is not None
        if not okay2run:
            return

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
                self.active_wp = light_wp if state != TrafficLight.GREEN else -1
            #rospy.logerr('lght:{} act:{}'.format( light_wp, self.active_wp))    
            self.upcoming_red_light_pub.publish(Int32(self.active_wp))
            
        #rospy.logerr('st:{}\tcnt:{}\twp:{}'.format( self.state, self.state_count, self.active_wp))

        self.state_count += 1

    def get_next_traffic_light_wp(self):
        """
        Finds closest visible traffic light, if one exists, and determines its location and color

        Returns:
            int: index of waypoint closes to the upcoming traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """

        self.pose_nearest_waypoint = self.tlh.get_nearest_waypoint(self.pose_position, self.base_waypoints, self.map_zone)

        i = self.get_nearest_light_wp( self.pose_nearest_waypoint, self.lights_waypoints_list)

        self.light_waypoint = self.lights_waypoints[i][2]
        #rospy.logerr("pose_n_wp:{}, i:{}, light_waypoint:{}".format(self.pose_nearest_waypoint, i, self.light_waypoint))

        ## TODO: Note Assuming all traffic lights have the same state
        state = self.get_light_state(self.lights[0])

        #rospy.logerr("TLDetectorGT.get_traffic_light_wp: car-wp: %s tl-wp:%s state:%s", self.pose_nearest_waypoint, self.light_waypoint, state)
        return self.light_waypoint, state

    #TODO: Assuming loop waypoints condition
    def get_nearest_light_wp(self, current_wp, lights_wps_list):
        min_wpt = 1000000
        min_wpt_key = None
        for lgt_wp in lights_wps_list:
            wpt = lgt_wp[1][2]
            if wpt < min_wpt:
                min_wpt = wpt
                min_wpt_key = lgt_wp[0]
            if wpt >= current_wp:
                return lgt_wp[0]
        return min_wpt_key

    def get_nearest_light_wp_(self, current_wp, lights_wps):
        min_wpt = 1000000
        min_wpt_key = None
        for key in lights_wps:
            wpt = lights_wps[key][2]
            if wpt < min_wpt:
                min_wpt = wpt
                min_wpt_key = key
            if wpt >= current_wp:
                return key
        return min_wpt_key

    def get_light_state(self, light):
        #rospy.logerr("TLDetectorGT.get_light_state: %s", light.state)
        return light.state

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
