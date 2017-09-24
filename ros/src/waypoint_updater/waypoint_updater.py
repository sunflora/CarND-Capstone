#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from waypoint_helper import WaypointHelper
from map_zone import MapZone

import random
import threading
import copy

'''
This node will publish base_lane from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.

LOG:
    2017.09.24  add threading.Lock and copy.deepcopy for data consistency
                add MapZone

'''

LOOKAHEAD_WPS = 100 # Number of base_lane we will publish. You can change this number
MPH_2_mps = 0.44704
TARGET_VELOCITY_MPS = 10 
BRAKE_LIMIT_LOWER = 15
BRAKE_LIMIT_UPPER = 40

class WaypointUpdater(object):
    def __init__(self):

        self.final_lane = Lane()
        self.final_lane_step = 2
        self.nearest_wy_indx = -1
        self.base_lane = None
        self.base_waypoints = None
        self.dbw_enabled = False
        self.max_velocity = TARGET_VELOCITY_MPS
        self.traffic_waypoint = None
        self.current_position = None

        self.temp_array = [500, 1000, 1500, 2000, 2500, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000]


        self.wph = WaypointHelper()

        self.map_zone = MapZone() 

        rospy.init_node('waypoint_updater')

        self.pose_lock = threading.Lock()
        self.base_waypoints_lock = threading.Lock()
        self.traffic_waypoint_lock = threading.Lock()

        self.msg_pose = None
        self.msg_base_waypoints = None
        self.msg_traffic_waypoint = None

        self.base_waypoints_updated = False

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size = 1)
        rospy.Subscriber('/base_waypoints', Lane, self.base_waypoints_cb, queue_size = 1)
        rospy.Subscriber('/vehicle/dbw_enabled',Bool, self.dbw_cb, queue_size = 1)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_waypoint_cb, queue_size = 1)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.loop()

    def get_traffic_waypoint(self, current_wp):
        for i in range(0, len(self.temp_array)):
            if self.temp_array[i] > current_wp:
                return self.temp_array[i]
        return self.temp_array[0]


    def loop(self):
        rate = rospy.Rate(4.0)
        while not rospy.is_shutdown():
            rate.sleep()

            if self.msg_pose is None or self.msg_base_waypoints is None or self.msg_traffic_waypoint is None:
                continue

            self.set_pose()

            if self.base_waypoints_updated:
                self.set_base_waypoints()
                self.base_waypoints_updated = False

            self.set_traffic_waypoint()

            self.nearest_wy_indx = self.wph.find_nearest_index(self.current_position, self.base_waypoints, self.map_zone)

            #rospy.logerr("WPUpdater, pose_cb: light_wy: %s car_wy:%s  ", self.traffic_waypoint, self.nearest_wy_indx)

            #self.traffic_waypoint = -1
 
            okay2run = self.dbw_enabled and self.nearest_wy_indx is not None
            if okay2run:
                
                tfwp = self.traffic_waypoint

                # Test code
                #tfwp = self.get_traffic_waypoint(self.nearest_wy_indx)
                #self.traffic_waypoint = tfwp if random.random() > 0.2 else -1
                
                # rospy.logerr("wpu.loop(), current_x {}  self.traffic_wayp {}    tfwp is:{}".format(self.nearest_wy_indx, self.traffic_waypoint, tfwp))

                # Test code
                '''
                x = self.current_position.x
                if (x > 2000.0) or (x < 500.0):
                    self.max_velocity = 15.0
                else:
                    self.max_velocity = 20.0
                '''
                self.max_velocity = TARGET_VELOCITY_MPS
                if self.traffic_waypoint != -1:
                    if  BRAKE_LIMIT_LOWER < (self.traffic_waypoint - self.nearest_wy_indx) < BRAKE_LIMIT_UPPER:
                        #rospy.logerr("WPUpdater, loop: Redlight Alert!  light_wy: %s car_wy:%s  ",self.traffic_waypoint, self.nearest_wy_indx)
                        self.max_velocity = 0

                self.final_lane = self.wph.create_lookahead_lane(self.base_lane, self.nearest_wy_indx, self.final_lane_step, LOOKAHEAD_WPS, self.max_velocity)
                #for i in range(0, len(self.final_lane.waypoints)):
                #    rospy.logerr("%i: %f, %f", i, self.final_lane.waypoints[i].pose.pose.position.x, self.final_lane.waypoints[i].pose.pose.position.y)

                self.final_waypoints_pub.publish(self.final_lane) 

    def dbw_cb(self, msg):
        self.dbw_enabled = msg.data
        pass

    def pose_cb(self, msg):
        self.pose_lock.acquire()
        self.msg_pose = msg.pose
        self.pose_lock.release()

    def set_pose(self):
        self.pose_lock.acquire() #---------------- acquire -----------
        self.current_position = copy.deepcopy(self.msg_pose.position)
        self.pose_lock.release() #---------------- release -----------
        pass

    def base_waypoints_cb(self, msg):
        self.base_waypoints_lock.acquire()
        self.msg_base_waypoints = msg
        self.base_waypoints_lock.release()

        self.base_waypoints_updated = True

    def set_base_waypoints(self):    
        self.base_waypoints_lock.acquire()
        self.base_lane = copy.deepcopy(self.msg_base_waypoints)
        self.base_waypoints_lock.release()

        self.base_waypoints = self.base_lane.waypoints

        i=0
        for p in self.base_waypoints:
            self.map_zone.addElement( i, p.pose.pose.position.x, p.pose.pose.position.y)
            i += 1

        #rospy.logerr(len(self.map_zone.map_zone))

        pass

    def traffic_waypoint_cb(self, msg):
        self.traffic_waypoint_lock.acquire()
        self.msg_traffic_waypoint = msg.data
        self.traffic_waypoint_lock.release()   

    def set_traffic_waypoint(self):    
        self.traffic_waypoint_lock.acquire()
        if self.traffic_waypoint != self.msg_traffic_waypoint:
            self.traffic_waypoint = copy.deepcopy(self.msg_traffic_waypoint)
        self.traffic_waypoint_lock.release()

        #rospy.logerr("WPUpdater.traffic_waypoint_cb, light_wy is %s", self.traffic_waypoint )
        pass


    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
