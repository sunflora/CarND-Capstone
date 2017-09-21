#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from waypoint_helper import WaypointHelper

'''
This node will publish base_lane from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 100 # Number of base_lane we will publish. You can change this number
MPH_2_mps = 0.44704
TARGET_VELOCITY_MPH = 10 * MPH_2_mps
BRAKE_DISTANCE = 10

class WaypointUpdater(object):
    def __init__(self):

        self.final_lane = Lane()
        self.final_lane_step = 2
        self.nearest_wy_indx = -1
        self.base_lane = None
        self.base_waypoints = None
        self.dbw_enabled = False
        self.max_velocity = TARGET_VELOCITY_MPH
        self.traffic_waypoint = None
        self.current_position = None

        self.wph = WaypointHelper()

        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size = 1)
        rospy.Subscriber('/base_waypoints', Lane, self.base_waypoints_cb, queue_size = 1)
        rospy.Subscriber('/vehicle/dbw_enabled',Bool, self.dbw_cb, queue_size = 1)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_waypoint_cb, queue_size = 1)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.loop()

    def loop(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():


            self.traffic_waypoint = -1


            #rospy.logerr("WPUpdater in loop")
         
            if self.base_waypoints is not None and self.current_position is not None:
                self.nearest_wy_indx = self.wph.find_nearest_index(self.current_position, self.base_waypoints)

            #rospy.logerr("WPUpdater, pose_cb: light_wy: %s car_wy:%s  ", self.traffic_waypoint, self.nearest_wy_indx)

            okay2run = self.dbw_enabled and self.base_waypoints is not None and self.traffic_waypoint is not None and self.nearest_wy_indx is not None
            if okay2run:
                
                if self.traffic_waypoint == -1:
                    self.max_velocity = TARGET_VELOCITY_MPH
                    #rospy.logerr("WPUpdater, pose_cb: light_wy: %s car_wy:%s  ", self.traffic_waypoint, self.nearest_wy_indx)
                else:
                    if 0 < (self.traffic_waypoint - self.nearest_wy_indx) < BRAKE_DISTANCE:
                        rospy.logerr("WPUpdater, pose_cb: Redlight Alert!  light_wy: %s car_wy:%s  ",self.traffic_waypoint, self.nearest_wy_indx)
                        self.max_velocity = 0
                    else:
                        self.max_velocity = TARGET_VELOCITY_MPH

                self.final_lane = self.wph.create_lookahead_lane(self.base_lane, self.nearest_wy_indx, self.final_lane_step, LOOKAHEAD_WPS, self.max_velocity)
                #rospy.logerr("WPUpdater, pose_cb: car_wy:%s  ", self.nearest_wy_indx)
                #for i in range(0, len(self.final_lane.waypoints)):
                #    rospy.logerr("%i: %f, %f", i, self.final_lane.waypoints[i].pose.pose.position.x, self.final_lane.waypoints[i].pose.pose.position.y)

                self.final_waypoints_pub.publish(self.final_lane)
            
            rate.sleep() 

    def traffic_waypoint_cb(self, msg):

        if self.traffic_waypoint != msg.data:
            self.traffic_waypoint = msg.data
            #rospy.logerr("WPUpdater.traffic_waypoint_cb, light_wy is %s", self.traffic_waypoint )
        pass

    def dbw_cb(self, msg):
        self.dbw_enabled = msg.data
        pass

    def pose_cb(self, msg):
        '''
        Callback for getting the current pose of the car.
        '''


        self.current_position = msg.pose.position

        pass

    def base_waypoints_cb(self, msg):
        self.base_lane = msg
        self.base_waypoints = msg.waypoints
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
