#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from waypoint_helper import WaypointHelper

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 20 # Number of waypoints we will publish. You can change this number
MPH_2_mps = 0.44704
TARGET_VELOCITY_MPH = 10 * MPH_2_mps

class WaypointUpdater(object):
    def __init__(self):

        self.final_lane = Lane()
        self.nearest_wy_indx = -1
        self.base_lane = None
        self.base_waypoints = None
        self.dbw_enabled = False
        self.max_velocity = TARGET_VELOCITY_MPH

        self.wph = WaypointHelper()

        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.base_waypoints_cb)
        rospy.Subscriber('/vehicle/dbw_enabled',Bool, self.dbw_cb)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        rospy.spin()

    def dbw_cb(self, msg):
        self.dbw_enabled = msg.data
        pass

    def pose_cb(self, msg):
        '''
        Callback for getting the current pose of the car.
        '''

        okay2run = self.dbw_enabled and self.base_waypoints is not None
        if not okay2run:
            return

        current_position = msg.pose.position

        self.nearest_wy_indx = self.wph.find_nearest_index(current_position, self.base_waypoints)
        self.final_lane = self.wph.create_lookahead_lane(self.base_lane, self.nearest_wy_indx, LOOKAHEAD_WPS, self.max_velocity)

        self.final_waypoints_pub.publish(self.final_lane)

        pass

    def base_waypoints_cb(self, msg):
        self.base_lane = msg
        self.base_waypoints = msg.waypoints
        pass


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
