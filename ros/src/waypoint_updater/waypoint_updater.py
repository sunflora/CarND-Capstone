#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import tf

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

class WaypointUpdater(object):
    def __init__(self):
        # TODO: Add other member variables you need below
        self.final_lane = Lane()
        self.current_point = -1
        self.lane = None
        self.msg_count = 2
        self.dbw_enabled = False

        rospy.init_node('waypoint_updater')

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/vehicle/dbw_enabled',Bool, self.dbw_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        #rospy.logerr("Hello world this is a testing!")
        rospy.spin()

    def dbw_cb(self, msg):
        self.dbw_enabled = msg.data
        pass

    def pose_cb(self, msg):
        #rospy.logerr("In pose callback.")
        # TODO: Implement

        if not self.dbw_enabled:
            return

        if (self.msg_count > 0):
            #rospy.logerr("header: %s", msg.header)
            #rospy.logerr("current pose %s %s", msg.pose.position, msg.pose.orientation)
            roll, pitch, yaw = tf.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]) 
            #rospy.logerr("yaw is %s", yaw)            
            #self.msg_count = self.msg_count - 1

        if self.lane is not None:
            self.final_lane.header.frame_id = self.lane.header.frame_id
            self.final_lane.header.frame_id = self.lane.header.frame_id
            self.current_point = self.find_current_position(msg)
            self.set_final_waypoints(20)
            self.final_waypoints_pub.publish(self.final_lane)
            #if (self.msg_count > 0):
            #    rospy.logerr(self.final_lane)
            #    self.msg_count = self.msg_count -1
        pass

    def waypoints_cb(self,lane):
        self.lane = lane
        pass


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def set_waypoint_velocity2(self, waypoint, velocity):
        waypoint.twist.twist.linear.x = velocity

    def get_distance_2D(self, p1, p2):
        dx = p1.x - p2.x
        dy = p1.y - p2.y
        d = math.sqrt(dx*dx + dy*dy)
	return d

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def find_current_position(self,msg):
        if self.lane is not None:
            lane = self.lane
            nearest_distance = 99999999;
            index = 0
            nearest_index = 0
            for p in lane.waypoints:
                d = self.get_distance_2D(p.pose.pose.position, msg.pose.position)
                if d < nearest_distance:
                    nearest_distance = d
                    nearest_index = index
                index = index + 1
        #rospy.logerr("Nearest index found: %s", self.current_point)
        return nearest_index

    def set_final_waypoints(self, velocity):
        p = self.current_point
        self.final_lane.waypoints = self.lane.waypoints[p:p+LOOKAHEAD_WPS]
        for i in self.final_lane.waypoints:
            self.set_waypoint_velocity2(i, velocity)
        

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
