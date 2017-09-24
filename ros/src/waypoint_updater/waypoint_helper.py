#!/usr/bin/env python

'''
LOG:
    2017.09.24  add map zoning to get a better performance on searching 
        the nearest waypoint
'''

import math
import tf
from map_zone import MapZone

from styx_msgs.msg import Lane, Waypoint

class WaypointHelper(object):
    def __init__(self, is_looped_waypoints = False):

        self.is_looped_waypoints = is_looped_waypoints

        pass

    def create_lookahead_lane(self, base_lane, begin_index, step, size, velocity):
        lane = Lane()
        lane.header.frame_id = base_lane.header.frame_id

        end_index = min( begin_index+size, len(base_lane.waypoints)-1)

        lane.waypoints = base_lane.waypoints[begin_index: end_index : step]

        if begin_index + size > end_index :
            end_index = begin_index + size - end_index
            lane.waypoints += base_lane.waypoints[0:end_index:step]

        for p in lane.waypoints:
            p.twist.twist.linear.x = velocity 
            p.twist.twist.linear.y = 0.0
            p.twist.twist.linear.z = 0.0
            p.twist.twist.angular.x = 0.0
            p.twist.twist.angular.y = 0.0
            p.twist.twist.angular.z = 0.0            
        return lane

    def find_nearest_index(self, current_position, base_waypoints, map_zone):

        nearest_distance = 99999999
        nearest_index = 0
        index = 0

        elems = map_zone.getZoneNeighborElements( current_position.x, current_position.y)

        if elems == []:
            elems = range(0, len(base_waypoints)-1)

        for index in elems:
            p = base_waypoints[index]
            d = self.get_distance_2D(p.pose.pose.position, current_position)
            if d < nearest_distance:
                nearest_distance = d
                nearest_index = index

        return nearest_index+1

    def get_distance_2D(self, p1, p2):
        dx = p1.x - p2.x
        dy = p1.y - p2.y
        d = math.sqrt(dx * dx + dy * dy)
        return d

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def set_waypoint_velocity2(self, waypoint, velocity):
        waypoint.twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

