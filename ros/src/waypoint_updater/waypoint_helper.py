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

WPS_FOR_BRAKE_AREA = 3
WPS_FOR_SLOW_DOWN_AREA = 20



class WaypointHelper(object):
    def __init__(self, is_looped_waypoints = False):

        self.is_looped_waypoints = is_looped_waypoints

        pass

    def create_lookahead_lane(self, base_lane, begin_index, stop_line_wpt, step, size, velocity):
        lane = Lane()
        lane.header.frame_id = base_lane.header.frame_id

        end_index = min( begin_index+size, len(base_lane.waypoints)-1)

        lane.waypoints = base_lane.waypoints[begin_index: end_index : step]

        indx_stop = 0        
        indx_brake = 0
        indx_slow_down = 0

        #adjust the slow down area according to its velocity
        global WPS_FOR_SLOW_DOWN_AREA
        WPS_FOR_SLOW_DOWN_AREA += int(min(velocity-4.47, 0.0 )*2)

        if stop_line_wpt != -1:   #Red Light Stop Line is Active
            if stop_line_wpt > begin_index:  # Stop Line is Ahead
                indx_stop = stop_line_wpt - begin_index
                indx_brake = indx_stop - WPS_FOR_BRAKE_AREA
                indx_slow_down = indx_stop - WPS_FOR_SLOW_DOWN_AREA

        if begin_index + size > end_index :
            end_index = begin_index + size - end_index
            lane.waypoints += base_lane.waypoints[0:end_index:step]

        indx = 0
        for p in lane.waypoints:
            if indx > indx_stop:
                v = velocity
            elif indx > indx_brake:
                v = 0.0
            elif indx > indx_slow_down:
                v = 4.0  #velocity * 0.5
            else:
                v = velocity    

            p.twist.twist.linear.x = v
            p.twist.twist.linear.y = 0.0
            p.twist.twist.linear.z = 0.0
            p.twist.twist.angular.x = 0.0
            p.twist.twist.angular.y = 0.0
            p.twist.twist.angular.z = 0.0
            indx += 1            
        return lane

    def find_nearest_index(self, current_position, base_waypoints, map_zone):

        nearest_distance = 99999999.9
        nearest_index = 0
        index = 0

        elems = map_zone.getZoneNeighborElements( current_position.x, current_position.y)

        if elems == []:
            elems = range(0, len(base_waypoints)-1)

        for index in elems:
            p = base_waypoints[index]
            d = self.get_distance_2D_sq(p.pose.pose.position, current_position)
            if d < nearest_distance:
                nearest_distance = d
                nearest_index = index

        return nearest_index+1

    def get_distance_2D(self, p1, p2):
        dx = p1.x - p2.x
        dy = p1.y - p2.y
        d = math.sqrt(dx * dx + dy * dy)
        return d

    def get_distance_2D_sq(self, p1, p2):
        dx = p1.x - p2.x
        dy = p1.y - p2.y
        d = dx * dx + dy * dy
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

