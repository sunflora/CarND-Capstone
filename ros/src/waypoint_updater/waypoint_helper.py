import math
import tf

from styx_msgs.msg import Lane, Waypoint

class WaypointHelper(object):
    def __init__(self, *args, **kwargs):
        pass

    def create_lookahead_lane(self, base_lane, begin_index, size, velocity):
        lane = Lane()
        lane.header.frame_id = base_lane.header.frame_id
        lane.waypoints = base_lane.waypoints[begin_index: begin_index + size]
        for p in lane.waypoints:
            p.twist.twist.linear.x = velocity
        return lane

    def find_nearest_index(self, current_position, base_waypoints):
        nearest_distance = 99999999;
        nearest_index = 0
        index = 0
        for p in base_waypoints:
            d = self.get_distance_2D(p.pose.pose.position, current_position)
            if d < nearest_distance:
                nearest_distance = d
                nearest_index = index
            index = index + 1
        return nearest_index

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

