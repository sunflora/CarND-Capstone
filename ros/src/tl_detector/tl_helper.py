import math

class TLHelper(object):

    def __init__(self, *args, **kwargs):
        pass

    def get_nearest_waypoint(self, current_position, base_waypoints):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.base_lane

        """
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
