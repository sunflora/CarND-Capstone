'''
Path Planner finds smooth curve for driving based on the final_waypoints using CubicSpline.

Reference:
    This code was ported from the Udacity Path Planning Project:
        https://github.com/udacity/CarND-Path-Planning-Project

    Twist Commands related subroutines inspired by pure_pursuit module developed by Nagoya University


TODO:
 This module includes a function to get the next waypoints.  This needs to be separated later.
 ROS integration has not been tested.
 Twist Command processing is not finished.

'''

import math
import numpy as np
from scipy import interpolate

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from styx_msgs.msg import Lane, Waypoint
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped

import tf


RADIUS_MAX = 9.0e10
KAPPA_MIN  = 1.0/RADIUS_MAX

LOOKAHEAD_WPS = 30  # 90 meters 30 segments


class PathPlanner(object):
    def __init__(self):
        self.waypoints = None
        self.current_pose = None
        self.current_speed = None
        self.yaw = None

        self.frenet_coordinate = None
        self.final_waypoints = None

        self.prev_angular_velocity = 0

        rospy.init_node('path_planner')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size = 1)
        rospy.Subscriber('/final_waypoints', Lane, self.final_waypoints_cb, queue_size = 1)
        rospy.Subscriber('/current_velocity',TwistStamped, self.current_velocity_cb, queue_size = 1)

        self.twist_cmd_pub = rospy.Publisher('twist_cmd', TwistStamped, queue_size=1)

        rospy.spin()

    def pose_cb(self, msg):
        self.current_pose = msg.pose

        # set the current yaw
        orientation = msg.pose.orientation
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        _,_,self.yaw = tf.transformations.euler_from_quaternion(q)

        pass

    def final_waypoints_cb(self, msg):
        self.final_waypoints = msg.waypoints

        cw_x = []
        cw_y = []

        for wp in self.final_waypoints:
            cw_x.append(wp.pose.pose.position.x)
            cw_y.append(wp.pose.pose.position.y)

        car_x = self.current_pose.pose.position.x
        car_y = self.current_pose.pose.position.y

        cw_x, cw_y = self.path_planning(LOOKAHEAD_WPS, car_x, car_y, self.yaw, self.current_speed, cw_x, cw_y)

        pass

    def calcCurvature( self, target): #geometry_msgs::Point

        denominator = math.pow(getPlaneDistance(target, self.current_pose.pose.position), 2.0)
        numerator = 2.0 * calcRelativeCoordinate(target, self.current_pose.pose).y

        if denominator != 0:
            kappa = numerator / denominator;
        else:
            if (numerator > 0):
                kappa = KAPPA_MIN
            else:
                kappa = -KAPPA_MIN

        rospy.ROS_INFO_STREAM("kappa :", kappa)

        return kappa

    def calcTwist(self, curvature, cmd_velocity):

        #TODO:verify whether vehicle is following the path
        # following_flag = verifyFollowing();
        following_flag = True

        twist = Twist()
        twist.linear.x = cmd_velocity

        if following_flag:
            twist.angular.z = self.prev_angular_velocity
        else:
            twist.angular.z = self.current_speed * curvature

        self.prev_angular_velocity = twist.angular.z

        return twist

    def outputTwist( self, twist):
        g_lateral_accel_limit = 0.8
        ERROR = 1e-8

        twistCmd = TwistStamped()
        twistCmd.twist = twist
        twistCmd.header.stamp = rospy.Time.now()

        v = twist.linear.x
        omega = twist.angular.z

        if math.fabs(omega) < ERROR:
            return twistCmd

        max_v = g_lateral_accel_limit / omega

        a = v * omega

        rospy.ROS_INFO("lateral accel = %lf", a)

        twistCmd.twist.linear.x = max_v if (math.fabs(a) > g_lateral_accel_limit) else v
        twistCmd.twist.angular.z = omega

        return twistCmd

    def publish(self, twist_cmd):

        self.twist_cmd_pub.publish(twist_cmd)
        pass

    def current_velocity_cb(self, msg):
        self.current_speed = msg.twist.linear.x
        pass


    def path_planning(self, waypoints_size, car_x, car_y, theta, car_speed, maps_x, maps_y):

        # find the next N points

        # Main car's localization Data
        car_s, car_d = self.getFrenet(car_x, car_y, theta, maps_x, maps_y)
        maps_s, maps_d = self.getMapsS(maps_x, maps_y)

        '''
        testX, testY = self.getXY(0.,0, maps_s, maps_x, maps_y)
        testX, testY = self.getXY(30,0, maps_s, maps_x, maps_y)
        testX, testY = self.getXY(60,0, maps_s, maps_x, maps_y)
        testX, testY = self.getXY(90,0, maps_s, maps_x, maps_y)
        '''

        map_waypoints_s = maps_s
        map_waypoints_x = maps_x
        map_waypoints_y = maps_y


        ref_vel = min(car_speed,10.0)  #limit to 10 miles/hr #TODO: multiply by factor??

        # Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
        # Later we will interpolate these waypoints with spline and fill it with
        # more points that control spline
        ptsx = []
        ptsy = []

        # reference x,y,yaw states
        # either we will reference the starting point as where the car is or at the previous path's end point
        ref_x = car_x
        ref_y = car_y
        ref_yaw = math.radians(theta)

        # Use two points that make the path tangent to the car
        prev_car_x = car_x - math.cos(ref_yaw)          # This is the same as math.cos(theta) * 1
        prev_car_y = car_y - math.sin(ref_yaw)

        ptsx.append(prev_car_x)
        ptsx.append(car_x)

        ptsy.append(prev_car_y)
        ptsy.append(car_y)

        # In Frenet add evenly 30m spaced points ahead of the starting reference
        lane = -1   # left lane: +1, right lane: -1
        road_width = 4
        middle_of_lane = 0

        next_wp0 = self.getXY(car_s + 30, middle_of_lane + road_width * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y)
        next_wp1 = self.getXY(car_s + 60, middle_of_lane + road_width * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y)
        next_wp2 = self.getXY(car_s + 90, middle_of_lane + road_width * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y)

        ptsx.append(next_wp0[0])
        ptsx.append(next_wp1[0])
        ptsx.append(next_wp2[0])

        ptsy.append(next_wp0[1])
        ptsy.append(next_wp1[1])
        ptsy.append(next_wp2[1])

        for i in range(0, len(ptsx)):
            # shift car reference angle to 0 degrees
            # convert into local car coordinate
            shift_x = ptsx[i] - ref_x
            shift_y = ptsy[i] - ref_y

            ptsx[i] = (shift_x * math.cos(0 - ref_yaw) - shift_y * math.sin(0 - ref_yaw))
            ptsy[i] = (shift_x * math.sin(0 - ref_yaw) + shift_y * math.cos(0 - ref_yaw))

        # create a cubic spline
        # set(x,y) points to the spline

        # Define the actual (x,y) points we will use for the planner
        next_x_vals = []
        next_y_vals = []


        # Calculate how to breakup spline points so that we travel at our desired reference velocity

        spline_s = [-1, 0, 30, 60, 90]

        pts = []
        for i in range(0, len(ptsx)):
            pts.append([ptsx[i],ptsy[i]])
        cs = interpolate.CubicSpline(spline_s, pts)

        # break the spline into 30 segments
        xs = np.linspace(0, 90, 30)
        #plt.figure(figsize = (6.5, 4))
        #plt.plot(cs(xs)[:, 0], cs(xs)[:, 1], label='spline')
        #plt.plot(ptsx, ptsy)

        spoints = cs(xs)

        # Fill out the rest of our path planner after filling it with previous points, here we will always output waypoints_size points
        for i in range(1, len(xs)):
            #N = (target_dist / (0.02 * ref_vel / 2.24))
            x_point = spoints[i][0]
            y_point = spoints[i][1]

            x_ref = x_point
            y_ref = y_point

            # rotate back to normal after rotating it earlier
            x_point = (x_ref * math.cos(ref_yaw) - y_ref * math.sin(ref_yaw))
            y_point = (x_ref * math.sin(ref_yaw) + y_ref * math.cos(ref_yaw))

            x_point += ref_x
            y_point += ref_y

            next_x_vals.append(x_point)
            next_y_vals.append(y_point)


        return next_x_vals, next_y_vals

    def getMapsS(self, maps_x, maps_y):
        # origin (s,d)
        maps_s = [0.0]
        maps_d = [0.0]
        map_s_accu = 0.0
        for i in range(1, len(maps_x)):
            #TODO: delete map_s, map_d = self.getFrenet(maps_x[i], maps_y[i], theta, maps_x, maps_y)
            map_s_accu = map_s_accu + self.distance( maps_x[i-1],maps_y[i-1],maps_x[i],maps_y[i])
            maps_s.append(map_s_accu)
            maps_d.append(0.0)
        return maps_s, maps_d

    # double distance(double x1, double y1, double x2, double y2)
    def distance(self, x1, y1, x2, y2):
        d = math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1))
        return d


    # int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
    def ClosestWaypoint(self, x, y, maps_x, maps_y):

        closestLen = 100000;  # large number
        closestWaypoint = -1; # -1 if point not found

        for i in range(len(maps_x)-1, 0, -1):
            map_x = maps_x[i]
            map_y = maps_y[i]
            dist = self.distance(x, y, map_x, map_y);
            if (dist < closestLen):
                closestLen = dist
                closestWaypoint = i

        return closestWaypoint;

    # int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
    def NextWaypoint(self, x, y, theta, maps_x, maps_y):
        closestWaypoint = self.ClosestWaypoint(x, y, maps_x, maps_y);
        map_x = maps_x[closestWaypoint]
        map_y = maps_y[closestWaypoint]

        heading = math.atan2((map_y - y), (map_x - x))
        angle = abs(theta - heading)

        if (angle > math.pi / 4):
            closestWaypoint = closestWaypoint + 1

        return closestWaypoint

    def getNextWaypoints(self, x, y, theta, wp_size, maps_x, maps_y):
        nextwaypoint = self.NextWaypoint(x, y, theta, maps_x, maps_y)
        #### TODO: Assuming this is a loop without overlapping waypoints!
        nextWaypoints_x = []
        nextWaypoints_y = []

        endpoint = min(nextwaypoint+wp_size, len(maps_x))
        nextWaypoints_x = maps_x[nextwaypoint:endpoint]
        nextWaypoints_y = maps_y[nextwaypoint:endpoint]
        if endpoint == len(maps_x):
            nextWaypoints_x = nextWaypoints_x + maps_x[0:wp_size - len(nextWaypoints_x)]
            nextWaypoints_y = nextWaypoints_y + maps_y[0:wp_size - len(nextWaypoints_y)]

        return nextWaypoints_x, nextWaypoints_y


    # If the course is a loop and there are overlapping waypoints, remove the duplicate points so the car may drive continuously
    def cleanseWaypoints(self, is_loop, maps_x, maps_y, start_duplicate = -1):
        if is_loop is not True:
            return

        if start_duplicate == -1:
            #find start duplicate
            search_size = 200  #number of points to search
            map_size = len(maps_x)
            begin_search = map_size - search_size
            start_duplicate = self.ClosestWaypoint(maps_x[0], maps_y[0], maps_x[begin_search:map_size], maps_y[begin_search:map_size])
            start_duplicate = begin_search + start_duplicate
            print("origin point: ", maps_x[0], maps_y[0])
            print("duplicate point:", maps_x[start_duplicate], maps_y[start_duplicate])
            print("start_duplicate found at index: ", start_duplicate)
        return maps_x[0:start_duplicate - 1], maps_y[0:start_duplicate - 1]

    # Transform from Cartesian x,y coordinates to Frenet s,d coordinates
    # vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
    def getFrenet(self, x, y, theta, maps_x, maps_y):

        next_wp = self.NextWaypoint(x, y, theta, maps_x, maps_y)

        prev_wp = next_wp - 1
        if (next_wp == 0):
            prev_wp = maps_x.size() - 1

        if next_wp >= len(maps_x):
            pass

        try:
            n_x = maps_x[next_wp] - maps_x[prev_wp]
            n_y = maps_y[next_wp] - maps_y[prev_wp]
            x_x = x - maps_x[prev_wp]
            x_y = y - maps_y[prev_wp]
        except:
            print("next_wp",next_wp)
            pass


        # find the projection of x onto n
        proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y)
        proj_x = proj_norm * n_x
        proj_y = proj_norm * n_y

        frenet_d = self.distance(x_x, x_y, proj_x, proj_y)

        # see if d value is positive or negative by comparing it to a center point

        center_x = 1000 - maps_x[prev_wp]
        center_y = 2000 - maps_y[prev_wp]
        centerToPos = self.distance(center_x, center_y, x_x, x_y)
        centerToRef = self.distance(center_x, center_y, proj_x, proj_y)

        if (centerToPos <= centerToRef):
            frenet_d *= -1;

        # calculate s value
        frenet_s = 0;
        for i in range(0, prev_wp):
            frenet_s += self.distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1])

        frenet_s += self.distance(0, 0, proj_x, proj_y);

        return frenet_s, frenet_d


    # Transform from Frenet s,d coordinates to Cartesian x,y
    # vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
    def getXY(self, s, d, maps_s, maps_x, maps_y):


        prev_wp = -1

        # TODO: Currently, we assume there exists at least one point on maps_s
        for i in range(0, len(maps_s)-1):
            if s >= maps_s[i]:
                prev_wp = i
            else:
                break

        if prev_wp == -1:
            return maps_x[0], maps_y[0]

        wp2 = (prev_wp + 1) % len(maps_x)

        heading = math.atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]))

        # the x,y,s along the segment
        seg_s = (s - maps_s[prev_wp])

        seg_x = maps_x[prev_wp] + seg_s * math.cos(heading)
        seg_y = maps_y[prev_wp] + seg_s * math.sin(heading)

        perp_heading = heading - math.pi / 2

        x = seg_x + d * math.cos(perp_heading)
        y = seg_y + d * math.sin(perp_heading)

        return x, y
