#!/usr/bin/python3
import rospy
import math
# import yaml
from scipy.special import comb
from shapely.geometry import Point
from shapely.geometry import Polygon as SPoly
from shapely.strtree import STRtree
import numpy as np
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import Polygon, Point32, PoseStamped
import ast
import os


class TLBO():

    def __init__(self, start, end, bounds,num_of_learners=15,path_points=25,subjects=2):
        self.start = start
        self.end = end
        self.bounds = bounds
        self.bounds_polygon = None
        self.obstacles = None
        self.subjects = subjects
        self.num_of_learners = num_of_learners
        self.delta = 0.58
        self.sigma = 100
        self.iterations = 3
        self.w1 = 0.65
        self.w2 = 0.35
        self.path_points = path_points
        self.t = np.linspace(0,1,self.path_points)
        self.bernstein = [self.__bernstein_poly(i, self.subjects+2-1, self.t) for i in range(self.subjects+2)]
        self.theta = None
        self.path_x = None
        self.path_y = None

    def __transformation(self, start, end):
        if self.theta:
            return np.array([[np.cos(self.theta), np.sin(self.theta)], [-np.sin(self.theta), np.cos(self.theta)]])
        
        else:
            self.theta = np.arctan2(end[1]-start[1], end[0]-start[0])
            return np.array([[np.cos(self.theta), np.sin(self.theta)], [-np.sin(self.theta), np.cos(self.theta)]])

    def __intersection(self, x, bound):
        intersections = []

        edges = [(bound[i], bound[(i+1) % 4]) for i in range(4)]

        for (x1, y1), (x2, y2) in edges:
            if (x1 - x) * (x2 - x) <= 0:
                if x1 == x2:
                    intersections.extend([(x1,y1), (x2,y2)])
                else:
                    y_intersect = y1 + (y2 - y1) * ((x - x1) / (x - x2))
                    intersections.append((x, y_intersect))
        intersections = sorted(set(intersections), key=lambda p:p[1])
        return intersections
    
    def __bernstein_poly(self, i, n, t):
        return comb(n, i) * (t ** i) * ((1-t) ** (n-i))


    def __learner_boundary(self):
        self.Xmin = np.array([self.bounds[0][0]] * self.subjects)
        self.Xmax = np.array([self.bounds[2][0]] * self.subjects)
        self.Ymin = np.array([self.bounds[0][1]] * self.subjects)
        self.Ymax = np.array([self.bounds[1][1]] * self.subjects)

    def __get_obstacles(self, obstacles=None):
        # add safety bounds to obstacles
        self.obstacles = []

        if not self.bounds_polygon:
            bound = SPoly(self.bounds)
            safety = bound.buffer(-self.delta)
            self.bounds_polygon = {'obstacle':bound, 'safety_polygon':safety}
        
        for coordinates in obstacles:
            obstacle = SPoly(coordinates)
            safety = obstacle.buffer(self.delta)
            self.obstacles.append({'obstacle':obstacle, 'safety_polygon':safety})

        if not hasattr(self, '_spatial_index') or len(self._spatial_index)!=len(self.obstacles):
            safety = [obs['safety_polygon'] for obs in self.obstacles]
            self._spatial_index = STRtree(safety)
    

    def __generate_learners(self, num=None):
        # generate learners for path planning
        if not num:
            self.learners_x = self.Xmin + np.random.uniform(0, 1, (self.num_of_learners, self.subjects)) * (self.Xmax - self.Xmin)
            self.learners_y = self.Ymin + np.random.uniform(0, 1, (self.num_of_learners, self.subjects)) * (self.Ymax - self.Ymin)
        else:
            learners_x = self.Xmin + np.random.uniform(0, 1, (num, self.subjects)) * (self.Xmax - self.Xmin)
            learners_y = self.Ymin + np.random.uniform(0, 1, (num, self.subjects)) * (self.Ymax - self.Ymin)
            pos = np.random.choice(self.num_of_learners, num)
            for i, p in enumerate(pos):
                self.learners_x[p] = np.copy(learners_x[i])
                self.learners_y[p] = np.copy(learners_y[i])

    def __curve_points(self, control_x, control_y):
        control_x = np.insert(control_x, 0, self.start[0])
        control_y = np.insert(control_y, 0, self.start[1])
        control_x = np.append(control_x, self.end[0])
        control_y = np.append(control_y, self.end[1])

        path_x = np.zeros_like(self.t)
        path_y = np.zeros_like(self.t)
        n = self.subjects+2-1
        for i in range(n+1):
            path_x += self.bernstein[i] * control_x[i]
            path_y += self.bernstein[i] * control_y[i]
        return path_x, path_y

    def __path_length(self, path_x, path_y):
        # calculate path length
        path_length = 0
        for i in range(1, self.path_points):
            path_length += np.sqrt((path_x[i] - path_x[i-1])**2 + (path_y[i] - path_y[i-1])**2)
        return path_length

    def __path_smoothness(self, path_x, path_y):
        # calculate path smoothness
        smoothness = 0
        for i in range(1, self.path_points-1):
            turning_angle = np.rad2deg(np.arctan2(path_y[i+1] - path_y[i], path_x[i+1] - path_x[i]) - np.arctan2(path_y[i] - path_y[i-1], path_x[i] - path_x[i-1]))
            smoothness += abs(turning_angle)
        return smoothness

    def __path_penalty(self, path_x, path_y):
        # calculate penalty
        if not self.obstacles:
            return 0
        
        penalty = 0
        path_points = [Point(x,y) for x,y in zip(path_x, path_y)]

        for point in path_points:
            nearby = self._spatial_index.query(point)
            for polygon_idx in nearby:
                if self.obstacles[polygon_idx]['safety_polygon'].contains(point):
                    penalty += 1 / point.distance(self.obstacles[polygon_idx]['safety_polygon'].boundary)
                    break
            
            if not self.bounds_polygon['safety_polygon'].contains(point):
                d =  point.distance(self.bounds_polygon['safety_polygon'].boundary)
                if d >= self.delta:
                    penalty += d
                else:
                    penalty += 1 / d
        return penalty

    def __f_xy(self, control_points_x, control_points_y):
        # objective function
        path_x, path_y = self.__curve_points(control_x=control_points_x,control_y=control_points_y)
        path_length = self.__path_length(path_x=path_x, path_y=path_y)
        path_smoothness = self.__path_smoothness(path_x=path_x, path_y=path_y)
        penalty = self.__path_penalty(path_x=path_x, path_y=path_y)
        return self.w1 * path_length + self.w2 * path_smoothness + self.sigma * penalty

    def __teacher_phase(self):
        teacher_index = np.argmin(self.fxy_values)
        teacher_x = self.learners_x[teacher_index]
        teacher_y = self.learners_y[teacher_index]

        mean_x = np.mean(self.learners_x, axis=0)
        mean_y = np.mean(self.learners_y, axis=0)

        diff_mean_x = np.random.uniform(0,1,self.subjects) * (teacher_x - np.random.rand(self.subjects) * mean_x)
        diff_mean_y = np.random.uniform(0,1,self.subjects) * (teacher_y - np.random.rand(self.subjects) * mean_y)

        for index in range(self.num_of_learners):
            for subject_index in range(self.subjects):
                x = np.copy(self.learners_x[index])
                y = np.copy(self.learners_y[index])

                x[subject_index] += diff_mean_x[subject_index]
                y[subject_index] += diff_mean_y[subject_index]

                new_fxy = self.__f_xy(control_points_x=x, control_points_y=y)

                if new_fxy < self.fxy_values[index]:
                    self.fxy_values[index] = np.copy(new_fxy)
                    self.learners_x[index] = np.copy(x)
                    self.learners_y[index] = np.copy(y)

    def __learner_phase(self):
        for XP_index in range(self.num_of_learners):
            for subject_index in range(self.subjects):
                XQ_index = np.random.choice(range(self.num_of_learners))

                if self.fxy_values[XP_index] == self.fxy_values[XQ_index]:
                    continue

                else:
                    XP_x = np.copy(self.learners_x[XP_index])
                    XP_y = np.copy(self.learners_y[XP_index])

                    XQ_x = np.copy(self.learners_x[XQ_index])
                    XQ_y = np.copy(self.learners_y[XQ_index])

                    if self.fxy_values[XP_index] < self.fxy_values[XQ_index]:
                        XP_x[subject_index] += np.random.uniform(0,1) * (XP_x[subject_index] - XQ_x[subject_index])
                        XP_y[subject_index] += np.random.uniform(0,1) * (XP_y[subject_index] - XQ_y[subject_index])
                    else:
                        XP_x[subject_index] += np.random.uniform(0,1) * (XQ_x[subject_index] - XP_x[subject_index])
                        XP_y[subject_index] += np.random.uniform(0,1) * (XQ_y[subject_index] - XP_y[subject_index])

                    new_fxy = self.__f_xy(control_points_x=XP_x, control_points_y=XP_y)
                    if new_fxy < self.fxy_values[XP_index]:
                        self.fxy_values[XP_index] = np.copy(new_fxy)
                        self.learners_x[XP_index] = np.copy(XP_x)
                        self.learners_y[XP_index] = np.copy(XP_y)


    def collision(self):
        # check if path has collision
        if self.path_x is None and self.path_y is None:
            return 1
        
        path_points = [Point(x,y) for x,y in zip(self.path_x, self.path_y)]
        for point in path_points:
            nearby = self._spatial_index.query(point)
            for polygon_idx in nearby:
                if self.obstacles[polygon_idx]['safety_polygon'].contains(point):
                    return 1
        return 0

    def path_planning(self, obstacles=None, new_start=None):
        if new_start:
            self.start = new_start

        self.__get_obstacles(obstacles=obstacles)
        
        check = self.collision()
        if not check:
            return self.path_x, self.path_y

        self.__learner_boundary()
        self.__generate_learners()
        self.fxy_values = []
        for idx in range(self.num_of_learners):
            self.fxy_values.append(self.__f_xy(self.learners_x[idx], self.learners_y[idx]))

        for i in range(self.iterations):
            self.__teacher_phase()
            self.__learner_phase()

        opt_idx = np.argmin(self.fxy_values)
        control_x = np.round(self.learners_x[opt_idx], 2)
        control_y = np.round(self.learners_y[opt_idx], 2)

        self.path_x, self.path_y = self.__curve_points(control_x=control_x, control_y=control_y)
        return self.path_x, self.path_y
    
def quaternion_from_yaw(yaw):
    qz = math.sin(yaw/2.0)
    qw = math.cos(yaw/2.0)
    return (0.0, 0.0, qz, qw)

def get_bounding_box(grid, i, j, visited, origin, resolution):
    min_x, min_y = j, i
    max_x, max_y = j, i

    stack = [(i,j)]
    while stack:
        x, y = stack.pop()
        if 0 <= x < grid.shape[0] and 0 <= y < grid.shape[1] and grid[x,y] == 100 and not visited[x,y]:
            visited[x,y] = True
            min_x, min_y = min(min_x, y), min(min_y, x)
            max_x, max_y = max(max_x, y), max(max_y, x)

            stack.extend([(x-1, y), (x+1, y), (x, y-1), (x, y+1)])

    bl = (origin[0] + min_x * resolution, origin[1] + min_y * resolution)
    tr = (origin[0] + (max_x+1) * resolution, origin[1] + (max_y+1) * resolution)

    return bl, tr

def extract_obstacles(grid, origin, resolution):
    obstacles = []
    visited = np.zeros_like(grid, dtype=bool)

    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            if grid[i,j] == 100 and not visited[i,j]:
                bottom_left, top_right = get_bounding_box(grid=grid, i=i, j=j, visited=visited, origin=origin, resolution=resolution)
                obstacles.append([bottom_left, (bottom_left[0], top_right[1]), top_right, (top_right[0], bottom_left[1])])
    return obstacles

def Path_callback(msg):
    global start
    global path_points
    global marker_publisher

    resolution = msg.info.resolution
    width = msg.info.width
    height = msg.info.height
    origin = (msg.info.origin.position.x, msg.info.origin.position.y, msg.info.origin.position.z)

    # Check the grid data size before reshaping
    if len(msg.data) != width * height:
        rospy.logerr("OccupancyGrid data size does not match expected dimensions!")
        return
    
    grid = np.array(msg.data).reshape((height, width))

    obstacles = extract_obstacles(grid=grid, origin=origin, resolution=resolution)

    unique_obstacles = []
    seen = set()
    for obs in obstacles:
        obs_tuple = tuple(sorted(obs))
        if obs_tuple not in seen:
            unique_obstacles.append(obs)
            seen.add(obs_tuple)

    path_x, path_y = path_planner.path_planning(obstacles=unique_obstacles, new_start=start)

    time_now = rospy.Time.now()
    path = Path()
    path.header.frame_id = "map"
    path.header.stamp = time_now

    for i, (p, color) in enumerate([((path_x[0], path_y[0]), (0,1,0)), ((path_x[-1], path_y[-1]), (1,0,0))]):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = time_now
        marker.ns = "path_markers"
        marker.id = i
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y = p
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.3
        marker.color.a ,marker.color.r ,marker.color.g, marker.color.b = (1.0, *color)
        marker_publisher.publish(marker)

    
    for i in range(len(path_x)):
        if i == 0:
            dx, dy = path_x[i+1] - path_x[i], path_y[i+1] - path_y[i]
        elif i == len(path_x) - 1:
            dx, dy = path_x[i] - path_x[i-1], path_y[i] - path_y[i-1]
        else:
            dx, dy = path_x[i+1] - path_x[i-1], path_y[i+1] - path_y[i-1]
        yaw = math.atan2(dy, dx)
        q = quaternion_from_yaw(yaw)
        
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = path_x[i]
        pose.pose.position.y = path_y[i]
        pose.pose.position.z = 0.0
    
        pose.pose.orientation.x, pose.pose.orientation.y = q[:2]
        pose.pose.orientation.z, pose.pose.orientation.w = q[2:]

        path.poses.append(pose)
    path_points.publish(path)

def Odom_callback(msg):

    global start
    start = (msg.pose.pose.position.x, msg.pose.pose.position.y)


start_x = rospy.get_param('start_x')
start_y = rospy.get_param('start_y')
start = (start_x, start_y)

end_x = rospy.get_param('end_x')
end_y = rospy.get_param('end_y')
end = (end_x,end_y)

bounds = rospy.get_param('bounds')

odom = rospy.get_param('odom_topic','/odom')
path_topic = rospy.get_param('path_topic','/path_topic')
marker_topic = rospy.get_param('marker_topic','/visualization_marker')
occupancy_map_topic = rospy.get_param('occupancy_map_topic','/occupancy_map/2D_occupancy_map')

num_of_learners = rospy.get_param('num_of_learners')
no_of_path_points = rospy.get_param('path_points')
subjects = rospy.get_param('subjects')


rospy.init_node("Navigation", anonymous=True)
path_points = rospy.Publisher(path_topic, Path, queue_size=10)
marker_publisher = rospy.Publisher(marker_topic, Marker, queue_size=10)
rospy.loginfo("Logger started")
rospy.on_shutdown(lambda: rospy.loginfo("Logger shutdown complete"))

rospy.Subscriber(odom, Odometry, Odom_callback)
path_planner = TLBO(start=start, end=end, bounds=bounds, num_of_learners=num_of_learners,path_points=no_of_path_points,subjects=subjects)
rospy.Subscriber(occupancy_map_topic, OccupancyGrid, Path_callback)

rospy.spin()

