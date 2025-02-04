
import rospy
import numpy as np
from scipy.special import comb
from geometry_msgs.msg import Point as RosPoint
from shapely.strtree import STRtree
from shapely.geometry import Point, Polygon
from std_msgs.msg import Float32MultiArray, Float32

class TLBO():

    def __init__(self, start, end, bounds, design_variables=2, number_of_learners=25):
        self.start = start
        self.end = end
        self.bounds = bounds
        self.design_variables = design_variables # subjects
        self.number_of_learners = number_of_learners # generated paths
        self.obstacles = None
        self.delta_safe = 0 # safety distance
        self.penalty_multiplier = 200 # sigma value
        self.max_iterations = 30
        self.w1 = 0.65 # weight for the distance between the start and end points
        self.w2 = 0.35 # weight for path smoothness
        self.optimal_path_x = None # x-coordinates of the optimal path
        self.optimal_path_y = None # y-coordinates of the optimal path
        self.number_of_path_points = 25 # number of points in the path
        self.optimal_control_points_x = None
        self.optimal_control_points_y = None
        self.objective_values = None
        self.Xmax = None
        self.Xmin = None
        self.Ymin = None
        self.Ymax = None
        self.learners_x = None
        self.learners_y = None
        self.bound_obstacle = None
        self.t_values = np.linspace(0,1,self.number_of_path_points)
        self.bernstien = [self.bernstein_poly(i, self.design_variables+2-1, self.t_values) for i in range(self.design_variables+2)]
    
    def custom_variables(self, delta_safe=None, penalty_multiplier=None, max_iterations=None, w1=None, w2=None, number_of_learners=None, design_variables=None):
        if delta_safe is not None:
            self.delta_safe = delta_safe
        if penalty_multiplier is not None:
            self.penalty_multiplier = penalty_multiplier
        if max_iterations is not None:
            self.max_iterations = max_iterations
        if w1 is not None:
            self.w1 = w1
        if w2 is not None:
            self.w2 = w2
        if number_of_learners is not None:
            self.number_of_learners = number_of_learners
        if design_variables is not None:
            self.design_variables = design_variables
    
    def define_obstacles(self, obstacle_coords):
        if self.obstacles is None:
            self.obstacles = []

        bound_polygon = Polygon(self.bounds)
        safety_polygon = bound_polygon.buffer(-self.delta_safe)
        self.bound_obstacle = {'obstacle': bound_polygon, 'safety_polygon': safety_polygon}
        
        for coordinates in obstacle_coords:
            polytropic_obstacle = Polygon(coordinates)
            safety_polygon = polytropic_obstacle.buffer(self.delta_safe)
            obstacle = {'obstacle': polytropic_obstacle, 'safety_polygon': safety_polygon}
            self.obstacles.append(obstacle)

        if not hasattr(self, '_spatial_index'):
            safety_polygons = [obs['safety_polygon'] for obs in self.obstacles]
            self._spatial_index = STRtree(safety_polygons)
    
    def __learners_boundary_calculation(self):
        segments = np.sort(np.linspace(start=self.start[0], stop=self.end[0], num=self.design_variables+1))

        self.Xmin = segments[:-1]
        self.Xmax = segments[1:]
        self.Ymin = np.array([self.bounds[0][1]] * len(self.Xmin))
        self.Ymax = np.array([self.bounds[1][1]] * len(self.Xmax))

    def __generate_learners(self):
        # these are control points for the path
        self.__learners_boundary_calculation()
        self.learners_x = self.Xmin + np.random.uniform(0,1,(self.number_of_learners, self.design_variables)) * (self.Xmax - self.Xmin)
        self.learners_y = self.Ymin + np.random.uniform(0,1,(self.number_of_learners, self.design_variables)) * (self.Ymax - self.Ymin)
    
    def bernstein_poly(self, i, n, t):
        return comb(n, i) * (t ** i) * ((1 - t) ** (n - i))
    
    def __generate_path_points(self, control_points_x, control_points_y):
        control_points_x = np.insert(control_points_x, 0, self.start[0])
        control_points_x = np.append(control_points_x, self.end[0])
        control_points_y = np.insert(control_points_y, 0, self.start[1])
        control_points_y = np.append(control_points_y, self.end[1])

        smooth_path_x = np.zeros_like(self.t_values)
        smooth_path_y = np.zeros_like(self.t_values)
        n = len(control_points_x) - 1
        for i in range(n+1):
            smooth_path_x += self.bernstien[i] * control_points_x[i]
            smooth_path_y += self.bernstien[i] * control_points_y[i]
        return smooth_path_x, smooth_path_y
    
    def __calculate_path_length(self, path_points_x, path_points_y):
        path_length = 0
        for i in range(1, len(path_points_x)):
            path_length += np.sqrt((path_points_x[i] - path_points_x[i-1])**2 + (path_points_y[i] - path_points_y[i-1])**2)
        return path_length
    
    def __calculate_path_smoothness(self, path_points_x, path_points_y):
        smoothness = 0
        for i in range(1, len(path_points_x)-1):
            turning_angle = np.rad2deg(np.arctan2(path_points_y[i+1] - path_points_y[i], path_points_x[i+1] - path_points_x[i]) - np.arctan2(path_points_y[i] - path_points_y[i-1], path_points_x[i] - path_points_x[i-1]))
            smoothness += abs(turning_angle)
        return smoothness
    
    def __path_point_collision_penalty(self, path_points_x, path_points_y):
        if self.obstacles is None:
            return 0
        
        penalty = 0
        path_points = [Point(x, y) for x, y in zip(path_points_x, path_points_y)]
        
        for point in path_points:

            nearby_polygons = self._spatial_index.query(point)
            for polygon_index in nearby_polygons:
                if self.obstacles[polygon_index]['safety_polygon'].contains(point):
                    penalty += 1 / point.distance(self.obstacles[polygon_index]['safety_polygon'].boundary)

            if not self.bound_obstacle['safety_polygon'].contains(point):
                d = point.distance(self.bound_obstacle['safety_polygon'].boundary)
                if d >= 3:
                    penalty += d
                else:
                    penalty += 1 / d    

        return penalty
    
    def objective_function(self, control_points_x, control_points_y):
        path_points_x, path_points_y = self.__generate_path_points(control_points_x, control_points_y)
        path_length = self.__calculate_path_length(path_points_x, path_points_y)
        path_smoothness = self.__calculate_path_smoothness(path_points_x, path_points_y)
        path_collision_penalty = self.__path_point_collision_penalty(path_points_x, path_points_y)
        return self.w1 * path_length + self.w2 * path_smoothness + self.penalty_multiplier * path_collision_penalty
    
    def __calculate_all_objectives(self):
        self.objective_values = []
        for i in range(self.number_of_learners):
            self.objective_values.append(self.objective_function(self.learners_x[i], self.learners_y[i]))

    def __teacher_phase(self):

        teacher_index = np.argmin(self.objective_values)
        teacher_x = self.learners_x[teacher_index]
        teacher_y = self.learners_y[teacher_index]

        mean_x = np.mean(self.learners_x, axis=0)
        mean_y = np.mean(self.learners_y, axis=0)

        diff_mean_x = np.random.uniform(0,1,self.design_variables) * (teacher_x - np.random.rand(self.design_variables) * mean_x)
        diff_mean_y = np.random.uniform(0,1,self.design_variables) * (teacher_y - np.random.rand(self.design_variables) * mean_y)
        
        for index in range(self.number_of_learners):
            for design_variable_index in range(self.design_variables):
                x = np.copy(self.learners_x[index])
                y = np.copy(self.learners_y[index])

                x[design_variable_index] = self.learners_x[index][design_variable_index] + diff_mean_x[design_variable_index]
                y[design_variable_index] = self.learners_y[index][design_variable_index] + diff_mean_y[design_variable_index]

                new_objective_value = self.objective_function(control_points_x=x, control_points_y=y)

                if new_objective_value < self.objective_values[index]:
                    self.objective_values[index] = np.copy(new_objective_value)
                    self.learners_x[index] = np.copy(x)
                    self.learners_y[index] = np.copy(y)
        
    def __learner_phase(self):

        for XP_index in range(self.number_of_learners):
            for desgin_variable_index in range(self.design_variables):

                XQ_index = np.random.choice(range(self.number_of_learners))

                if self.objective_values[XP_index] == self.objective_values[XQ_index]:
                    continue
                else:
                    XP_x = np.copy(self.learners_x[XP_index])
                    XP_y = np.copy(self.learners_y[XP_index])

                    XQ_x = np.copy(self.learners_x[XQ_index])
                    XQ_y = np.copy(self.learners_y[XQ_index])

                    if self.objective_values[XP_index] < self.objective_values[XQ_index]:

                        XP_x[desgin_variable_index] = XP_x[desgin_variable_index] + np.random.uniform(0,1) * (XP_x[desgin_variable_index] - XQ_x[desgin_variable_index])

                        XP_y[desgin_variable_index] = XP_y[desgin_variable_index] + np.random.uniform(0,1) * (XP_y[desgin_variable_index] - XQ_y[desgin_variable_index])
                    
                    else:
                        XP_x[desgin_variable_index] = XP_x[desgin_variable_index] + np.random.uniform(0,1) * (XQ_x[desgin_variable_index] - XP_x[desgin_variable_index])

                        XP_y[desgin_variable_index] = XP_y[desgin_variable_index] + np.random.uniform(0,1) * (XQ_y[desgin_variable_index] - XP_y[desgin_variable_index])
                    
                    new_objective_value = self.objective_function(control_points_x=XP_x, control_points_y=XP_y)

                    if new_objective_value < self.objective_values[XP_index]:
                        self.objective_values[XP_index] = np.copy(new_objective_value)
                        self.learners_x[XP_index] = np.copy(XP_x)
                        self.learners_y[XP_index] = np.copy(XP_y)
    
    def run_iterations(self):
                
        self.__learners_boundary_calculation()
        self.__generate_learners()
        self.__calculate_all_objectives()
        
        for _ in range(self.max_iterations):
            self.__teacher_phase()
            self.__learner_phase()
        
        optimal_value_index = np.argmin(self.objective_values)
        self.optimal_control_points_x = np.round(self.learners_x[optimal_value_index], 2)
        self.optimal_control_points_y = np.round(self.learners_y[optimal_value_index], 2)

        self.optimal_path_x, self.optimal_path_y = self.__generate_path_points(self.optimal_control_points_x, self.optimal_control_points_y)
        


# def publish_control_points(control_points):
#     pub = rospy.Publisher('Control_points', Float32MultiArray, queue_size=10)
#     rospy.init_node('bezier_publisher', anonymous=True)
#     rate = rospy.Rate(1)  # 1 Hz publishing rate
#     while not rospy.is_shutdown():

#         flattened_points = Float32MultiArray()
        
#         # Flatten the control points (list of tuples) into a single list of floats
#         for pt in control_points:
#             msg = RosPoint()
#             msg.x = pt[0]
#             msg.y = pt[1]
#             msg.z = 0
#             flattened_points.data.append(msg)
        
#         rospy.loginfo("Publishing control points:%s", flattened_points)

#         pub.publish(flattened_points)
        
#         rate.sleep()
        # break

class ControlPointPublisher:
    def __init__(self):
        self.ready_to_publish = False
        self.pub = rospy.Publisher('Control_points', Float32MultiArray, queue_size=10)
        self.sub = rospy.Subscriber('start_signal', Float32, self.start_signal_callback)

    def start_signal_callback(self, msg):
        rospy.loginfo(msg)
        if msg.data == 1:
            rospy.loginfo("Received '1' from subscriber. Ready to publish control points.")
            self.ready_to_publish = True

    def publish_control_points(self, control_points):
        rospy.init_node('bezier_publisher', anonymous=True)
        rate = rospy.Rate(1)  # 1 Hz publishing rate

        # Wait for subscriber to send '1' before starting the publishing
        rospy.loginfo("Waiting for start signal...")
        while not rospy.is_shutdown() and not self.ready_to_publish:
            rospy.sleep(0.01)  # Sleep a bit and keep checking


        rospy.loginfo("Start signal received. Publishing control points now.")

        # Publish the control points when the signal is received
        while not rospy.is_shutdown():

            flattened_points = Float32MultiArray()
            
            # Flatten the control points (list of tuples) into a single list of floats
            for pt in control_points:
                msg = RosPoint()
                msg.x = pt[0]
                msg.y = pt[1]
                msg.z = 0
                flattened_points.data.append(msg)    # Add z (0 as specified)
            
            rospy.loginfo("Publishing control points:%s", flattened_points)
            self.pub.publish(flattened_points)
            
            rate.sleep()
        
        
def main():
    # Example control points: Replace with your calculated Bezier control points\
    rospy.init_node('bezier_publisher', anonymous=True)    
    
    bounds = [(0,0), (0,5230), (8390, 5230), (8390, 0)]
    obstacles = [
    [(1560, 3380), (1560, 5230), (2170, 5230), (2170, 3380)],
    [(2530, 3380), (2530, 5230), (3740, 5230), (3740, 3380)],
    [(4080, 1630), (4080, 5230), (5280, 5230), (5280, 1630)],
    [(600, 2900), (600, 4600), (1600, 4600), (1600, 2900)]
    ]
    start = (534,534)
    end = (7200, 4400)
    
    tlbo = TLBO(start=start, end=end, bounds=bounds)
    tlbo.custom_variables(delta_safe=533, penalty_multiplier=6000000)
    tlbo.define_obstacles(obstacle_coords=obstacles)
    tlbo.run_iterations()
    
    control_points = list(zip(tlbo.optimal_control_points_x, tlbo.optimal_control_points_y))
    control_points.insert(0, start)
    control_points.append(end)

    publisher = ControlPointPublisher()
    publisher.publish_control_points(control_points)



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
