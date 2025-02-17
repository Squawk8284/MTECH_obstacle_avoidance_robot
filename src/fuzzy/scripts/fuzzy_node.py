#!/usr/bin/env python
import rospy
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from math import comb
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float64MultiArray
import tf.transformations as tf_trans

##############################################################################
# Global variables for robot state and control points
##############################################################################
robot_state = {'x': None, 'y': None, 'heading': None}
control_points = None  # Will be a list of (x,y) tuples

##############################################################################
# 1. Bezier Curve Generation (for any number of control points)
##############################################################################
def bezier_curve(control_points, num_samples=20):
    """
    Generates waypoints along a Bezier curve defined by an arbitrary number
    of control points using the Bernstein polynomial formulation.
    """
    n = len(control_points) - 1
    t_values = np.linspace(0, 1, num_samples)
    waypoints = []
    for t in t_values:
        x_t = 0.0
        y_t = 0.0
        for i, (x, y) in enumerate(control_points):
            binom = comb(n, i)
            basis = binom * (t ** i) * ((1 - t) ** (n - i))
            x_t += basis * x
            y_t += basis * y
        waypoints.append((x_t, y_t))
    return waypoints

##############################################################################
# 2. Compute α₁ and α₂ from ordered waypoints
##############################################################################
def compute_alpha1_alpha2_ordered(waypoints, idx):
    """
    Given an ordered list of waypoints and a current index idx (Ni),
    define:
      - p0 = waypoints[idx]   (Ni)
      - p1 = waypoints[min(idx+1, len(waypoints)-1)]   (Ni+1)
      - p2 = waypoints[min(idx+2, len(waypoints)-1)]   (Ni+2)
      - p3 = waypoints[min(idx+3, len(waypoints)-1)]   (Ni+3)
    Then, α₁ is the angle between vector p0→p1 and p1→p2 and
          α₂ is the angle between vector p1→p2 and p2→p3.
    Angles are computed in degrees.
    """
    if not waypoints:
        return 0.0, 0.0

    i0 = idx
    i1 = min(idx + 1, len(waypoints) - 1)
    i2 = min(idx + 2, len(waypoints) - 1)
    i3 = min(idx + 3, len(waypoints) - 1)

    p0 = np.array(waypoints[i0])
    p1 = np.array(waypoints[i1])
    p2 = np.array(waypoints[i2])
    p3 = np.array(waypoints[i3])

    v1 = p1 - p0
    v2 = p2 - p1
    v3 = p3 - p2

    def angle_between(v_a, v_b):
        norm_a = np.linalg.norm(v_a)
        norm_b = np.linalg.norm(v_b)
        if norm_a == 0 or norm_b == 0:
            return 0.0
        cosine_angle = np.clip(np.dot(v_a, v_b) / (norm_a * norm_b), -1.0, 1.0)
        return np.degrees(np.arccos(cosine_angle))

    alpha1 = angle_between(v1, v2)
    alpha2 = angle_between(v2, v3)
    return alpha1, alpha2

##############################################################################
# 3. Fuzzy Controller Definitions
##############################################################################
# --- Controller C: inputs (α₁, α₂) → output c ---
alpha1_var = ctrl.Antecedent(np.linspace(0, 180, 181), 'alpha1')
alpha2_var = ctrl.Antecedent(np.linspace(0, 180, 181), 'alpha2')
c_var = ctrl.Consequent(np.linspace(0, 5.5, 551), 'c')

alpha1_var['Straight1'] = fuzz.trimf(alpha1_var.universe, [0, 0, 20])
alpha1_var['High1']     = fuzz.trimf(alpha1_var.universe, [1, 20, 95])
alpha1_var['VeryHigh1'] = fuzz.trimf(alpha1_var.universe, [85, 140, 180])

alpha2_var['Straight2'] = fuzz.trimf(alpha2_var.universe, [0, 0, 30])
alpha2_var['High2']     = fuzz.trimf(alpha2_var.universe, [10, 30, 95])
alpha2_var['VeryHigh2'] = fuzz.trimf(alpha2_var.universe, [80, 140, 180])

c_var['low']       = fuzz.trimf(c_var.universe, [0, 0, 0.1])
c_var['medium']    = fuzz.trimf(c_var.universe, [1.5, 1.5, 1.6])
c_var['high']      = fuzz.trimf(c_var.universe, [4, 4, 4.1])
c_var['very_high'] = fuzz.trimf(c_var.universe, [5, 5, 5.1])

rules_c = [
    ctrl.Rule(alpha1_var['Straight1'] & alpha2_var['Straight2'], c_var['low']),
    ctrl.Rule(alpha1_var['Straight1'] & alpha2_var['High2'],     c_var['medium']),
    ctrl.Rule(alpha1_var['Straight1'] & alpha2_var['VeryHigh2'], c_var['high']),
    ctrl.Rule(alpha1_var['High1']     & alpha2_var['Straight2'], c_var['high']),
    ctrl.Rule(alpha1_var['High1']     & alpha2_var['High2'],     c_var['high']),
    ctrl.Rule(alpha1_var['High1']     & alpha2_var['VeryHigh2'], c_var['high']),
    ctrl.Rule(alpha1_var['VeryHigh1'] & alpha2_var['Straight2'], c_var['very_high']),
    ctrl.Rule(alpha1_var['VeryHigh1'] & alpha2_var['High2'],     c_var['very_high']),
    ctrl.Rule(alpha1_var['VeryHigh1'] & alpha2_var['VeryHigh2'], c_var['very_high'])
]

c_ctrl_system = ctrl.ControlSystem(rules_c)
c_sim = ctrl.ControlSystemSimulation(c_ctrl_system)

# --- Controller V: inputs (dR, dθ, Vc, c_in) → outputs (V, ω) ---
dR_var = ctrl.Antecedent(np.linspace(0, 1.5, 71), 'dR')
dtheta_var = ctrl.Antecedent(np.linspace(-180, 180, 361), 'dtheta')
Vc_var = ctrl.Antecedent(np.linspace(0, 1.0, 101), 'Vc')
c_in_var = ctrl.Antecedent(np.linspace(0, 5.5, 501), 'c_in')

V_var = ctrl.Consequent(np.linspace(0, 0.4, 401), 'V')
omega_var = ctrl.Consequent(np.linspace(-30, 30, 601), 'omega')

c_in_var['Low']      = fuzz.trimf(c_in_var.universe, [0, 0, 1.5])
c_in_var['High']     = fuzz.trimf(c_in_var.universe, [1, 2, 5.5])
c_in_var['VeryHigh'] = fuzz.trapmf(c_in_var.universe, [3, 4, 6, 6])

dR_var['Near']  = fuzz.trimf(dR_var.universe, [0.0, 0.0, 0.1])
dR_var['Close'] = fuzz.trimf(dR_var.universe, [0.02, 0.70, 1.40])
dR_var['Far']   = fuzz.trapmf(dR_var.universe, [1.0, 1.5, 1.8, 1.8])

dtheta_var['nVeryHigh'] = fuzz.trapmf(dtheta_var.universe, [-200, -200, -100, -50])
dtheta_var['nHigh']     = fuzz.trimf(dtheta_var.universe, [-100, -10, -1])
dtheta_var['Small']     = fuzz.trimf(dtheta_var.universe, [-10, 0, 10])
dtheta_var['High']      = fuzz.trimf(dtheta_var.universe, [1, 10, 100])
dtheta_var['VeryHigh']  = fuzz.trapmf(dtheta_var.universe, [50, 100, 200, 200])

Vc_var['Low']       = fuzz.trimf(Vc_var.universe, [0, 0, 0.2])
Vc_var['High']      = fuzz.trimf(Vc_var.universe, [0.1, 0.25, 1.0])
Vc_var['VeryHigh']  = fuzz.trapmf(Vc_var.universe, [0.4, 0.5, 1.5, 1.5])

V_var['low']    = fuzz.trimf(V_var.universe, [0.0, 0.0, 0.2])
V_var['medium'] = fuzz.trimf(V_var.universe, [0.1, 0.2, 0.3])
V_var['high']   = fuzz.trimf(V_var.universe, [0.25, 0.4, 0.5])

omega_var['neg']  = fuzz.trimf(omega_var.universe, [-35, -15, 0])
omega_var['zero'] = fuzz.trimf(omega_var.universe, [-10, 0, 10])
omega_var['pos']  = fuzz.trimf(omega_var.universe, [0, 15, 35])


table_str = """Low	Near	nVeryHigh	Low	low	neg
Low	Near	nVeryHigh	High	low	neg
Low	Near	nVeryHigh	VeryHigh	low	neg
Low	Near	nHigh	Low	low	neg
Low	Near	nHigh	High	low	neg
Low	Near	nHigh	VeryHigh	low	neg
Low	Near	Small	Low	medium	zero
Low	Near	Small	High	medium	zero
Low	Near	Small	VeryHigh	medium	zero
Low	Near	High	Low	low	pos
Low	Near	High	High	low	pos
Low	Near	High	VeryHigh	low	pos
Low	Near	VeryHigh	Low	low	pos
Low	Near	VeryHigh	High	low	pos
Low	Near	VeryHigh	VeryHigh	low	pos
Low	Close	nVeryHigh	Low	low	neg
Low	Close	nVeryHigh	High	low	neg
Low	Close	nVeryHigh	VeryHigh	low	neg
Low	Close	nHigh	Low	medium	neg
Low	Close	nHigh	High	medium	neg
Low	Close	nHigh	VeryHigh	medium	neg
Low	Close	Small	Low	medium	zero
Low	Close	Small	High	medium	zero
Low	Close	Small	VeryHigh	medium	zero
Low	Close	High	Low	medium	pos
Low	Close	High	High	medium	pos
Low	Close	High	VeryHigh	medium	pos
Low	Close	VeryHigh	Low	low	pos
Low	Close	VeryHigh	High	low	pos
Low	Close	VeryHigh	VeryHigh	low	pos
Low	Far	nVeryHigh	Low	low	neg
Low	Far	nVeryHigh	High	low	neg
Low	Far	nVeryHigh	VeryHigh	low	neg
Low	Far	nHigh	Low	medium	neg
Low	Far	nHigh	High	medium	neg
Low	Far	nHigh	VeryHigh	medium	neg
Low	Far	Small	Low	high	zero
Low	Far	Small	High	high	zero
Low	Far	Small	VeryHigh	high	zero
Low	Far	High	Low	medium	pos
Low	Far	High	High	medium	pos
Low	Far	High	VeryHigh	medium	pos
Low	Far	VeryHigh	Low	low	pos
Low	Far	VeryHigh	High	low	pos
Low	Far	VeryHigh	VeryHigh	low	pos
High	Near	nVeryHigh	Low	low	neg
High	Near	nVeryHigh	High	low	neg
High	Near	nVeryHigh	VeryHigh	low	neg
High	Near	nHigh	Low	low	neg
High	Near	nHigh	High	low	neg
High	Near	nHigh	VeryHigh	low	neg
High	Near	Small	Low	low	zero
High	Near	Small	High	low	zero
High	Near	Small	VeryHigh	low	zero
High	Near	High	Low	low	pos
High	Near	High	High	low	pos
High	Near	High	VeryHigh	low	pos
High	Near	VeryHigh	Low	low	pos
High	Near	VeryHigh	High	low	pos
High	Near	VeryHigh	VeryHigh	low	pos
High	Close	nVeryHigh	Low	low	neg
High	Close	nVeryHigh	High	low	neg
High	Close	nVeryHigh	VeryHigh	low	neg
High	Close	nHigh	Low	low	neg
High	Close	nHigh	High	low	neg
High	Close	nHigh	VeryHigh	low	neg
High	Close	Small	Low	medium	zero
High	Close	Small	High	medium	zero
High	Close	Small	VeryHigh	medium	zero
High	Close	High	Low	low	pos
High	Close	High	High	low	pos
High	Close	High	VeryHigh	low	pos
High	Close	VeryHigh	Low	low	pos
High	Close	VeryHigh	High	low	pos
High	Close	VeryHigh	VeryHigh	low	pos
High	Far	nVeryHigh	Low	low	neg
High	Far	nVeryHigh	High	low	neg
High	Far	nVeryHigh	VeryHigh	low	neg
High	Far	nHigh	Low	medium	neg
High	Far	nHigh	High	medium	neg
High	Far	nHigh	VeryHigh	medium	neg
High	Far	Small	Low	medium	zero
High	Far	Small	High	medium	zero
High	Far	Small	VeryHigh	medium	zero
High	Far	High	Low	medium	pos
High	Far	High	High	medium	pos
High	Far	High	VeryHigh	medium	pos
High	Far	VeryHigh	Low	low	pos
High	Far	VeryHigh	High	low	pos
High	Far	VeryHigh	VeryHigh	low	pos
VeryHigh	Near	nVeryHigh	Low	low	neg
VeryHigh	Near	nVeryHigh	High	low	neg
VeryHigh	Near	nVeryHigh	VeryHigh	low	neg
VeryHigh	Near	nHigh	Low	low	neg
VeryHigh	Near	nHigh	High	low	neg
VeryHigh	Near	nHigh	VeryHigh	low	neg
VeryHigh	Near	Small	Low	low	zero
VeryHigh	Near	Small	High	low	zero
VeryHigh	Near	Small	VeryHigh	low	zero
VeryHigh	Near	High	Low	low	pos
VeryHigh	Near	High	High	low	pos
VeryHigh	Near	High	VeryHigh	low	pos
VeryHigh	Near	VeryHigh	Low	low	pos
VeryHigh	Near	VeryHigh	High	low	pos
VeryHigh	Near	VeryHigh	VeryHigh	low	pos
VeryHigh	Close	nVeryHigh	Low	low	neg
VeryHigh	Close	nVeryHigh	High	low	neg
VeryHigh	Close	nVeryHigh	VeryHigh	low	neg
VeryHigh	Close	nHigh	Low	low	neg
VeryHigh	Close	nHigh	High	low	neg
VeryHigh	Close	nHigh	VeryHigh	low	neg
VeryHigh	Close	Small	Low	medium	zero
VeryHigh	Close	Small	High	medium	zero
VeryHigh	Close	Small	VeryHigh	medium	zero
VeryHigh	Close	High	Low	low	pos
VeryHigh	Close	High	High	low	pos
VeryHigh	Close	High	VeryHigh	low	pos
VeryHigh	Close	VeryHigh	Low	low	pos
VeryHigh	Close	VeryHigh	High	low	pos
VeryHigh	Close	VeryHigh	VeryHigh	low	pos
VeryHigh	Far	nVeryHigh	Low	low	neg
VeryHigh	Far	nVeryHigh	High	low	neg
VeryHigh	Far	nVeryHigh	VeryHigh	low	neg
VeryHigh	Far	nHigh	Low	medium	neg
VeryHigh	Far	nHigh	High	medium	neg
VeryHigh	Far	nHigh	VeryHigh	medium	neg
VeryHigh	Far	Small	Low	medium	zero
VeryHigh	Far	Small	High	medium	zero
VeryHigh	Far	Small	VeryHigh	medium	zero
VeryHigh	Far	High	Low	medium	pos
VeryHigh	Far	High	High	medium	pos
VeryHigh	Far	High	VeryHigh	medium	pos
VeryHigh	Far	VeryHigh	Low	low	pos
VeryHigh	Far	VeryHigh	High	low	pos
VeryHigh	Far	VeryHigh	VeryHigh	low	pos
"""

rules_list = []
for line in table_str.strip().splitlines():
    cols = line.split()
    antecedent_expr = c_in_var[cols[0]] & dR_var[cols[1]] & dtheta_var[cols[2]] & Vc_var[cols[3]]
    consequent_expr = (V_var[cols[4]], omega_var[cols[5]])
    rule = ctrl.Rule(antecedent_expr, consequent_expr)
    rules_list.append(rule)

v_ctrl_system = ctrl.ControlSystem(rules_list)
v_sim = ctrl.ControlSystemSimulation(v_ctrl_system)


##############################################################################
# 4. ROS Subscribers for Odometry and Control Points
##############################################################################
def odom_callback(msg):
    global robot_state
    robot_state['x'] = msg.pose.pose.position.x
    robot_state['y'] = msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    euler = tf_trans.euler_from_quaternion([orientation_q.x,
                                             orientation_q.y,
                                             orientation_q.z,
                                             orientation_q.w])
    robot_state['heading'] = np.degrees(euler[2])

def control_points_callback(msg):
    global control_points
    # Expect an even number of floats (x,y pairs)
    if len(msg.data) >= 2 and len(msg.data) % 2 == 0:
        control_points = []
        for i in range(0, len(msg.data), 2):
            control_points.append((msg.data[i], msg.data[i+1]))
        rospy.loginfo("Received new control points: %s", str(control_points))
    else:
        rospy.logwarn("Received control points message with insufficient or uneven data.")


##############################################################################
# 5. Main ROS Node: Fuzzy Control Using Ordered Waypoints
##############################################################################
def main():
    global control_points
    rospy.init_node('fuzzy_node')

    # Load parameters from the parameter server (from YAML)
    num_samples   = rospy.get_param('~num_samples', 20)
    threshold     = rospy.get_param('~threshold', 0.005)
    control_rate  = rospy.get_param('~rate', 10)
    initial_V     = rospy.get_param('~initial_velocity', 0.0001)

    # Subscribers and publishers
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/control_points', Float64MultiArray, control_points_callback)
    vel_pub = rospy.Publisher('/cmd_vel', Float64, queue_size=10)
    angular_pub = rospy.Publisher('/cmd_angular', Float64, queue_size=10)

    rospy.loginfo("Fuzzy controller node started, waiting for odometry and control points...")
    rate = rospy.Rate(control_rate)

    # Wait for control points to be received
    while not rospy.is_shutdown() and control_points is None:
        rospy.loginfo("Waiting for control points on /control_points...")
        rate.sleep()

    # Generate the Bezier curve waypoints from the received control points
    waypoints = bezier_curve(control_points, num_samples=num_samples)
    rospy.loginfo("Generated %d waypoints.", len(waypoints))
    wp_index = 0
    current_V = initial_V
    dt = sim_dt

    while not rospy.is_shutdown() and wp_index < len(waypoints):
        # Ensure odometry is available
        if robot_state['x'] is None or robot_state['y'] is None or robot_state['heading'] is None:
            rate.sleep()
            continue

        # Current target waypoint is the next in order
        target_wp = waypoints[wp_index]
        dx = target_wp[0] - robot_state['x']
        dy = target_wp[1] - robot_state['y']
        dist_to_wp = np.hypot(dx, dy)

        # If the robot is close enough to the current target, move to the next waypoint
        if dist_to_wp < threshold:
            rospy.loginfo("Reached waypoint %d: (%.2f, %.2f)", wp_index, target_wp[0], target_wp[1])
            wp_index += 1
            continue

        # Compute α₁ and α₂ using the ordered waypoints starting at the current index
        alpha1, alpha2 = compute_alpha1_alpha2_ordered(waypoints, wp_index)
        a1 = max(0, min(180, alpha1))
        a2 = max(0, min(180, alpha2))
        c_sim.input['alpha1'] = a1
        c_sim.input['alpha2'] = a2
        c_sim.compute()
        c_val = c_sim.output['c']

        # Compute dR and heading error (dθ) based on the target waypoint
        angle_to_wp = np.degrees(np.arctan2(dy, dx))
        dtheta_val = (angle_to_wp - robot_state['heading']) % 360
        if dtheta_val > 180:
            dtheta_val -= 360

        # Run Controller V
        v_sim.input['dR'] = dist_to_wp
        v_sim.input['dtheta'] = dtheta_val
        v_sim.input['Vc'] = current_V
        v_sim.input['c_in'] = c_val
        v_sim.compute()
        current_V = v_sim.output['V']
        omega_cmd = v_sim.output['omega']

        dt = min(0.09, (dist_to_wp / current_V) if current_V > 0 else 0.09)

        # Publish computed commands
        vel_pub.publish(Float64(current_V))
        angular_pub.publish(Float64(omega_cmd))

        rospy.loginfo("WP %d: dist=%.3f, α₁=%.2f, α₂=%.2f, V=%.3f, ω=%.2f",
                      wp_index, dist_to_wp, a1, a2, current_V, omega_cmd)
        rate.sleep()

    rospy.loginfo("All waypoints reached or node shutdown.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass