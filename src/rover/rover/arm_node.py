import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from pykdl import kdl

# Constants (You should update these with your actual arm parameters)
l1 = 1.0  # Length of the first link (in meters)
l2 = 1.0  # Length of the second link (in meters)

# Function to compute inverse kinematics (IK)
def theta_i(w_x, w_z):
    p = math.sqrt(math.pow(w_z, 2) + math.pow(w_x, 2))
    
    num = (math.pow(l1, 2) * -1) + math.pow(l1, 2) + math.pow(p, 2)
    denom = 2 * l1 * p
    
    calc = math.acos(num / denom)
    return math.degrees(calc)

def theta_s(w_x, w_z):
    i = math.radians(theta_i(w_x, w_z))
    p = math.sqrt(math.pow(w_z, 2) + math.pow(w_x, 2))
    
    num = p * math.sin(i)
    calc = math.asin(num / l2)
    return math.degrees(calc)

def theta_3(w_x, w_z):
    s = math.radians(theta_s(w_x, w_z))
    return math.degrees(math.pi - s)

def theta_E(w_x, w_z):
    return math.degrees(math.atan2(w_z, w_x))

def theta_2(w_x, w_z):
    deg = math.degrees(math.radians(theta_E(w_x, w_z)) - math.radians(theta_i(w_x, w_z)))
    
    if deg > 30:
        return None
    return deg

# Function to validate if the target position is reachable
def length_restriction(w_x, w_z):
    p = math.sqrt(math.pow(w_z, 2) + math.pow(w_x, 2))
    minimum = l1 - l2
    maximum = l1 + l2
    if minimum <= p <= maximum:
        return True
    return False

# Function for forward kinematics (FK)
def forward_kinematics(theta_1, theta_2):
    # Use KDL for FK calculations (2D arm)
    chain = kdl.Chain()
    segment1 = kdl.Segment(kdl.Joint(kdl.Joint.RotZ), kdl.Frame(kdl.Vector(l1, 0, 0)))
    segment2 = kdl.Segment(kdl.Joint(kdl.Joint.RotZ), kdl.Frame(kdl.Vector(l2, 0, 0)))
    chain.addSegment(segment1)
    chain.addSegment(segment2)
    
    # Set joint angles (theta_1 and theta_2 are the joint angles)
    joint_angles = kdl.JntArray(2)
    joint_angles[0] = math.radians(theta_1)
    joint_angles[1] = math.radians(theta_2)
    
    # Forward kinematics: Calculate end-effector position
    fk_solver = kdl.ForwardKinematics(chain)
    end_effector_frame = fk_solver.JntToCart(joint_angles)
    
    # Get the position of the end effector
    end_effector_position = end_effector_frame.p
    return end_effector_position.x(), end_effector_position.y()

# Main ROS Node that gets the joint angles
def joint_angles_callback(msg):
    # Assuming theta_1 and theta_2 are being published as a Twist message
    theta_1 = msg.linear.x  # Example of reading the first joint angle
    theta_2 = msg.linear.y  # Example of reading the second joint angle
    
    # Forward Kinematics to compute position
    x, y = forward_kinematics(theta_1, theta_2)
    rospy.loginfo(f"End-effector position: x = {x}, y = {y}")

    # Perform Inverse Kinematics based on the end effector position
    if length_restriction(x, y):
        theta1 = theta_2(x, y)
        theta2 = theta_s(x, y)
        rospy.loginfo(f"Inverse Kinematics: theta1 = {theta1}, theta2 = {theta2}")
    else:
        rospy.logwarn("Target position is not reachable.")

def ik_fk_node():
    rospy.init_node('ik_fk_node', anonymous=True)
    rospy.Subscriber('/joint_angles', Twist, joint_angles_callback)
    rospy.spin()

if __name__ == '__main__':
    ik_fk_node()
