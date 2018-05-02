#!/usr/bin/env python

# Copyright(C) 2017 Udacity Inc.

# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # -------------------------------------------------------------------------------------------------------------
        # FK code here
        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        # Create Modified DH parameters
        s = {alpha0:    0,  a0:   0, d1: 0.75, q1: q1,
             alpha1: -pi/2,  a1:     0.35, d2: 0, q2: q2 - pi/2,
             alpha2:    0,  a2:     1.25, d3: 0, q3: q3,
             alpha3: -pi/2,  a3: -0.054, d4: 1.50, q4: q4,
             alpha4:  pi/2,  a4:     0, d5: 0, q5: q5,
             alpha5: -pi/2,  a5:     0, d6: 0, q6: q6,
             alpha6:      0,  a6:     0, d7: 0.303, q7: 0}

        # Individual Transformations
        # Homogeneuos Transformation Link_0 to link_1
        T0_1 = Matrix([[cos(q1),       -sin(q1),        0,      a0],
                       [sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                       [sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
                       [0,        0,      0,      1]])
        T0_1 = T0_1.subs(s)
        # Homogeneuos Transformation Link_1 to link_2
        T1_2 = Matrix([[cos(q2),       -sin(q2),        0,      a1],
                       [sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                       [sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
                       [0,        0,      0,      1]])
        T1_2 = T1_2.subs(s)
        # Homogeneuos Transformation Link_2 to link_3
        T2_3 = Matrix([[cos(q3),       -sin(q3),        0,      a2],
                       [sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                       [sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
                       [0,        0,      0,      1]])
        T2_3 = T2_3.subs(s)
        # Homogeneuos Transformation Link_3 to link_4
        T3_4 = Matrix([[cos(q4),       -sin(q4),        0,      a3],
                       [sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                       [sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
                       [0,        0,      0,      1]])
        T3_4 = T3_4.subs(s)
        # Homogeneuos Transformation Link_4 to link_5
        T4_5 = Matrix([[cos(q5),       -sin(q5),        0,      a4],
                       [sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                       [sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
                       [0,        0,      0,      1]])
        T4_5 = T4_5.subs(s)
        # Homogeneuos Transformation Link_5 to link_6
        T5_6 = Matrix([[cos(q6),       -sin(q6),        0,      a5],
                       [sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                       [sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
                       [0,        0,      0,      1]])
        T5_6 = T5_6.subs(s)
        # Homogeneuos Transformation Link_6 to link_7 (Gripper)
        T6_G = Matrix([[cos(q7),       -sin(q7),        0,      a6],
                       [sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                       [sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
                       [0,        0,      0,      1]])
        T6_G = T6_G.subs(s)

        # Transform from Base link to end effector (Gripper)
        # Important: If we multiply in conjunction the result is different.
        T0_2 = (T0_1 * T1_2)  # Link_0 to Link_2
        T0_3 = (T0_2 * T2_3)  # Link_0 to Link_3
        T0_4 = (T0_3 * T3_4)  # Link_0 to Link_4
        T0_5 = (T0_4 * T4_5)  # Link_0 to Link_5
        T0_6 = (T0_5 * T5_6)  # Link_0 to Link_6
        T0_7 = (T0_6 * T6_G)  # Link_0 to Link_7

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            joint_trajectory_point = JointTrajectoryPoint()
            # Your IK code here
            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            # Saving the EE position in a matrix
            EE = Matrix([[px],
                         [py],
                         [pz]])

            # Requested end-effector orientation
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x,
                 req.poses[x].orientation.y,
                 req.poses[x].orientation.z,
                 req.poses[x].orientation.w])

            # Creating symbols for the rotation matrices (Roll, Pitch, Yaw)
            r, p, y = symbols('r p y')

            # Roll
            ROT_x = Matrix([[1,       0,       0],
                            [0,  cos(r), -sin(r)],
                            [0,  sin(r),  cos(r)]])
            # Pitch
            ROT_y = Matrix([[cos(p),       0,  sin(p)],
                            [0,       1,       0],
                            [-sin(p),       0,  cos(p)]])
            # Yaw
            ROT_z = Matrix([[cos(y), -sin(y),       0],
                            [sin(y),  cos(y),       0],
                            [0,       0,       1]])
            # The rotation matrix amoung the 3 axis
            ROT_EE = ROT_z * ROT_y * ROT_x

            # Correction Needed to Account for Orientation Difference Between
            # Definition of Gripper Link_G in URDF versus DH Convention
            ROT_corr = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))
            ROT_EE = ROT_EE * ROT_corr
            ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

            # Calculate Wrest Center
            WC = EE - (0.303) * ROT_EE[:, 2]

            # Calculate joint angles
            # Calculate theat1
            theta1 = atan2(WC[1], WC[0])

            # find the 3rd side of the triangle
            A = 1.50
            C = 1.25
            B = sqrt(pow((sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))

            # Cosine Laws SSS to find all inner angles of the triangle
            a = acos((B*B + C*C - A*A) / (2*B*C))
            b = acos((A*A + C*C - B*B) / (2*A*C))
            c = acos((A*A + B*B - C*C) / (2*A*B))

            # Find theta2 and theta3
            theta2 = pi/2 - a - atan2(WC[2]-0.75, sqrt(WC[0]*WC[0]+WC[1]*WC[1])-0.35)
            theta3 = pi/2 - (b+0.036)

            # Extract rotation matrix R0_3 from transformation matrix T0_3 the substitute angles q1-3
            R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

            # Get rotation matrix R3_6 from (transpose of R0_3 * R_EE)
            R3_6 = R0_3.transpose() * ROT_EE

            # Euler angles from rotation matrix
            theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
            theta5 = atan2(sqrt(R3_6[0, 2]*R3_6[0, 2] + R3_6[2, 2]*R3_6[2, 2]), R3_6[1, 2])
            theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()


if __name__ == "__main__":
    IK_server()
