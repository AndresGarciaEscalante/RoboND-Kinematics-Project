#!/usr/bin/python
# Author: Andres Ricardo Garcia Escalante
# Testing the forward kinematics of Kuka KR210

import numpy as np
from sympy import symbols, cos, sin, pi, pprint
from sympy.matrices import Matrix

# Your FK code here
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

# Define Modified DH Transformation matrix
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

# Create individual transformation matrices
# Transform from Base link to end effector (Gripper)
# Important: If we multiply in conjunction the result is different.
T0_2 = (T0_1 * T1_2)  # Link_0 to Link_2
T0_3 = (T0_2 * T2_3)  # Link_0 to Link_3
T0_4 = (T0_3 * T3_4)  # Link_0 to Link_4
T0_5 = (T0_4 * T4_5)  # Link_0 to Link_5
T0_6 = (T0_5 * T5_6)  # Link_0 to Link_6
T0_7 = (T0_6 * T6_G)  # Link_0 to Link_7

# Correction needed to Account of Orientation Difference Between Definition of Gripper link (URDF vs DH)
R_z = Matrix([[cos(np.pi), -sin(np.pi), 0, 0],
              [sin(np.pi),  cos(np.pi), 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])

R_y = Matrix([[cos(-np.pi/2), 0, sin(-np.pi/2), 0],
              [0,  1, 0, 0],
              [-sin(-np.pi/2), 0, cos(-np.pi/2), 0],
              [0, 0, 0, 1]])

R_corr = R_z * R_y
# Final Matrix values
T_total = (T0_7 * R_corr)

# Verifying the matrices output
# homogeneuos transformation from Link_0 to Link_7
print("\nT0_7 = \n")
pprint(T0_7.evalf(subs={q1: 1, q2: 0.30, q3: -0.45, q4: 0.90, q5: -0.35, q6: 0}))
print("\n")

# Final Matrix
print("\nT_total = \n")
pprint(T_total.evalf(subs={q1: 0, q2: 0, q3: 0, q4: 0, q5: 0, q6: 0}))
print("\n")
