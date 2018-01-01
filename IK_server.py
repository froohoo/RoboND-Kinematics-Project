#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
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

q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
#
# Create Modified DH parameters
#
s = {alpha0:    0., a0:   0., d1:  .75, q1: q1,
     alpha1:-pi/2., a1:  .35, d2:   0., q2: q2-pi/2.,
     alpha2:    0., a2: 1.25, d3:   0., q3: q3,
     alpha3:-pi/2., a3:-.054, d4:  1.5, q4: q4,
     alpha4: pi/2., a4:   0., d5:   0., q5: q5,
     alpha5:-pi/2., a5:   0., d6:   0., q6: q6,
     alpha6:    0., a6:   0., d7: .303, q7:  0.}
#
# Define Modified DH Transformation matrix
#
### Since the DH matrix is symbolically the same for each Tn-1 -> Tn transform
### rather than explicity express it each time, just create some generic symbols
### and an expression to represent the DH matrix.Then use that to create our
### individual transformation matrices via substitution:

#
# Create the generic DH parameter symbols:
#
theta, alpha, a, d = symbols('theta, alpha, a, d')
#
# Create the generalized DH matrix using generic symbols:
#
DH_Matrix = Matrix([
    [        cos(theta)   ,     -sin(theta)      ,      0     ,       a      ],
    [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
    [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha) ,  cos(alpha)*d],
    [           0         ,            0         ,      0     ,       1      ]])

#
# Create individual transformation matrices
# Define the elementary rotation matrices :
theta_x, theta_y, theta_z = symbols('theta_x, theta_y, theta_z')

xRot = Matrix([[      1       ,       0       ,        0       ],
               [      0       ,  cos(theta_x) , -sin(theta_x)  ],
               [      0       ,  sin(theta_x) ,  cos(theta_x)  ]])

yRot = Matrix([[  cos(theta_y),       0       ,  sin(theta_y)  ],
               [      0       ,       1       ,        0       ],
               [ -sin(theta_y),       0       ,  cos(theta_y)  ]])

zRot = Matrix([[  cos(theta_z),  -sin(theta_z),        0       ],
               [  sin(theta_z),   cos(theta_z),        0       ],
               [      0       ,       0       ,        1       ]])

# Create correction Rotation matrix
cRot = zRot * yRot
cRot = cRot.col_insert(3,Matrix([0,0,0]))
cRot = cRot.row_insert(3,Matrix([[0,0,0,1]]))
cRotn = cRot.subs({theta_y: -pi/2, theta_z:pi})

#
# Define the Homogeneous tranforms from each joint to the next
# and composition transfforms from 0-->7 for forward kinematics
# solver and 0-->6 for use in inverse kinematics solver.
#
T0_1 = DH_Matrix.subs({theta: q1, alpha: alpha0, a: a0, d: d1})
T0_1 = T0_1.subs(s)
T1_2 = DH_Matrix.subs({theta: q2, alpha: alpha1, a: a1, d: d2})
T1_2 = T1_2.subs(s)
T2_3 = DH_Matrix.subs({theta: q3, alpha: alpha2, a: a2, d: d3})
T2_3 = T2_3.subs(s)
T3_4 = DH_Matrix.subs({theta: q4, alpha: alpha3, a: a3, d: d4})
T3_4 = T3_4.subs(s)
T4_5 = DH_Matrix.subs({theta: q5, alpha: alpha4, a: a4, d: d5})
T4_5 = T4_5.subs(s)
T5_6 = DH_Matrix.subs({theta: q6, alpha: alpha5, a: a5, d: d6})
T5_6 = T5_6.subs(s)
T6_7 = DH_Matrix.subs({theta: q7, alpha: alpha6, a: a6, d: d7})
T6_7 = T6_7.subs(s)
T0_6 = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6
T0_7 = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 *T6_7

# Define gamma and its symbols, which is the unit vector pointing 
# from the wrist center to the ee. Will use it in our IK solver
ROLL,PITCH,YAW = symbols('ROLL PITCH YAW')
gamma = Matrix([cos(YAW)*cos(PITCH), sin(YAW)*cos(PITCH), -sin(PITCH)])

R0_3 = (T0_1*T1_2*T2_3)[0:3,0:3]
R0_3T = R0_3.T
Rrpy = zRot * yRot * xRot

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
		
 
        #Moved FK and IK code out of this function as it does not need to run
        #every time a request is recieved, it just needs to run once

        # Initialize service response

        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            ##Gripper Rotation
            rpy = {ROLL:roll, PITCH:pitch, YAW:yaw}
            gamma.subs(rpy)
            ##Gripper Position
            pos = Matrix([px, py, pz])
            ##Wrist center calculation
            wc = pos - s[d7] * gamma.subs(rpy) 


            ## Theta1 #############################################################################
            theta1 = atan2(wc[1],wc[0])
            #-------------------------------------------------------------------------------------- 

            ## Theta2 ############################################################################# 
            ## Calculate the l35, 123, l25 lengths, and V25 vector using wrist center - joint2 position, 
            ## and then take it's norm to find length 
            l35 = sqrt(s[a3]*s[a3] + s[d4]*s[d4])
            l23 = s[a2]
            v25 = wc - Matrix([s[a1]*cos(theta1), s[a1]*sin(theta1), s[d1]])
            l25 = sqrt(v25[0]*v25[0]+v25[1]*v25[1]+v25[2]*v25[2])

            ## NOW CALCLUATE rho and alpha (from diagram 5) so that we can calculate theta2 
            ## angle using the formula theta2 = pi/2-alpha-rho. Note that in the writeup for
            ## determing ll the z terms were d1, but since they were both d1, I just use 0 below.
            ll = Matrix([wc[0],wc[1],0]) - s[a1] * Matrix([cos(theta1), sin(theta1),0]) 
            rho = atan2(wc[2]-s[d1], sqrt(ll[0]*ll[0] + ll[1]*ll[1] + ll[2]*ll[2]))

            # Determine cos_alpha and use it to derive alpha.Note that this alpha is 
            # NOT the same alpha used in the DH table.
            cos_alpha = ((l35*l35 - l23*l23 - l25*l25) / (-2 * l23 * l25))
            alpha = atan2(sqrt(1 - cos_alpha*cos_alpha), cos_alpha)

            # Now that we have rho and alpha, we can calculate theta2
            theta2 = pi/2 -alpha -rho 
            #-------------------------------------------------------------------------------------- 

            ## Theta3 #############################################################################
            # Now calculate gamma (The angle between l23 and l35)   
            cos_gamma = (l25*l25 - l35*l35 - l23*l23) / (-2 * l35 * l23)
            # Then calculate theta3. The formular represented below is pi/2 - gamma - phi
            theta3 = pi/2 - atan2(sqrt(1 - cos_gamma*cos_gamma), cos_gamma) + atan2(s[a3],s[d4]) 
            #--------------------------------------------------------------------------------------

            ## Spherical Wrist Euler Angles #######################################################
            R0_3Tn = R0_3T.subs({q1:theta1, q2:theta2, q3:theta3})
            Rrpyn = Rrpy.subs({theta_z: rpy[YAW], theta_y: rpy[PITCH], theta_x: rpy[ROLL]})
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            Rrpyn = Rrpyn * cRotn[:3,:3]
            RHS=(R0_3Tn*Rrpyn)
            theta4 = atan2(RHS[2,2],-RHS[0,2])
            theta5 = atan2(sqrt(RHS[2,2]*RHS[2,2] + RHS[0,2]*RHS[0,2]), RHS[1,2]) 
            theta6 = -atan2(RHS[1,1],RHS[1,0])
            
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            if x and (len(req.poses)-x)>4:
                if abs(theta6-prev_theta6)>1.and abs(theta4-prev_theta4)>1. and abs(theta6 + theta4) < .1:
                    print("Theta4 / Theta6 Cancel",theta4,theta6)
                    theta4 = prev_theta4
                    theta6 = prev_theta6
            prev_theta4, prev_theta6 = theta4,theta6
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
