#!/usr/bin/env python2.7
import numpy as np
import rospy
import time
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from mavros_msgs.srv import CommandTOL, SetMode, CommandBool
from mavros_msgs.msg import AttitudeTarget, PositionTarget
from geometry_msgs.msg import PoseStamped, Pose, Point, Twist, TwistStamped, Wrench, Vector3
from gazebo_msgs.srv import ApplyBodyWrench
from time import sleep
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import pickle


## IMPORTANT!! -> This code doesn't take into account of the transformation of angles and body rates yet. 
## 			   -> So, only use the linear acceleration commands for now.


def euler_to_quaternion(r):
    (yaw, pitch, roll) = (r[0], r[1], r[2])
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

def quaternion_to_euler(q):
    w, x, y, z = q[0],q[1], q[2], q[3]
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    phi = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    theta = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    psi = np.arctan2(t3, t4)

    return np.array([phi, theta, psi]).T


class FLIGHT_CONTROLLER:

    def __init__(self):
		
		#NODE
        rospy.init_node('iris_drone', anonymous = True)

		#SUBSCRIBERS																	
        self.get_position = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.get_pose)
        self.get_linear_vel = rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.get_vel)
        self.get_imu_data_acc = rospy.Subscriber('/mavros/imu/data', Imu, self.get_acc)
        self.get_imu_attitude = rospy.Subscriber('/mavros/imu/data', Imu, self.get_attitude)

		#PUBLISHERS
        self.publish_pose = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped,queue_size=1)
        self.publish_pos_tar = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget,queue_size=1)

		#SERVICES
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        self.flight_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        rospy.loginfo('INIT')

        self.transform = np.zeros((3, 3))
        time.sleep(2)
        self.transform = self.get_rotmat()

	
    def get_rotmat(self):
        psi_i = self.att[2]
        print(psi_i)
        return np.array([[np.cos(psi_i), -np.sin(psi_i), 0], [np.sin(psi_i), np.cos(psi_i), 0], [0, 0, 1]])

    def toggle_arm(self, arm_bool):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            self.arm_service(arm_bool)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: " %e)

    def takeoff(self, t_alt):
        rospy.wait_for_service('/mavros/cmd/takeoff')
        try:
            self.takeoff_service(0,0,0,0,t_alt)
            rospy.loginfo('TAKEOFF')
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: " %e)
	
	
    def land(self, l_alt):
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            self.land_service(0.0, 0.0, 0, 0, l_alt)
            rospy.loginfo("LANDING")
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: " %e)


    def set_mode(self,md):
            rospy.wait_for_service('/mavros/set_mode')
            try:
                self.flight_mode_service(0, md)
                rospy.loginfo("Mode changed")	
            except rospy.ServiceException as e:
                rospy.loginfo("Mode could not be set: " %e)

    def set_Guided_mode(self):
        rate = rospy.Rate(20)
        PS = PoseStamped()
        PS.pose.position.x = self.pose[0]
        PS.pose.position.y = self.pose[1]
        PS.pose.position.z = self.pose[2]
        for i in range(10):
            self.publish_pose.publish(PS)	
            rate.sleep()
        print('done')
        self.set_mode("GUIDED")

    def get_pose(self, location_data):
        pose = location_data.pose.position
        self.pose = np.dot(self.transform.T, np.array([pose.x, pose.y, pose.z]).T)

    def get_vel(self, vel_data):
        vel = vel_data.twist.linear
        self.vel = np.dot(self.transform.T, np.array([vel.x, vel.y, vel.z]).T)

    def get_acc(self, imu_data_acc):
        acc = imu_data_acc.linear_acceleration
        self.acc = np.dot(self.transform.T, np.array([acc.x, acc.y, acc.z - 9.80665]).T)

    def get_attitude(self, imu_attitude):
        att_q = np.array([imu_attitude.orientation.w, imu_attitude.orientation.x, imu_attitude.orientation.y, imu_attitude.orientation.z])
        self.att = quaternion_to_euler(att_q)


	#PUBLISHERS
	
    def gotopose(self, pose_tar):
		
        rate = rospy.Rate(20)
        sp = PoseStamped()
        pose_tar_mod = np.dot(self.transform, pose_tar)

        sp.pose.position.x = pose_tar_mod[0]
        sp.pose.position.y = pose_tar_mod[1]
        sp.pose.position.z = pose_tar_mod[2]

        sp.pose.orientation.x = 0.0
        sp.pose.orientation.y = 0.0
        sp.pose.orientation.z = 0.0
        sp.pose.orientation.w = 1.0

        dist = np.sqrt(((self.pose[0] - pose_tar[0])**2) + ((self.pose[1] - pose_tar[1])**2) + ((self.pose[2] - pose_tar[2])**2))

        while(dist > 0.2):
            self.publish_pose.publish(sp)
            dist = np.sqrt(((self.pose[0] - pose_tar[0])**2) + ((self.pose[1] - pose_tar[1])**2) + ((self.pose[2] - pose_tar[2])**2))
            rate.sleep()
		
        #print('Reached ',x,y,z)


    def set_pos(self, a):
		
        sp = PositionTarget()
        sp.coordinate_frame = 1
        sp.type_mask = 3135
        a_mod = np.dot(self.transform, a)

        sp.acceleration_or_force.x = a_mod[0]
        sp.acceleration_or_force.y = a_mod[1]
        sp.acceleration_or_force.z = a_mod[2]

        self.publish_pos_tar.publish(sp)


if __name__ == '__main__':

    mav = FLIGHT_CONTROLLER()
    time.sleep(3)
    mav.set_mode('STABILIZE')
    mav.toggle_arm(1)
    time.sleep(2)
    mav.set_Guided_mode()
    mav.takeoff(4)
    time.sleep(3)
    mav.gotopose(np.array([4,0,4]).T)
    time.sleep(3)
    mav.land(4)
	
