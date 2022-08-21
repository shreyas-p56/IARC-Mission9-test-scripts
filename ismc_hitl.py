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
		self.disp = np.array([0,0,0]).T
		time.sleep(2)
		self.transform = self.get_rotmat()
		print(self.transform)
		print(self.disp)
		self.disp = np.dot(self.transform, self.pose)
		print(self.disp)
		self.disp = np.array([2,3,4])

	
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
		self.pose = np.dot(self.transform.T, np.array([pose.x, pose.y, pose.z]).T - self.disp)

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
		pose_tar_mod = self.disp + np.dot(self.transform, pose_tar)

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


class smc:
	
	def __init__(self, m, alpha_1, alpha_2, beta, d_p, dt):
		
		self.g = np.array([0 , 0 , -9.80665]).T				#gravity
		self.m = m											#mass
		self.alpha_1 = alpha_1
		self.alpha_2 = alpha_2
		self.beta = beta
		self.d_p = d_p
		self.dt = dt

	def controller(self, p, p_dot, p_d, p_dot_d, p_ddot_d, s_int):

		# Numerical Integration
		self.p_e_u = p_d - p
		self.v_e_u = p_dot_d - p_dot		
		self.s_0_u = self.alpha_1*s_int + self.alpha_2*self.p_e_u + self.v_e_u

		E = self.alpha_1*self.p_e_u + self.alpha_2*self.v_e_u - self.g + p_ddot_d + np.dot(self.d_p,p_dot)/self.m

		sat_s = np.array([0,0,0]).T

		if((self.s_0_u[0] > -0.05) and (self.s_0_u[0] < 0.05)):
			sat_s[0] = 0.05*self.s_0_u[0]/0.05
		elif((self.s_0_u[0] > -0.3) and (self.s_0_u[0] < 0.3)):
			sat_s[0] = 0.7*self.s_0_u[0]/0.3
		else:
			sat_s[0] = 0.85*np.sign(self.s_0_u[0])

		if((self.s_0_u[1] > -0.3) and (self.s_0_u[1] < 0.3)):
			sat_s[1] = 0.7*self.s_0_u[1]/0.3
		elif((self.s_0_u[1] > -0.8) and (self.s_0_u[1] < 0.8)):
			sat_s[1] = 1.1*self.s_0_u[1]/0.8
		else:
			sat_s[1] = 1.5*np.sign(self.s_0_u[1])
			
		if((self.s_0_u[2] > -0.01) and (self.s_0_u[2] < 0.01)):
			sat_s[2] = 0.1*self.s_0_u[2]/0.01
		elif((self.s_0_u[2] > -0.08) and (self.s_0_u[2] < 0.08)):
			sat_s[2] = 0.75*self.s_0_u[2]/0.08
		else:
			sat_s[2] = 0.95*np.sign(self.s_0_u[2])

		E_hat = E + sat_s
		self.p_ddot_c = (E_hat + self.g - np.dot(self.d_p,p_dot)/self.m)

if __name__ == '__main__':

	#unpickle the trajectory from its file

	pickle_off = open("Documents/IARC/5D10_t.txt", "rb")			#modify the first input in open() to be the location of the trajectory file, and default is in home
	gen = pickle.load(pickle_off)

	[x_path, x_dot_path, x_ddot_path, y_path, y_dot_path, y_ddot_path, z_path, z_dot_path, z_ddot_path, psi_path] = gen


	mav = FLIGHT_CONTROLLER()
	time.sleep(3)
	mav.set_mode('STABILIZE')
	mav.toggle_arm(1)
	time.sleep(2)
	mav.set_Guided_mode()
	mav.takeoff(z_path[0])
	time.sleep(3)
	#mav.gotopose(np.array([x_path[0],y_path[0],z_path[0]]).T)
	
	m = 1.5													# mass of the quadrotor
	alpha_1 = 0.25											
	alpha_2 = 0.9											
	beta = np.array([0.1,0.5,2,7])
	J_p = np.diag(np.array([4.9e-3, 4.9e-3, 8.8e-3]))		# coefficients of the rotary inertia
	Tau_n = np.array([1, 1, 1]).T							# moments in the body-fixed frame
	d_p = np.diag(np.array([0.00,0.00,0.00]))				# air drag
	d_eta = np.diag(np.array([6e-5,6e-5,6e-5]))				# aerodynamic drag coefficients

	Smc = smc(m, alpha_1, alpha_2, beta, d_p, dt=0.1)

	p = mav.pose
	p_dot = mav.vel
	p_ddot = mav.acc
		
	p_e = np.array([x_path[0], y_path[0], z_path[0]]).T - p
	v_e = np.array([x_dot_path[0], y_dot_path[0], z_dot_path[0]]).T - p_dot
	s_0 = alpha_2*p_e + v_e
	s_int = 0

	p_prev = p

	rate = rospy.Rate(10)
	irate = rospy.Rate(30)

	for iter in range(len(z_path)):
		
		p_d = np.array([x_path[iter], y_path[iter], z_path[iter]]).T
		p_dot_d = np.array([x_dot_path[iter], y_dot_path[iter], z_dot_path[iter]]).T
		p_ddot_d = np.array([x_ddot_path[iter], y_ddot_path[iter], z_ddot_path[iter]]).T
		psi_d = np.arctan2(p_dot_d[1],p_dot_d[0])

		Smc.controller(p, p_dot, p_d, p_dot_d, p_ddot_d, s_int)
		
		for i in range (3):
			mav.set_pos(Smc.p_ddot_c)
			p_inst = mav.pose										#instantaneous position
			s_int += (2*p_d - p_inst - p_prev)/60					#integral of position error

			p_prev = p_inst
			irate.sleep()

		p = mav.pose
		p_dot = mav.vel
		p_ddot = mav.acc

		p_e = np.copy(Smc.p_e_u)
		v_e = np.copy(Smc.v_e_u)
		s_0 = np.copy(Smc.s_0_u)

	time.sleep(2)
	mav.land(z_path[0])
