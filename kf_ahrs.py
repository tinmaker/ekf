import numpy as np
import matplotlib.pyplot as plt
from  ctypes import *
import serial 
import binascii
import time
import struct
import threading

path = '/home/hhy/work/test/ekf/libinsdisk.so'
insdisk = cdll.LoadLibrary(path)

plt.ion()

class uf_q():
	"""docstring for uf_q"""
	def __init__(self):
		# self.Xt = np.mat([1.0, 0.0, 0.0, 0.0]).T
		# self.Ft = np.mat([[0 ,0 ,0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0 ,0 ,0, 0]])
		self.q0 = 1.0
		self.q1 = 0.0
		self.q2 = 0.0
		self.q3 = 0.0
		self.exInt = 0.0
		self.eyInt = 0.0
		self.ezInt = 0.0
		self.Ki = 0.01
		self.Kp = 2.0
		self.M_PI = 3.1415926
	def updata(self, gyro, dt, accel):
		angle = [0, 0]
		if accel==[0,0,0]:
			return angle
		m = np.sqrt(accel[0]**2+accel[1]**2+accel[2]**2)
		ax = accel[0]/m
		ay = accel[1]/m
		az = accel[2]/m
		# print "accel[]",accel 
		ax_ = 2*(self.q1*self.q3 - self.q0*self.q2)
		ay_ = 2*(self.q0*self.q1 + self.q2*self.q3)
		az_ = self.q0*self.q0 - self.q1*self.q1 - self.q2*self.q2 + self.q3*self.q3

		ex = (ay*az_ - az*ay_)
		ey = (az*ax_ - ax*az_)
		ez = (ax*ay_ - ay*ax_)
		self.exInt = self.exInt + ex * self.Ki * dt
		self.eyInt = self.eyInt + ey * self.Ki * dt
		self.ezInt = self.ezInt + ez * self.Ki * dt
		gx = gyro[0] + self.Kp*ex + self.exInt
		gy = gyro[1] + self.Kp*ey + self.eyInt
		gz = gyro[2] + self.Kp*ez + self.ezInt
		tempq0 = self.q0 + (-self.q1*gx - self.q2*gy - self.q3*gz)*dt
		tempq1 = self.q1 + (self.q0*gx + self.q2*gz - self.q3*gy)*dt
		tempq2 = self.q2 + (self.q0*gy - self.q1*gz + self.q3*gx)*dt
		tempq3 = self.q3 + (self.q0*gz + self.q1*gy - self.q2*gx)*dt
		m = np.sqrt(tempq0**2+tempq1**2+tempq2**2+tempq3**2)

		self.q0 = tempq0/m
		self.q1 = tempq1/m
		self.q2 = tempq2/m
		self.q3 = tempq3/m
		
		angle[0] = np.arctan2(2 * self.q0 * self.q1 + 2 * self.q2 * self.q3, self.q0**2 - self.q1**2 - self.q2**2 + self.q3**2)
  		angle[1] = np.arcsin(-2 * self.q1 * self.q3 + 2 * self.q0 * self.q2)
  		return angle


class uf_q_xyz():
	"""docstring for uf_q"""
	def __init__(self):
		# self.Xt = np.mat([1.0, 0.0, 0.0, 0.0]).T
		# self.Ft = np.mat([[0 ,0 ,0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0 ,0 ,0, 0]])
		self.q0 = 1.0
		self.q1 = 0.0
		self.q2 = 0.0
		self.q3 = 0.0
		self.exInt = 0.0
		self.eyInt = 0.0
		self.ezInt = 0.0
		self.Ki = 0.01
		self.Kp = 2.0
		self.M_PI = 3.1415926
	def updata(self, gyro, dt, accel, mag):
		angle = [0, 0,0]
		if accel==[0,0,0]:
			return angle
		m = np.sqrt(accel[0]**2+accel[1]**2+accel[2]**2)
		ax = accel[0]/m
		ay = accel[1]/m
		az = accel[2]/m
		m = np.sqrt(mag[0]**2+mag[1]**2+mag[2]**2)
		mx = mag[0]/m
		my = mag[1]/m
		mz = mag[2]/m

		hx = 2*mx*(0.5 - self.q2*self.q2 - self.q3*self.q3) + 2*my*(self.q1*self.q2 - self.q0*self.q3) + 2*mz*(self.q1*self.q3 + self.q0*self.q2)
		hy = 2*mx*(self.q1*self.q2 + self.q0*self.q3) + 2*my*(0.5 - self.q1*self.q1 - self.q3*self.q3) + 2*mz*(self.q2*self.q3 - self.q0*self.q1)
		hz = 2*mx*(self.q1*self.q3 - self.q0*self.q2) + 2*my*(self.q2*self.q3 + self.q0*self.q1) + 2*mz*(0.5 - self.q1*self.q1 - self.q2*self.q2)     
		bx = np.sqrt((hx*hx) + (hy*hy))
		bz = hz

		wx = 2*bx*(0.5 - self.q2*self.q2 - self.q3*self.q3) + 2*bz*(self.q1*self.q3 - self.q0*self.q2)
		wy = 2*bx*(self.q1*self.q2 - self.q0*self.q3) + 2*bz*(self.q0*self.q1 + self.q2*self.q3)
		wz = 2*bx*(self.q0*self.q2 + self.q1*self.q3) + 2*bz*(0.5 - self.q1*self.q1 - self.q2*self.q2); 

		# print "accel[]",accel 
		ax_ = 2*(self.q1*self.q3 - self.q0*self.q2)
		ay_ = 2*(self.q0*self.q1 + self.q2*self.q3)
		az_ = self.q0*self.q0 - self.q1*self.q1 - self.q2*self.q2 + self.q3*self.q3

		ex = (ay*az_ - az*ay_) + (my*wz - mz*wy)
		ey = (az*ax_ - ax*az_) + (mz*wx - mx*wz)
		ez = (ax*ay_ - ay*ax_) + (mx*wy - my*wx)

		self.exInt = self.exInt + ex * self.Ki * dt
		self.eyInt = self.eyInt + ey * self.Ki * dt
		self.ezInt = self.ezInt + ez * self.Ki * dt
		gx = gyro[0] + self.Kp*ex + self.exInt
		gy = gyro[1] + self.Kp*ey + self.eyInt
		gz = gyro[2] + self.Kp*ez + self.ezInt

		tempq0 = self.q0 + (-self.q1*gx - self.q2*gy - self.q3*gz)*dt
		tempq1 = self.q1 + (self.q0*gx + self.q2*gz - self.q3*gy)*dt
		tempq2 = self.q2 + (self.q0*gy - self.q1*gz + self.q3*gx)*dt
		tempq3 = self.q3 + (self.q0*gz + self.q1*gy - self.q2*gx)*dt
		m = np.sqrt(tempq0**2+tempq1**2+tempq2**2+tempq3**2)

		self.q0 = tempq0/m
		self.q1 = tempq1/m
		self.q2 = tempq2/m
		self.q3 = tempq3/m
		
		angle[0] = np.arctan2(2 * self.q0 * self.q1 + 2 * self.q2 * self.q3, self.q0**2 - self.q1**2 - self.q2**2 + self.q3**2)
  		angle[1] = np.arcsin(-2 * self.q1 * self.q3 + 2 * self.q0 * self.q2)
		angle[2] = np.arctan2(2 * self.q0 * self.q3 + 2 * self.q1 * self.q2, self.q0**2 + self.q1**2 - self.q2**2 - self.q3**2)

  		return angle


class Kf_q2():
	"""docstring for Kf_q"""
	def __init__(self):
		self.Ft = np.mat([[0 ,0 ,0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0 ,0 ,0, 0]])
		self.Pt = np.mat([[0.0001 ,0 ,0, 0 ,0 ,0, 0],\
									[0, 0.0001, 0, 0 ,0 ,0, 0], \
									[0, 0, 0.0001, 0 ,0 ,0, 0], \
									[0, 0, 0, 0.0001 ,0 ,0, 0], \
									[0, 0, 0, 0 ,0.0002 ,0, 0], \
									[0, 0, 0, 0 ,0 ,0.0002, 0], \
									[0, 0, 0, 0 ,0 ,0, 0.0002]])
		self.Q = np.mat([[0.0001 ,0 ,0, 0 ,0 ,0, 0],\
									[0, 0.0001, 0, 0 ,0 ,0, 0], \
									[0, 0, 0.0001, 0 ,0 ,0, 0], \
									[0, 0, 0, 0.0001 ,0 ,0, 0], \
									[0, 0, 0, 0 ,0.0005 ,0, 0], \
									[0, 0, 0, 0 ,0 ,0.0005, 0], \
									[0, 0, 0, 0 ,0 ,0, 0.0005]])
		self.Ht = np.mat([[0 ,0 ,0 ,0], [0, 0, 0 ,0], [0, 0, 0 ,0]])
		self.I = np.mat([[1 ,0 ,0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
		self.R = np.mat([0.000449, 0.0002039 ,0.0002345]).T
		self.M_PI = 3.1415926
		self.G = 9.8
	def updata(self, deltaAngle, accel):
		self.Ft	= np.mat([	[1, -deltaAngle[0], -deltaAngle[1], -deltaAngle[2]],\
										[deltaAngle[0], 1, deltaAngle[2], -deltaAngle[1]], \
			 							[deltaAngle[1], -deltaAngle[2], 1, deltaAngle[0]], \
			 							[deltaAngle[2], deltaAngle[1], -deltaAngle[0], 1] ])
		self.Ht = self.G*np.mat([[-self.Xt[2,0], self.Xt[3,0], -self.Xt[0,0], self.Xt[1,0]], \
												[self.Xt[1,0], self.Xt[0,0], self.Xt[3,0], self.Xt[2,0]], \
												[self.Xt[0,0], -self.Xt[1,0], -self.Xt[2,0], self.Xt[3,0]]])

		self.Zt = np.mat(accel).T

		X_ = self.Ft*self.Xt
		m = np.sqrt(X_[0,0]**2+X_[1,0]**2+X_[2,0]**2+X_[3,0]**2)
		X_[0,0] /= m
		X_[1,0] /= m
		X_[2,0] /= m
		X_[3,0] /= m

		P_ = self.Ft*self.Pt*self.Ft.T+self.Q
		K = P_*self.Ht.T*(self.Ht*P_*self.Ht.T+self.R).I
		self.Xt = X_ + K*(self.Zt - self.Ht*X_)
		self.Pt = (self.I - K*self.Ht)*P_
		m = np.sqrt(self.Xt[0,0]**2+self.Xt[1,0]**2+self.Xt[2,0]**2+self.Xt[3,0]**2)
		self.Xt[0,0] /= m
		self.Xt[1,0] /= m
		self.Xt[2,0] /= m
		self.Xt[3,0] /= m

		angle = [0, 0]
		angle[0] = -np.arctan2(2 * self.Xt[1,0] * self.Xt[2,0] + 2 * self.Xt[0,0] * self.Xt[3,0], -2 * self.Xt[2,0]*self.Xt[2,0] - 2 * self.Xt[3,0] * self.Xt[3,0] + 1)
  		angle[1] = -np.arcsin(-2 * self.Xt[1,0] * self.Xt[3,0] + 2 * self.Xt[0,0] * self.Xt[2,0])
  		return angle

class Kf_q():
	"""docstring for Kf_q"""
	def __init__(self):
		self.Xt = np.mat([1, 0, 0, 0]).T
		self.Zt = np.mat([0 ,0 ,0]).T
		self.Ft = np.mat([[0 ,0 ,0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0 ,0 ,0, 0]])
		self.Pt = np.mat([[1 ,0 ,0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
		self.Q = np.mat([[0.0000001 ,0, 0, 0], [0, 0.0000001, 0, 0], [0, 0, 0.0000001, 0], [0, 0, 0, 0.0000001]])
		self.Ht = np.mat([[0 ,0 ,0 ,0], [0, 0, 0 ,0], [0, 0, 0 ,0]])
		self.I = np.mat([[1 ,0 ,0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
		# self.R = np.mat([0.0001, 0.0001 ,0.0001]).T
		self.R = np.mat([[0.1, 0 ,0],\
								[0,0.1, 0], \
								[0, 0, 0.1]])
		self.M_PI = 3.1415926
		self.G = 9.8
	def updata(self, deltaAngle, accel):
		self.Ft	= np.mat([	[1, -deltaAngle[0], -deltaAngle[1], -deltaAngle[2]],\
										[deltaAngle[0], 1, deltaAngle[2], -deltaAngle[1]], \
			 							[deltaAngle[1], -deltaAngle[2], 1, deltaAngle[0]], \
			 							[deltaAngle[2], deltaAngle[1], -deltaAngle[0], 1] ])
		self.Ht = self.G*np.mat([[-self.Xt[2,0], self.Xt[3,0], -self.Xt[0,0], self.Xt[1,0]], \
												[self.Xt[1,0], self.Xt[0,0], self.Xt[3,0], self.Xt[2,0]], \
												[self.Xt[0,0], -self.Xt[1,0], -self.Xt[2,0], self.Xt[3,0]]])

		self.Zt = np.mat(accel).T

		X_ = self.Ft*self.Xt
		m = np.sqrt(X_[0,0]**2+X_[1,0]**2+X_[2,0]**2+X_[3,0]**2)
		X_ /= m

		P_ = self.Ft*self.Pt*self.Ft.T+self.Q
		K = P_*self.Ht.T*(self.Ht*P_*self.Ht.T+self.R).I
		self.Xt = X_ + K*(self.Zt - self.Ht*X_)
		self.Pt = (self.I - K*self.Ht)*P_
		# self.Xt = X_
		m = np.sqrt(self.Xt[0,0]**2+self.Xt[1,0]**2+self.Xt[2,0]**2+self.Xt[3,0]**2)
		self.Xt[0,0] /= m
		self.Xt[1,0] /= m
		self.Xt[2,0] /= m
		self.Xt[3,0] /= m

		angle = [0, 0]
		angle[0] = np.arctan2(2 * self.Xt[0,0] * self.Xt[1,0] + 2 * self.Xt[2,0] * self.Xt[3,0], self.Xt[0,0]**2 - self.Xt[1,0]**2 - self.Xt[2,0]**2 + self.Xt[3,0]**2)
  		angle[1] = np.arcsin(-2 * self.Xt[1,0] * self.Xt[3,0] + 2 * self.Xt[0,0] * self.Xt[2,0])
  		return angle


class Kf_q_xyz():
	"""docstring for Kf_q"""
	def __init__(self):
		self.Xt = np.mat([1, 0, 0, 0]).T
		self.Zt = np.mat([0 ,0 ,0]).T
		self.Ft = np.mat([[0 ,0 ,0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0 ,0 ,0, 0]])
		self.Pt = np.mat([[1 ,0 ,0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
		self.Q = np.mat([[0.0000001 ,0, 0, 0], [0, 0.0000001, 0, 0], [0, 0, 0.0000001, 0], [0, 0, 0, 0.0000001]])
		# self.Ht = np.mat([[0 ,0 ,0 ,0], [0, 0, 0 ,0], [0, 0, 0 ,0]])
		self.I = np.mat([[1 ,0 ,0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
		# self.R = np.mat([0.0001, 0.0001 ,0.0001]).T
		self.R = np.mat([	[0.001, 0 ,0, 0, 0],\
										[0,0.001, 0, 0, 0], \
										[0, 0, 0.001, 0, 0], \
										[0, 0, 0, 0.0001, 0], \
										[0, 0, 0, 0, 0.0001]])
		self.M_PI = 3.1415926
		self.G = 9.8
	def updata(self, deltaAngle, accel, mag):
		self.Ft	= np.mat([	[1, -deltaAngle[0], -deltaAngle[1], -deltaAngle[2]],\
										[deltaAngle[0], 1, deltaAngle[2], -deltaAngle[1]], \
			 							[deltaAngle[1], -deltaAngle[2], 1, deltaAngle[0]], \
			 							[deltaAngle[2], deltaAngle[1], -deltaAngle[0], 1] ])
		self.Ht = np.mat([[-self.Xt[2,0], self.Xt[3,0], -self.Xt[0,0], self.Xt[1,0]], \
												[self.Xt[1,0], self.Xt[0,0], self.Xt[3,0], self.Xt[2,0]], \
												[self.Xt[0,0], -self.Xt[1,0], -self.Xt[2,0], self.Xt[3,0]], \
												[self.Xt[0,0], self.Xt[1,0], -self.Xt[2,0], -self.Xt[3,0]], \
												[-self.Xt[3,0], self.Xt[2,0], self.Xt[1,0], -self.Xt[0,0]]])

		# m = np.sqrt(mag[0]**2+mag[1]**2+mag[2]**2)
		# mag[0] = mag[0]/m
		# mag[1] = mag[1]/m
		# mag[2] = mag[2]/m
		# print mag0, mag1, mag2
		ang2 = np.arctan2(mag[1], mag[2])
		print ang2
		ang = np.arctan2(mag[0], np.cos(ang2)*(np.sqrt(mag[1]**2+mag[2]**2)))
		
		self.Zt = np.mat([accel[0]/self.G, accel[1]/self.G, accel[2]/self.G, np.sin(ang), np.cos(ang)]).T

		X_ = self.Ft*self.Xt
		m = np.sqrt(X_[0,0]**2+X_[1,0]**2+X_[2,0]**2+X_[3,0]**2)
		X_ /= m

		P_ = self.Ft*self.Pt*self.Ft.T+self.Q
		K = P_*self.Ht.T*(self.Ht*P_*self.Ht.T+self.R).I
		self.Xt = X_ + K*(self.Zt - self.Ht*X_)
		self.Pt = (self.I - K*self.Ht)*P_
		# self.Xt = X_
		m = np.sqrt(self.Xt[0,0]**2+self.Xt[1,0]**2+self.Xt[2,0]**2+self.Xt[3,0]**2)
		self.Xt[0,0] /= m
		self.Xt[1,0] /= m
		self.Xt[2,0] /= m
		self.Xt[3,0] /= m

		angle = [0, 0, 0]
		angle[0] = np.arctan2(2 * self.Xt[0,0] * self.Xt[1,0] + 2 * self.Xt[2,0] * self.Xt[3,0], self.Xt[0,0]**2 - self.Xt[1,0]**2 - self.Xt[2,0]**2 + self.Xt[3,0]**2)
  		angle[1] = np.arcsin(-2 * self.Xt[1,0] * self.Xt[3,0] + 2 * self.Xt[0,0] * self.Xt[2,0])
		angle[2] = np.arctan2(2 * self.Xt[0,0] * self.Xt[3,0] + 2 * self.Xt[1,0] * self.Xt[2,0], self.Xt[0,0]**2 + self.Xt[1,0]**2 - self.Xt[2,0]**2 - self.Xt[3,0]**2)
  		return angle


class Kfuf_q():
	"""docstring for Kf_q"""
	def __init__(self):
		self.Xt = np.mat([1, 0, 0, 0]).T
		self.Zt = np.mat([0 ,0 ,0]).T
		self.Ft = np.mat([[0 ,0 ,0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0 ,0 ,0, 0]])
		self.Pt = np.mat([[1 ,0 ,0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
		self.Q = np.mat([[0.0001 ,0, 0, 0], [0, 0.0001, 0, 0], [0, 0, 0.0001, 0], [0, 0, 0, 0.0001]])
		self.Ht = np.mat([[0 ,0 ,0 ,0], [0, 0, 0 ,0], [0, 0, 0 ,0]])
		self.I = np.mat([[1 ,0 ,0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
		# self.R = np.mat([0.0001, 0.0001 ,0.0001]).T
		self.R = np.mat([[0.00001, 0 ,0],\
								[0,0.00001, 0], \
								[0, 0, 0.00001]])
		self.M_PI = 3.1415926
		self.G = 9.8
		self.exInt = 0.0
		self.eyInt = 0.0
		self.ezInt = 0.0
		self.Ki = 0.01
		self.Kp = 2.0
		self.M_PI = 3.1415926
		self.ke = 100
		self.max_e = 0.01
		self.error_ = 0.001
		self.b = 0
		self.maxb = self.error_ *0.9
	def updata(self, gyro, dt, accel):
		angle = [0, 0]
		if accel==[0,0,0]:
			return angle
		m = np.sqrt(accel[0]**2+accel[1]**2+accel[2]**2)
		ax = accel[0]/m
		ay = accel[1]/m
		az = accel[2]/m

		ax_ = 2*(self.Xt[1,0]*self.Xt[3,0] - self.Xt[0,0]*self.Xt[2,0])
		ay_ = 2*(self.Xt[0,0]*self.Xt[1,0] + self.Xt[2,0]*self.Xt[3,0])
		az_ = self.Xt[0,0]*self.Xt[0,0] - self.Xt[1,0]*self.Xt[1,0] - self.Xt[2,0]*self.Xt[2,0] + self.Xt[3,0]*self.Xt[3,0]

		ex = (ay*az_ - az*ay_)
		ey = (az*ax_ - ax*az_)
		ez = (ax*ay_ - ay*ax_)

		# print ex,ey,ez
		# self.exInt = self.exInt + ex * self.Ki * dt
		# self.eyInt = self.eyInt + ey * self.Ki * dt
		# self.ezInt = self.ezInt + ez * self.Ki * dt

		# gx = gyro[0] - self.Kp*ex - self.exInt
		# gy = gyro[1] - self.Kp*ey - self.eyInt
		# gz = gyro[2] - self.Kp*ez - self.ezInt
		exyz = abs(ex)+abs(ey)+abs(ez)
		if exyz >0.1:
			exyz = 0.1
		if exyz<0.00001:
			exyz = 0.00001

		e_ = 0.1/self.maxb
		self.b = exyz/e_
		bb = 100000
		self.R = np.mat([[self.error_+self.b*bb, 0 ,0],\
								[0,self.error_+self.b*bb, 0], \
								[0, 0, self.error_+self.b*bb]])
		# self.Q = np.mat([[self.error_ - self.b ,0, 0, 0], [0, self.error_ - self.b, 0, 0], [0, 0, self.error_ - self.b, 0], [0, 0, 0, self.error_ - self.b]])

		
		print self.error_+self.b*bb, self.error_-self.b
		# if abs(ex)>self.max_e or abs(ey)>self.max_e or abs(ez)>self.max_e:
		# 	self.R = np.mat([[100, 0 ,0],\
		# 						[0,100, 0], \
		# 						[0, 0, 100]])
		# else:
		# 	self.R = np.mat([[0.00001, 0 ,0],\
		# 						[0,0.00001, 0], \
		# 						[0, 0, 0.00001]])

		# print exyz*self.ke
		gx = gyro[0]
		gy = gyro[1]
		gz = gyro[2]

		deltaAngle = [0,0,0]
		deltaAngle[0] = gx*dt
		deltaAngle[1] = gy*dt
		deltaAngle[2] = gz*dt

		self.Ft	= np.mat([	[1, -deltaAngle[0], -deltaAngle[1], -deltaAngle[2]],\
										[deltaAngle[0], 1, deltaAngle[2], -deltaAngle[1]], \
			 							[deltaAngle[1], -deltaAngle[2], 1, deltaAngle[0]], \
			 							[deltaAngle[2], deltaAngle[1], -deltaAngle[0], 1] ])
		self.Ht = self.G*np.mat([[-self.Xt[2,0], self.Xt[3,0], -self.Xt[0,0], self.Xt[1,0]], \
												[self.Xt[1,0], self.Xt[0,0], self.Xt[3,0], self.Xt[2,0]], \
												[self.Xt[0,0], -self.Xt[1,0], -self.Xt[2,0], self.Xt[3,0]]])

		self.Zt = np.mat(accel).T

		X_ = self.Ft*self.Xt
		m = np.sqrt(X_[0,0]**2+X_[1,0]**2+X_[2,0]**2+X_[3,0]**2)
		X_ /= m

		P_ = self.Ft*self.Pt*self.Ft.T+self.Q
		K = P_*self.Ht.T*(self.Ht*P_*self.Ht.T+self.R).I
		self.Xt = X_ + K*(self.Zt - self.Ht*X_)
		self.Pt = (self.I - K*self.Ht)*P_
		# self.Xt = X_
		m = np.sqrt(self.Xt[0,0]**2+self.Xt[1,0]**2+self.Xt[2,0]**2+self.Xt[3,0]**2)
		self.Xt[0,0] /= m
		self.Xt[1,0] /= m
		self.Xt[2,0] /= m
		self.Xt[3,0] /= m

		angle = [0, 0]
		angle[0] = np.arctan2(2 * self.Xt[0,0] * self.Xt[1,0] + 2 * self.Xt[2,0] * self.Xt[3,0], self.Xt[0,0]**2 - self.Xt[1,0]**2 - self.Xt[2,0]**2 + self.Xt[3,0]**2)
  		angle[1] = np.arcsin(-2 * self.Xt[1,0] * self.Xt[3,0] + 2 * self.Xt[0,0] * self.Xt[2,0])
  		return angle

class Kf_ahrs():
	"""docstring for ClassName"""
	def __init__(self):
		self.Xt = np.mat([0, 0, 1]).T
		self.Zt = np.mat([0 ,0 ,0]).T
		self.Ft = np.mat([[0 ,0 ,0], [0, 0, 0], [0, 0, 0]])
		self.Pt = np.mat([[1 ,0 ,0], [0, 1, 0], [0, 0, 1]])
		self.Q = np.mat([[0.0000001 ,0 ,0], [0, 0.0000001, 0], [0, 0, 0.0000001]])
		self.H = np.mat([[1 ,0 ,0], [0, 1, 0], [0, 0, 1]])
		self.I = np.mat([[1 ,0 ,0], [0, 1, 0], [0, 0, 1]])
		# self.R = np.mat([0.000449, 0.0002039 ,0.0002345]).T
		self.R = np.mat([[0.000449, 0 ,0],\
									[0, 0.0002039, 0],\
									[0, 0, 0.0002345]])
		self.count = 0
	def updata(self, deltaAngle, accel):
		# print self.Xt
		self.count +=1
		self.Ft = self.eulerangle2cosine(deltaAngle[0], deltaAngle[1], deltaAngle[2])
		self.Zt = np.mat(accel).T

		X_ = self.Ft*self.Xt
		P_ = self.Ft*self.Pt*self.Ft.T+self.Q
		K = P_*self.H.T*(self.H*P_*self.H.T+self.R).I
		self.Xt = X_ + K*(self.Zt - self.H*X_)
		self.Pt = (self.I - K*self.H)*P_
		angle = [0, 0]
		# angle[0] = np.arctan2(self.Xt[1, 0], np.sqrt(self.Xt[0, 0]**2+self.Xt[2, 0]**2))
		angle[0] = np.arctan2(self.Xt[1, 0], self.Xt[2, 0])
		angle[1] = np.arctan2(-self.Xt[0, 0], self.Xt[2, 0])
		return angle
	def 	eulerangle2cosine(self, x, y ,z):
		R = [[0 ,0 ,0], [0, 0, 0], [0, 0, 0]]
		R[0][0] = np.cos(z)*np.cos(y)
		R[0][1] = np.sin(z)*np.cos(x)+np.cos(z)*np.sin(x)*np.sin(y)
		R[0][2] = np.sin(z)*np.sin(x)-np.cos(z)*np.cos(x)*np.sin(y)
		R[1][0] = -np.cos(y)*np.sin(z)
		R[1][1] = np.cos(z)*np.cos(x)-np.sin(z)*np.sin(x)*np.sin(y)
		R[1][2] = np.cos(z)*np.sin(x)+np.sin(z)*np.cos(x)*np.sin(y)
		R[2][0] = np.sin(y)
		R[2][1] = -np.sin(x)*np.cos(y)
		R[2][2] = np.cos(y)*np.cos(x)

		return np.mat(R)

class Kf_ahrs_xyz():
	"""docstring for ClassName"""
	def __init__(self):
		self.Xt = np.mat([0, 0, 1]).T
		self.Xmt = np.mat([0, 0, 1]).T
		self.Zt = np.mat([0 ,0 ,0]).T
		self.Zmt = np.mat([0 ,0 ,0]).T
		self.Ft = np.mat([[0 ,0 ,0], [0, 0, 0], [0, 0, 0]])
		self.Pt = np.mat([[1 ,0 ,0], [0, 1, 0], [0, 0, 1]])
		self.Pmt = np.mat([[1 ,0 ,0], [0, 1, 0], [0, 0, 1]])
		self.Q = np.mat([[0.0000001 ,0 ,0], [0, 0.0000001, 0], [0, 0, 0.0000001]])
		self.Qm = np.mat([[0.001 ,0 ,0], [0, 0.001, 0], [0, 0, 0.001]])
		self.H = np.mat([[1 ,0 ,0], [0, 1, 0], [0, 0, 1]])
		self.I = np.mat([[1 ,0 ,0], [0, 1, 0], [0, 0, 1]])
		# self.R = np.mat([0.000449, 0.0002039 ,0.0002345]).T
		self.R = np.mat([[0.000449, 0 ,0],\
									[0, 0.0002039, 0],\
									[0, 0, 0.0002345]])
		self.Rm = np.mat([[250, 0 ,0],\
										[0, 250, 0],\
										[0, 0, 250]])
		self.count = 0
	def updata(self, deltaAngle, accel, mag):
		# print self.Xt
		self.count +=1
		self.Ft = self.eulerangle2cosine(deltaAngle[0], deltaAngle[1], deltaAngle[2])
		self.Zt = np.mat(accel).T
		self.Zmt = np.mat(mag).T

		X_ = self.Ft*self.Xt
		P_ = self.Ft*self.Pt*self.Ft.T+self.Q
		K = P_*self.H.T*(self.H*P_*self.H.T+self.R).I
		self.Xt = X_ + K*(self.Zt - self.H*X_)
		self.Pt = (self.I - K*self.H)*P_

		X_ = self.Ft*self.Xmt
		P_ = self.Ft*self.Pmt*self.Ft.T+self.Qm
		K = P_*self.H.T*(self.H*P_*self.H.T+self.Rm).I
		self.Xmt = X_ + K*(self.Zmt - self.H*X_)
		self.Pmt = (self.I - K*self.H)*P_

		angle = [0, 0, 0]
		# angle[0] = np.arctan2(self.Xt[1, 0], np.sqrt(self.Xt[0, 0]**2+self.Xt[2, 0]**2))
		angle[0] = np.arctan2(self.Xt[1, 0], self.Xt[2, 0])
		angle[1] = np.arctan2(-self.Xt[0, 0], self.Xt[2, 0])
		F = self.eulerangle2cosine(angle[0], angle[1], 0)
		Xm = F.T*self.Xmt
		angle[2] = np.arctan2(-Xm[1, 0], Xm[0, 0])
		return angle
	def 	eulerangle2cosine(self, x, y ,z):
		R = [[0 ,0 ,0], [0, 0, 0], [0, 0, 0]]
		R[0][0] = np.cos(z)*np.cos(y)
		R[0][1] = np.sin(z)*np.cos(x)+np.cos(z)*np.sin(x)*np.sin(y)
		R[0][2] = np.sin(z)*np.sin(x)-np.cos(z)*np.cos(x)*np.sin(y)
		R[1][0] = -np.cos(y)*np.sin(z)
		R[1][1] = np.cos(z)*np.cos(x)-np.sin(z)*np.sin(x)*np.sin(y)
		R[1][2] = np.cos(z)*np.sin(x)+np.sin(z)*np.cos(x)*np.sin(y)
		R[2][0] = np.sin(y)
		R[2][1] = -np.sin(x)*np.cos(y)
		R[2][2] = np.cos(y)*np.cos(x)

		return np.mat(R)

