import numpy as np
import matplotlib.pyplot as plt
from  ctypes import *
import serial 
import binascii
import time
import struct
import threading

from kf_ahrs import *

path = '/home/hhy/work/test/ekf/libinsdisk.so'
insdisk = cdll.LoadLibrary(path)

plt.ion()

class insdisk_cd(Structure):
	_fields_ = [('yaw',c_float),('pitch', c_float),('roll', c_float),('alt',c_float),('tempr', c_float),('press', c_float)]

class insdisk_od(Structure):
	_fields_ = [('ax',c_short),('ay', c_short),('az', c_short),('gx',c_short),('gy', c_short),('gz', c_short),('hx',c_short),('hy', c_short),('hz', c_short)]

class insdisk_gps(Structure):
	_fields_ = [('GPS_Altitude',c_float),('Latitude_GPS', c_float),('Longitude_GPS', c_float),('Speed_GPS',c_float),('Course_GPS', c_float),('GPS_STA_Num', c_char)]

class insdisk_data(Structure):
	_fields_ = [('pra_case',c_uint),('cal_data', insdisk_cd),('org_data', insdisk_od),('gps_data', insdisk_gps)]

class sensor_mag_s(Structure):
	_fields_ = [('timestamp',c_uint64),('x', c_float),('y', c_float),('z', c_float)]
class sensor_accel_s(Structure):
	_fields_ = [('timestamp',c_uint64),('x', c_float),('y', c_float),('z', c_float)]	
class sensor_gyro_s(Structure):
	_fields_ = [('timestamp',c_uint64),('x', c_float),('y', c_float),('z', c_float)]	

insdata = insdisk_data()
serialbuf = c_ubyte*1024
buf = serialbuf()
bufc = c_uint()

gyro = sensor_gyro_s()
accel = sensor_accel_s()
mag = sensor_mag_s()

port = "/dev/ttyUSB0"
baudrate = 921600
ser = serial.Serial(port, baudrate, timeout = 0.5)

timef = c_double()

plt_x = []
plt_y = []
max_len = 200

lock = threading.Lock()
def plot_thread():
	global plt_x
	global plt_y
	global max_len
	count = 0
	fortime = 0
	while 1:
		plt.clf()

		lock.acquire()
		if len(plt_x)>max_len:
			plt.plot(range(max_len), plt_x, label="xt")
			plt.plot(range(max_len), plt_y, label="yt")
		else:
			plt.plot(range(len(plt_x)), plt_x, label="xt")
			plt.plot(range(len(plt_x)), plt_y, label="yt")
		plt.ylim(-190, 190)
		plt.legend(loc = 0)
		lock.release()

		plt.pause(0.001)		
		time.sleep(0.001)
		time_t = time.time()
		if time_t-fortime>1:
			# print "plt f=", count
			count = 0
			fortime = time_t
		count = count+1
def main():
	kf = Kf_ahrs()
	kfq = Kf_q()
	kfufq = Kfuf_q()
	ufq = uf_q()
	ufqxyz = uf_q_xyz()
	kf_ahrs_xyz = Kf_ahrs_xyz()
	kf_q_xyz = Kf_q_xyz()

	gyro_time = c_uint64()
	gyro_time = 0
	count = 0
	global plt_x
	global max_len
	fortime = 0
	t1 = threading.Thread(target=plot_thread, name='plot_thread')
	t1.setDaemon(True)
	t1.start()
	gyro_x_av = 0
	gyro_y_av = 0
	gyro_z_av = 0
	accel_x_av = 0
	accel_y_av = 0
	accel_z_av = 0
	while 1:
		# cnt = ser.inWaiting()
		time.sleep(0.001)
		cnt =50
		if cnt>0:
			bufstr = ser.read(cnt)
			# print '>', len(bufstr), bufstr
			"""
			1. buf += ser.read(cnt)
			2. msg_list = buf.split("\xa5\x5a")
			3. for msg in msg_list[:-1]:process_msg(msg)
			4. buf = msg[-1]

			process_msg(msg)
			msg_id = ord(msg[0])
			if(msg_id == ACC_RAW):
				xxxxx
			"""
			bufstr_len = len(bufstr)
			# print "len = ", bufstr_len
			if bufstr_len>1024:
				bufstr_len = 1024
			# bytebuf = map(lambda ch:ord(ch), bufstr)
			for k in range(0, bufstr_len):
				# buf[k] = ord(bufstr[k])
				buf[k], = struct.unpack("<B", bufstr[k])
			bufc = cnt

			ret  = insdisk.insdisk_parsing(bufc, byref(buf), byref(insdata))
			if ret==3:
				insdisk.gyro_get(byref(gyro))
				insdisk.accel_get(byref(accel))
				insdisk.mag_get(byref(mag))
				# print gyro.x

				temp = (gyro.timestamp-gyro_time)/1000000.0
				# gyro.x = gyro_x_av = (gyro.x+gyro_x_av)/2.0
				# gyro.y = gyro_y_av = (gyro.y+gyro_y_av)/2.0
				# gyro.z = gyro_z_av = (gyro.z+gyro_z_av)/2.0
				# accel.x = accel_x_av = (accel.x+accel_x_av)/2.0
				# accel.y = accel_y_av = (accel.y+accel_y_av)/2.0
				# accel.z = accel_z_av = (accel.z+gyro_z_av)/2.0

				# ag = kf.updata([gyro.x*temp,gyro.y*temp,gyro.z*temp], [accel.x,accel.y, accel.z])
				# ag = kfq.updata([gyro.x*temp,gyro.y*temp,gyro.z*temp], [accel.x,accel.y, accel.z])
				# ag = kfufq.updata([gyro.x,gyro.y,gyro.z], temp, [accel.x,accel.y, accel.z])
				# ag = ufq.updata([gyro.x,gyro.y,gyro.z], temp, [accel.x,accel.y, accel.z])
				# ag = ufqxyz.updata([gyro.x,gyro.y,gyro.z], temp, [accel.x,accel.y, accel.z], [mag.x,mag.y, mag.z])
				# ag = kf_ahrs_xyz.updata([gyro.x*temp,gyro.y*temp,gyro.z*temp], [accel.x,accel.y, accel.z], [mag.x,mag.y, mag.z])
				ag = kf_q_xyz.updata([gyro.x*temp,gyro.y*temp,gyro.z*temp], [accel.x,accel.y, accel.z], [mag.x,mag.y, mag.z])
				gyro_time = gyro.timestamp

				lock.acquire()
				plt_x.append(ag[0]*180/3.1415926)
				plt_y.append(ag[2]*180/3.1415926)
				# plt.clf()
				if len(plt_x)>max_len:
					plt_x.pop(0)
					plt_y.pop(0)
					# plt.plot(range(max_len), plt_x)
				# else:
					# plt.plot(range(len(plt_x)), plt_x)
				# plt.pause(0.001)	
				lock.release()

				time_t = time.time()
				if time_t-fortime>1:
					# print count
					count = 0
					fortime = time_t
				count = count+1


if __name__ == '__main__':
	main()