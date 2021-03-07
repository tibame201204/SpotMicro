import time
import threading
from adafruit_servokit import ServoKit
import os
url = os.path.dirname(os.path.abspath(__file__))
os.chdir(url) 
import sys
from math import pi
class MyServo():
	def __init__(self):
		self.kit = ServoKit(channels=16)
		self.angle_scale_factor_lf1 =  3/3 #足
		self.angle_scale_factor_lf2 =  3/3 #臂
		self.angle_scale_factor_lf3 =  3/3 #肩
		self.angle_scale_factor_rf1 =  3/3 #足
		self.angle_scale_factor_rf2 =  3/3 #臂
		self.angle_scale_factor_rf3 =  3/3 #肩
		self.angle_scale_factor_rr1 =  3/3 #足
		self.angle_scale_factor_rr2 =  3/3 #臂
		self.angle_scale_factor_rr3 =  3/3 #肩
		self.angle_scale_factor_lr1 =  3/3 #足
		self.angle_scale_factor_lr2 =  3/3 #臂
		self.angle_scale_factor_lr3 =  3/3 #肩
		self.dir_lf1 = 1
		self.dir_lf2 = 1
		self.dir_lf3 = 1
		self.dir_rf1 = 1
		self.dir_rf2 = 1
		self.dir_rf3 = 1
		self.dir_rr1 = 1
		self.dir_rr2 = 1
		self.dir_rr3 = 1
		self.dir_lr1 = 1
		self.dir_lr2 = 1
		self.dir_lr3 = 1
		self.zero_lf1 = 0
		self.zero_lf2 = 0
		self.zero_lf3 = 0
		self.zero_rf1 = 0
		self.zero_rf2 = 0
		self.zero_rf3 = 0
		self.zero_rr1 = 0
		self.zero_rr2 = 0
		self.zero_rr3 = 0
		self.zero_lr1 = 0
		self.zero_lr2 = 0
		self.zero_lr3 = 0

	def servo(self, channel, angle_list):
		for angle in angle_list:
			self.kit.servo[int(channel-1)].angle = angle
			time.sleep(0.5)

	def modservo(self, channel, degree, joint):
		print(degree)
		factor_ = eval('self.angle_scale_factor_%s' % joint)
		dir_ = eval('self.dir_%s' % joint)
		zero_ = eval('self.zero_%s' % joint)
		real_angle = degree / pi * 180 * factor_ *  dir_ + zero_
		print(real_angle)
		try:
			self.kit.servo[channel-1].angle = real_angle
			return real_angle
		except ValueError:
			if real_angle >= 180:
				print("%s Angle out of range, set angle to 180" % joint)
				self.kit.servo[channel-1].angle = 180
				return 180
			elif real_angle <= 0:
				print("%s Angle out of range, set angle to 0" % joint)
				self.kit.servo[channel-1].angle = 0
				return 0
			

	def motion(self):
		# 把線程加入清單
		threads = []
		# threads.append(threading.Thread(target = self.servo, args = (0, [90, 0, 45, 0])))
		threads.append(threading.Thread(target = self.servo, args = (4, [0, 90, 0, 45])))
		# 開啟多線程
		for i in range(len(threads)):
			threads[i].start()
		# 等待所有子執行緒結束
		for i in range(len(threads)):
			threads[i].join()
		print("motion Done...")


if __name__ == '__main__':
	spot = MyServo()
	spot.servo(2, [180])
	spot.servo(4, [0])
	spot.servo(1, [90, 80, 70, 80, 90, 100, 110, 100, 90])
	spot.servo(2, [170, 160, 150, 140, 130, 120, 110, 100])
	spot.servo(4, [120])
	spot.servo(2, [110, 120, 130, 140, 150, 160, 170, 180])
	spot.servo(4, [0])
	
	# spot.modservo(2, 1.14, 'lf2')

	# angle_list = []
	# if len(sys.argv) >= 3:
	# 	for i in range(2, len(sys.argv)):
	# 		angle_list.append(int(sys.argv[i]))
	# 	MyServo().servo(int(sys.argv[1]), angle_list)
	# else:
	# 	print('Please input yours servo channel number and angles !!!')

	# MyServo().motion()
