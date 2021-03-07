import time
import threading
from adafruit_servokit import ServoKit
import os
url = os.path.dirname(os.path.abspath(__file__))
os.chdir(url) 
import sys

class MyServo():
	def __init__(self):
		self.kit = ServoKit(channels=16)

	def servo(self, channel, angle_list):
		for angle in angle_list:
			self.kit.servo[channel-1].angle = angle
			time.sleep(0.5)

	def motion(self):
		# 把線程加入清單
		threads = []
		threads.append(threading.Thread(target = self.servo, args = (0, [90, 0, 45, 0])))
		threads.append(threading.Thread(target = self.servo, args = (3, [0, 90, 0, 45])))
		# 開啟多線程
		for i in range(len(threads)):
			threads[i].start()
		# 等待所有子執行緒結束
		for i in range(len(threads)):
			threads[i].join()
		print("motion Done...")


if __name__ == '__main__':
	int(sys.argv[0])
	int(sys.argv[1])
	angle_list = []
	if len(sys.argv) >= 3:
		for i in range(2, len(sys.argv)):
			angle_list.append(sys.argv[i])
			MyServo().servo(int(sys.argv[1]), angle_list)
	else:
		print('Please input yours servo channel number and angles !!!')
	# MyServo().motion()