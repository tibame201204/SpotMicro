import cv2  
import os
import platform
url = os.path.dirname(os.path.abspath(__file__))
os.chdir(url) 

class MyCamera():
	def __init__(self):
		self.cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
		self.waitframe = 33

	def cap_status(self):
		if not self.cap.isOpened():
			print("Cannot open camera...")
			return False
		else: 
			print("Camera Open...")
			return True

	def cap_read_status(self):
		ret, frame = self.cap.read()
		if not ret:
			print("Can't receive frame (stream end?). Exiting ...")
		return ret, frame

	def basic_test(self):
		if not self.cap_status():return
		if not self.cap_read_status()[0]:return
		while True:
			frame = self.cap_read_status()[1]

			cv2.imshow("basic_test", frame)
			# 按下 q 鍵離開迴圈
			if cv2.waitKey(33) == ord('q'):break

		# 釋放該攝影機裝置
		self.cap.release()
		cv2.destroyWindow("basic_test")

	def color_test(self):
		import numpy as np
		if not self.cap_status():return
		if not self.cap_read_status()[0]:return
		# 啟動Camera設定變數
		ret, frame = self.cap.read()
		rows, cols, _ = frame.shape
		x_medium = int(cols / 2)
		center = int(cols / 2)

		while True:
			frame = self.cap_read_status()[1]
			# color to gray
			hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
			# yellow color
			low_yellow = np.array([26, 43, 46])
			high_yellow = np.array([34, 255, 255])
			yellow_mask = cv2.inRange(hsv_frame, low_yellow, high_yellow)

			try: # opencv 3.2.0 有三個值
				ret, contrours, _ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
			except ValueError: # opencv 4.5.0 有二個值
				contrours, _ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
			contrours = sorted(contrours, key=lambda x: cv2.contourArea(x), reverse=True)

			for cnt in contrours:
				(x, y, w, h) = cv2.boundingRect(cnt)
				cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
				x_medium = int((x + x + w) / 2)
				y_medium = int((y + y + h) / 2)
				print("x_medium", x_medium, "y_medium", y_medium)
				# cv2.line(frame, (x_medium, 0), (x_medium, 480), (0, 0, 255), 2)
				break

			# 顯示圖片
			cv2.imshow('color_test', frame)
			# cv2.imshow("color_mask", yellow_mask)
			# 按下 q 鍵離開迴圈
			if cv2.waitKey(33) == ord('q'):break

		# 釋放該攝影機裝置
		self.cap.release()
		cv2.destroyWindow("color_test")

	def face_test(self):
		if not self.cap_status():return
		if not self.cap_read_status()[0]:return
		# 啟動Camera設定變數
		ret, frame = self.cap.read()
		rows, cols, _ = frame.shape
		x_medium = int(cols / 2)
		center = int(cols / 2)

		if platform.architecture()[1][0:3] == 'Win':
			print('OOK')
			faceCascade = cv2.CascadeClassifier(r"haarcascade_frontalface_default.xml")
		else:
			faceCascade = cv2.CascadeClassifier(url + "/" + "haarcascade_frontalface_default.xml")

		while True:
			frame = self.cap_read_status()[1]
			faces = faceCascade.detectMultiScale(frame, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30), flags=cv2.CASCADE_SCALE_IMAGE)
			cv2.rectangle(frame, (10, frame.shape[0] - 20), (110, frame.shape[0]), (0, 0, 0), -1)
			cv2.putText(frame, "Find " + str(len(faces)) + " face!", (10, frame.shape[0] - 5),
						cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
			for (x, y, w, h) in faces:
				cv2.rectangle(frame, (x, y), (x + w, y + h), (128, 255, 0), 2)

			# x_medium = int((frame.shape[1]) / 2)
			# cv2.line(frame, (x_medium, 0), (x_medium, 480), (0, 0, 255), 2)

			# 顯示圖片
			cv2.imshow("face_test", frame)
			# 按下 q 鍵離開迴圈
			if cv2.waitKey(33) == ord('q'):break

		# 釋放該攝影機裝置
		self.cap.release()
		cv2.destroyWindow("face_test")

if __name__ == "__main__":
	# MyCamera().basic_test()
	# MyCamera().color_test()
	MyCamera().face_test()