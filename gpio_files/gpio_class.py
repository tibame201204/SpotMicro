import RPi.GPIO as GPIO
import time
import cv2
import numpy as np

def main():
	# 控制伺服馬達sg90
	angle = 0  # 轉到180度的位置
	CONTROL_PIN = 17  # bcm17腳位控制
	PWM_FREQ = 50  # 頻率hz
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(CONTROL_PIN, GPIO.OUT)
	pwm = GPIO.PWM(CONTROL_PIN, PWM_FREQ)
	pwm.start(0)
	# 控制伺服馬達sg90到初始位置
	duty_cycle = (0.05 * PWM_FREQ) + (0.19 * PWM_FREQ * angle / 180)
	print(duty_cycle)
	pwm.ChangeDutyCycle(duty_cycle)  # 轉動到180度
	time.sleep(2)
	pwm.stop()
	GPIO.cleanup()

# class MyGPIO():
# 	def __init__(self):

if __name__ == '__main__':
	main()
