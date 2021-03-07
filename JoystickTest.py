import pygame
import numpy as np
import time
pygame.init()
screen = pygame.display.set_mode((600, 600)) 
pygame.display.set_caption("SPOTMICRO")

pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()
clock = pygame.time.Clock()

joypos =np.zeros(6)  #[0. 0. 0. 0. 0. 0.]
joybut = np.zeros(10) # [0. 0. 0. 0. 0. 0. 0. 0. 0. 0.]



while True:
	clock.tick(50)
	
	for event in pygame.event.get():  # User did something.
		if event.type == pygame.QUIT:  # If user clicked close window.  pygame.QUIT == 256
			continuer = False
		if event.type == pygame.MOUSEBUTTONDOWN:  # pygame.MOUSEBUTTONDOWN == 1025
			mouseclick = True
		else:
			mouseclick = False

	for i in range (0,6): #read analog joystick position
		joypos[i] = joystick.get_axis(i)
	for i in range (0,10):  #read buttons
		joybut[i] = joystick.get_button(i)
	joyhat = joystick.get_hat(0)  #read hat  (0, 0)

	if joybut[0] == 1:
		print('尿尿')
	elif joybut[1] == 1:
		print('扭屁股')
	elif joybut[2] == 1:
		print('坐下')
	elif joybut[3] == 1:
		print('趴下')
	elif joybut[4] == 1:
		print('動作')
	elif joybut[5] == 1:
		print('模擬')
	elif joybut[6] == 1:
		print('這是joybut6')
	elif joybut[7] == 1:
		print('走路')

	# print(joypos[1])
	# time.sleep(0.5)
	if round(joypos[0], 0) == 1:
		print('右轉')
	elif round(joypos[0], 0) == -1:
		print('左轉')
	elif round(joypos[1]) == 1:
		print('後滾')
	elif round(joypos[1]) == -1:
		print('前滾')
	elif round(joypos[2]) == 1:
		print('抬左手')
	# elif round(joypos[2]) == -1:
	# 	print('放左手')
	elif round(joypos[3]) == 1:
		print('往右走')
	elif round(joypos[3]) == -1:
		print('往左走')
	elif round(joypos[4]) == 1:
		print('往後走')
	elif round(joypos[4]) == -1:
		print('往前走')
	elif round(joypos[5]) == 1:
		print('抬右手')
	# elif round(joypos[5]) == -1:
	# 	print('放右手')