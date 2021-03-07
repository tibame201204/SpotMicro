import random
from time import sleep, time
from math import pi, sin, cos, atan, atan2, sqrt
import Spotmicro_Inverse_Kinematics_and_Position_Library_v01
Spot = Spotmicro_Inverse_Kinematics_and_Position_Library_v01.Spot()
import Spotmicro_Animation_Library_v01
SpotAnim = Spotmicro_Animation_Library_v01.SpotAnim()
import Spotmicro_Gravity_Center_Library_v01
SpotCG = Spotmicro_Gravity_Center_Library_v01.SpotCG()
from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)
import pygame
pygame.init()
screen = pygame.display.set_mode((600, 600))
pygame.display.set_caption("SPOTMICRO")

t = 0
steering =200
walking_direction = 0
walking_speed = 0

x_offset = 0
track = 58.09
b_height = 200
Angle = [0, 0]

x_spot = [0, x_offset, Spot.xlf, Spot.xrf, Spot.xrr, Spot.xlr,0,0,0]
y_spot = [0,0,Spot.ylf+track, Spot.yrf-track, Spot.yrr-track, Spot.ylr+track,0,0,0]
z_spot = [0,b_height,0,0,0,0,0,0,0]
theta_spot = [0,0,0,0,0,0]

pos_init = [-x_offset,track,-b_height,-x_offset,-track,-b_height,-x_offset,-track,-b_height,-x_offset,track,-b_height]
pos = [-x_offset,track,-b_height,-x_offset,-track,-b_height,-x_offset,-track,-b_height,-x_offset,track,-b_height,theta_spot,x_spot,y_spot,z_spot]

distance =[]
balance =[]
timing = []

lying = True
stop = False
tstep = 0.01

while True:
        # alpha_sitting = -30 / 180 * pi
        # alpha_pawing = 0 / 180 * pi
        # L_paw = 220
        #
        # x_end_sitting = Spot.xlr - Spot.L2 + Spot.L1 * cos(pi / 3) + Spot.Lb / 2 * cos(-alpha_sitting) - Spot.d * sin(
        #         -alpha_sitting)
        # z_end_sitting = Spot.L1 * sin(pi / 3) + Spot.Lb / 2 * sin(-alpha_sitting) + Spot.d * cos(-alpha_sitting)
        # start_frame_pos = [0, 0, 0, x_offset, 0, b_height]  # x,y,z rotations then translations
        #
        # # end_frame_pos = [0,0,0,x_offset,0,b_height-20] # x,y,z rotations then translations
        # end_frame_pos = [0, alpha_sitting, 0, x_end_sitting, 0, z_end_sitting]  # x,y,z rotations then translations
        # pos = Spot.moving(t, start_frame_pos, end_frame_pos, pos)

        # if (t == 1) & (pawing == False):
        #         pos_sit_init = pos
        #
        # if (t == 1):  # pawing is possible
        #         if (pawing == True):
        #                 # print (pos_sit_init[3],pos_sit_init[5])
        #                 pos[3] = pos_sit_init[3] + (L_paw * cos(alpha_pawing) - pos_sit_init[3]) * (joypar + 1) / 2
        #                 pos[5] = pos_sit_init[5] + (-Spot.d - L_paw * sin(alpha_pawing) - pos_sit_init[5]) * (
        #                 joypar + 1) / 2
        #
        #                 pos[0] = pos_sit_init[0] + (L_paw * cos(alpha_pawing) - pos_sit_init[0]) * (joypal + 1) / 2
        #                 pos[2] = pos_sit_init[2] + (-Spot.d - L_paw * sin(alpha_pawing) - pos_sit_init[2]) * (
        #                 joypal + 1) / 2
        #
        #                 thetarf = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[3], pos[4], pos[5], -1)[0]
        #                 thetalf = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[0], pos[1], pos[2], -1)[0]
        #                 # update of right front leg absolute position
        #                 legrf = Spot.FK(thetarf, -1)
        #                 leglf = Spot.FK(thetalf, -1)
        #                 xlegrf = Spot.xrf + pos[3]
        #                 ylegrf = Spot.yrf + pos[4]
        #                 zlegrf = pos[5]
        #                 xleglf = Spot.xlf + pos[0]
        #                 yleglf = Spot.ylf + pos[1]
        #                 zleglf = pos[2]
        #
        #                 theta_spot_sit = pos[12]
        #
        #                 x_spot_sit = pos[13]
        #                 y_spot_sit = pos[14]
        #                 z_spot_sit = pos[15]
        #
        #                 M = Spot.xyz_rotation_matrix(theta_spot_sit[3], theta_spot_sit[4],
        #                                              theta_spot_sit[2] + theta_spot_sit[5], False)
        #
        #                 paw_rf = Spot.new_coordinates(M, xlegrf, ylegrf, zlegrf, x_spot_sit[1], y_spot_sit[1],
        #                                               z_spot_sit[1])
        #                 paw_lf = Spot.new_coordinates(M, xleglf, yleglf, zleglf, x_spot_sit[1], y_spot_sit[1],
        #                                               z_spot_sit[1])
        #
        #                 x_spot_sit[3] = paw_rf[0]
        #                 y_spot_sit[3] = paw_rf[1]
        #                 z_spot_sit[3] = paw_rf[2]
        #                 x_spot_sit[2] = paw_lf[0]
        #                 y_spot_sit[2] = paw_lf[1]
        #                 z_spot_sit[2] = paw_lf[2]
        #
        #                 pos[13] = x_spot_sit
        #                 pos[14] = y_spot_sit
        #                 pos[15] = z_spot_sit

                # joypar_old = joypar
                # if (joypal == -1):
                #         if (((joypos[pos_rightpaw] != 0) & (joypos[pos_rightpaw] != -1)) | (joypar != -1)):
                #                 pawing = True
                #                 if (joypos[pos_rightpaw] >= joypar):
                #                         joypar = min(joypos[pos_rightpaw], joypar + 0.05)
                #                 else:
                #                         joypar = max(joypos[pos_rightpaw], joypar - 0.05)
                #         else:
                #                 pawing = False
                #
                # if (joypar_old == -1):
                #         if (((joypos[pos_leftpaw] != 0) & (joypos[pos_leftpaw] != -1)) | (joypal != -1)):
                #                 pawing = True
                #                 if (joypos[pos_leftpaw] >= joypal):
                #                         joypal = min(joypos[pos_leftpaw], joypal + 0.05)
                #                 else:
                #                         joypal = max(joypos[pos_leftpaw], joypal - 0.05)
                #         else:
                #                 pawing = False

        # if (stop == False):
        #         t = t + 4 * tstep
        #         if (t >= 1):
        #                 t = 1
        # elif (pawing == False):
        #         t = t - 4 * tstep
        #         if (t <= 0):
        #                 t = 0
        #                 stop = False
        #                 sitting = False
        #                 Free = True

        if (lying == True):
            angle_lying = 40/180*pi
            x_end_lying= Spot.xlr-Spot.L2 + Spot.L1*cos(angle_lying)+Spot.Lb/2
            z_end_lying = Spot.L1*sin(angle_lying)+Spot.d
            start_frame_pos = [0,0,0,x_offset,0,b_height] # x,y,z rotations then translations
            end_frame_pos = [0,0,0, x_end_lying,0,z_end_lying] # x,y,z rotations then translations
            pos = Spot.moving (t, start_frame_pos,end_frame_pos, pos)
            if (stop == False):
                t=t+3*tstep
                if (t>=1):
                    t= 1
            else:
                t=t-3*tstep
                if (t<= 0):
                    t= 0
                    stop  = False
                    lying = False
                    Free = True

        xc = steering * cos(walking_direction)
        yc = steering * sin(walking_direction)
        center_x = x_spot[0]+(xc*cos(theta_spot[2])-yc*sin(theta_spot[2])) #absolute center x position
        center_y = y_spot[0]+(xc*sin(theta_spot[2])+yc*cos(theta_spot[2])) #absolute center y position

        thetalf = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[0], pos[1], pos[2], 1)[0]
        thetarf = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[3], pos[4], pos[5], -1)[0]
        thetarr = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[6], pos[7], pos[8], -1)[0]
        thetalr = Spot.IK(Spot.L0, Spot.L1, Spot.L2, Spot.d, pos[9], pos[10], pos[11], 1)[0]


        # print(thetarf[0]/pi*180 +30)
        # print(thetarf[1]/pi*180 +30)

        print(round(thetalf[2] / pi * 180 * 4 / 3 * (1) + 187, 0))
        print(round(thetarf[2] / pi * 180 * 4 / 3 * (-1) - 6.5, 0))
        kit.servo[0].angle = round(thetalf[2] / pi * 180 * 4 / 3 * (1) + 187, 0)
        kit.servo[1].angle = round(thetarf[2] / pi * 180 * 4 / 3 * (-1) - 6.5, 0)

        stance = [False, False, False, False]
        if (pos[15][2] < 0.01):
            stance[0] = True
        if (pos[15][3] < 0.01):
            stance[1] = True
        if (pos[15][4] < 0.01):
            stance[2] = True
        if (pos[15][5] < 0.01):
            stance[3] = True

        print('===============================')
        print("左前腳: ", [round(i/pi*180, 2) for i in thetalf])
        print("右前腳: ", [round(i/pi*180, 2) for i in thetarf])
        print("左後腳: ", [round(i/pi*180, 2) for i in thetalr])
        print("右後腳: ", [round(i/pi*180, 2) for i in thetarr])
        print('===============================')
        SpotAnim.animate(pos,t,pi/12,-135/180*pi,Angle,center_x,center_y,thetalf,thetarf,thetarr,thetalr,walking_speed,walking_direction,steering,stance)

        pygame.display.flip()
        # sleep(0.1)

        """ CG update """
        CG = SpotCG.CG_calculation (thetalf,thetarf,thetarr,thetalr)
        #Calculation of CG absolute position
        M = Spot.xyz_rotation_matrix(theta_spot[0],theta_spot[1],theta_spot[2],False)
        CGabs = Spot.new_coordinates(M,CG[0],CG[1],CG[2],x_spot[1],y_spot[1],z_spot[1])
        dCG = SpotCG.CG_distance(x_spot[2:6],y_spot[2:6],z_spot[2:6],CGabs[0],CGabs[1],stance)


        pos[13][6] = CG[0] #x
        pos[14][6] = CG[1] #y
        pos[15][6] = CG[2] #z

        pos[13][7] = CGabs[0] #x
        pos[14][7] = CGabs[1] #y
        pos[15][7] = CGabs[2] #z

        pos[13][8] = dCG[1] #xint
        pos[14][8] = dCG[2] #yint
        pos[15][8] = dCG[3] #balance

        # distance.append(dCG[0])
        # timing.append(t)

pygame.quit()