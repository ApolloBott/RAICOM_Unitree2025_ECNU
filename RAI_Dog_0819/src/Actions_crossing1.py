#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from Timer import timer
from Img_Processor import Img_Processor
from PID import PID
import time
import sys
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.go2.sport.sport_client import (
    SportClient,
    PathPoint,
    SPORT_PATH_POINT_SIZE,
)
import math
import os
from distance import read_distance
from distance import read_distance1
import serial
from ArmControl import ArmControl

pid_go_r_vy = PID(0.003, 0, 0, 256)
pid_go_l_vy = PID(0.002, 0 ,0, 180)
pid_go_l_vy_s = PID(0.003, 0 ,0, 160)
pid_go_l_vy_s1 = PID(0.003, 0 ,0, 180)
pid_go_r_vy_s = PID(0.003, 0 ,0, 256)
pid_go_ys = PID(0.005, 0, 0, 90)
pid_go_ys_s = PID(0.02, 0, 0, 90)

pid_ent_cro_vy = PID(0.002, 0, 0, 350)
pid_ent_cro_ys = PID(0.03, 0, 0, -90)

pid_sml_cro_vy = PID(0.0012, 0, 0, 90)
pid_sml_cro_ys = PID(0.03, 0, 0, 90)

pid_sml_cro_vy_l = PID(0.0012, 0, 0, 130)
pid_sml_cro_vy_l_1 = PID(0.0012, 0, 0, 100)
pid_sml_cro_vy_r = PID(0.0012, 0, 0, 210)
#pid_sml_cro_ys_r = PID(0.02, 0, 0, 90)

pid_sml_cro_vy_s = PID(0.0012, 0, 0, 140)
pid_sml_cro_ys_s = PID(0.0025, 0, 0, 90)

pid_sml_cro_vy1 = PID(0.0008, 0, 0, 140)
pid_sml_cro_ys1 = PID(0.015, 0, 0, 90)




pid_start_vy = PID(0.002, 0 ,0, 160)
pid_start_ys = PID(0.005, 0, 0, 90)
pid_start_dis_in = PID(0.0007,0,0, 320)
pid_start_dis_out = PID(0.0007,0,0, 280)


pid_cir_vy = PID(0.0012, 0, 0, 110)
pid_cir_ys = PID(0.02, 0, 0, 90)
pid_cir_go_vy = PID(0.003, 0 ,0, 160)
pid_cir_go_ys = PID(0.005, 0, 0, 90)


pid_cro1_vy = PID(0.0012, 0, 0, 110)
pid_cro1_ys = PID(0.017, 0, 0, 90)


pid_cro2_go_vy = PID(0.002, 0 ,0, 160)
pid_cro2_go_ys = PID(0.006, 0, 0, 95)
pid_cro2_vy = PID(0.0012, 0, 0, 110)
pid_cro2_ys = PID(0.02, 0, 0, 90)


pid_up_go_vy = PID(0.002, 0 ,0, 185)
pid_up_go_ys = PID(0.01, 0, 0, 90)


pid_fin_vy = PID(0.0012, 0, 0, 160)
pid_fin_ys = PID(0.008, 0, 0, 90)


class Actions:
    def __init__(self):

        self.micro_s = 0
        self.w_v_x_ = 0
        self.w_v_y_ = 0
        self.w_yaw_speed_ = 0
        self.w_arm_mode_ = 0
        self.w_move_mode_ = 0
        self.w_light_mode = 0
        self.w_gait_type_ = 0
        self.w_body_height_ = 0
        self.w_finish = 0
        self.sm = "s_start"
        

        self.right = 0
        self.front = 0

    def f_start(self,img_processor: Img_Processor,sport_client: SportClient, armcontrol: ArmControl, ser3):
    #def f_start(self,img_processor: Img_Processor,sport_client: SportClient):
        #print(f"s:{self.micro_s}")
        #ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        #ser1 = serial.Serial('/dev/ttyUSB2', 115200, timeout=1)
        
        right_even_agl = abs(img_processor.right_even_agl)
        if self.micro_s == 0:
            img_processor.finish_visual = False
            timer.start_timer()
            self.micro_s = 1
        elif self.micro_s == 1:
            timer.get_elapsed(8) 
            if timer.time_is_up == True:
                self.micro_s = 2
                timer.start_timer()
        elif self.micro_s == 2:
            timer.get_elapsed(2.2)
            sport_client.SwitchGait(3)
            sport_client.Move(0,0.35,0)
            if timer.time_is_up == True:
                self.micro_s = 3
                timer.start_timer()
        elif self.micro_s == 3:
            timer.get_elapsed(2.8)
            sport_client.Move(0.5,0,0)
            if timer.time_is_up == True:
                self.micro_s = 4
                timer.start_timer()
        elif self.micro_s == 4:
            timer.get_elapsed(1.8)
            sport_client.Move(0,-0.35,0)  
            if timer.time_is_up == True:
                self.micro_s = 5  
                timer.start_timer() 
            #sport_client.SwitchGait(3)
            #self.w_v_x = 0.35
            #self.w_v_y_ = pid_go_l_vy.compute(img_processor.left_even_x)
            #self.w_yaw_speed_ = pid_go_ys_s.compute(right_even_agl)
            #self.w_v_y_ = 0
            #self.w_yaw_speed_ = 0
        elif self.micro_s == 5:
            timer.get_elapsed(1.3)
            sport_client.Move(0.35,0,0)
            if timer.time_is_up == True:
                self.micro_s = 6
        elif self.micro_s == 6:       
            self.w_v_x_ = 0.35
            self.w_v_y_ = pid_start_vy.compute(img_processor.left_even_x)
            if img_processor.left_even_agl > 0:
                self.w_yaw_speed_ = -pid_start_ys.compute(right_even_agl)
            else:
                self.w_yaw_speed_ = pid_start_ys.compute(right_even_agl)
            sport_client.Move(self.w_v_x_,self.w_v_y_,self.w_yaw_speed_)
            if img_processor.get_l_contour == False and img_processor.left_even_x != 0:
                self.micro_s = 7
                timer.start_timer()
        elif self.micro_s == 7:
            timer.get_elapsed(1.5)
            sport_client.Move(0.35,0,0)
            if timer.time_is_up == True:
                self.micro_s = 8
                #img_processor.finish_visual = True
                timer.start_timer()
        elif self.micro_s == 8:
            timer.get_elapsed(1.6)
            sport_client.Move(0,-0.2,0)
            if timer.time_is_up == True:
                self.micro_s = 9
                timer.start_timer()
        elif self.micro_s == 9:
            timer.get_elapsed(4)
            self.right = read_distance1(ser3)
            self.w_v_y_ = pid_start_dis_in.compute(self.right)
            #print(f"right:{self.right}")
            sport_client.Move(0.35,self.w_v_y_,0)
            if timer.time_is_up:
                self.micro_s = 10
                timer.start_timer() 
        elif self.micro_s == 10:
            timer.get_elapsed(1.8)
            #self.front = read_distance1(ser4)
            #print(f"front:{self.front}")
            sport_client.Move(0.2,0,0)
            #if self.front < 150:
            if timer.time_is_up:
                self.micro_s = 11
                timer.start_timer() 
        elif self.micro_s == 11:
            timer.get_elapsed(1.6)
            sport_client.Move(0.2,0,1)
            if timer.time_is_up:
                self.micro_s = 12
                timer.start_timer() 
        elif self.micro_s == 12:
            timer.get_elapsed(3)
            self.right = read_distance1(ser3)
            self.w_v_y_ = pid_start_dis_out.compute(self.right)
            sport_client.Move(0.5,self.w_v_y_,0)
            if timer.time_is_up:
                self.micro_s = 13
        elif self.micro_s == 13:
            timer.get_elapsed(1)
            sport_client.Move(0.5,0,0)
            if timer.time_is_up:
                self.micro_s = 14
                timer.start_timer() 
        elif self.micro_s == 14:
            timer.get_elapsed(1)
            sport_client.Move(0,0.2,0)
            if timer.time_is_up:
                self.micro_s = 15
                timer.start_timer() 
        elif self.micro_s == 15:
            if img_processor.get_l_contour == True and img_processor.left_even_x > 80:
                self.micro_s = 16
                timer.start_timer() 
        elif self.micro_s == 16:
            timer.get_elapsed(2)
            img_processor.recog_tag()
            self.w_v_x_ = 0.35
            self.w_v_y_ = pid_start_vy.compute(img_processor.left_even_x)
            if img_processor.left_even_agl > 0:
                self.w_yaw_speed_ = -pid_start_ys.compute(right_even_agl)
            else:
                self.w_yaw_speed_ = pid_start_ys.compute(right_even_agl)
            sport_client.Move(self.w_v_x_,self.w_v_y_,self.w_yaw_speed_)
            if timer.time_is_up:
                self.micro_s = 17
        elif self.micro_s == 17:
            img_processor.recog_tag()
            sport_client.Move(0.35,0,0)
            if img_processor.ids == 4:
                self.sm = "s_in_circle"
                self.micro_s = 0
       

    def f_in_circle(self,img_processor: Img_Processor,sport_client: SportClient,armcontrol: ArmControl,ser0,ser1,ser2):
    #def f_in_circle(self,img_processor: Img_Processor,sport_client: SportClient):
        #ser0 = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        #ser1 = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
       # ser0 = serial.Serial('/dev/ttyCH343USB0', 921600, timeout=1)
        img_processor.recog_tag()
        img_processor.finish_visual = False
        left_even_agl = abs(img_processor.left_even_agl)
        right_even_agl = abs(img_processor.right_even_agl)
        #print(f"s:{self.micro_s}")
        #print(f"w:{img_processor.width_map}")
        #print(f"r:{img_processor.right_even_agl}")
        if self.micro_s == 0:
            timer.start_timer()
            self.micro_s = 1
        elif self.micro_s == 1:
            timer.get_elapsed(2)
            sport_client.Move(0.35,0,0)
            if timer.time_is_up:
                self.micro_s = 2
                timer.start_timer()
        elif self.micro_s == 2:
            timer.get_elapsed(8)
            img_processor.ids = 0
            self.w_v_x_ = 0.35
            self.w_v_y_ = pid_cir_vy.compute(img_processor.left_even_x)
            if img_processor.left_even_agl > 0:
                self.w_yaw_speed_ = -pid_cir_ys.compute(left_even_agl)
            else:
                self.w_yaw_speed_ = pid_cir_ys.compute(left_even_agl)
            sport_client.Move(self.w_v_x_,self.w_v_y_,self.w_yaw_speed_)
            if timer.time_is_up:
                self.micro_s = 3
                timer.start_timer()
        elif self.micro_s == 3:
            timer.get_elapsed(3)
            sport_client.Move(0,0.15,-0.15)
            if timer.time_is_up:
                self.micro_s = 4
                #timer.start_timer()
        elif self.micro_s == 4:
            #timer.get_elapsed(3)
            sport_client.Move(0,0,0)
            #if timer.time_is_up:
            armcontrol.arm_catch_and_put(ser0,ser1,ser2)
            if armcontrol.fin_circle == True:
                self.micro_s = 5
                #armcontrol.reset_arm(125, 450, 100, 450, 530, 1000)
                timer.start_timer()
        elif self.micro_s == 5:
            timer.get_elapsed(1.6)
            sport_client.Move(0,-0.2,0)
            if timer.time_is_up:
                self.micro_s = 6
                timer.start_timer()
        elif self.micro_s == 6:
            timer.get_elapsed(4.5)
            self.w_v_x_ = 0.35
            self.w_v_y_ = pid_cir_vy.compute(img_processor.left_even_x)
            if img_processor.left_even_agl > 0:
                self.w_yaw_speed_ = -pid_cir_ys.compute(left_even_agl)
            else:
                self.w_yaw_speed_ = pid_cir_ys.compute(left_even_agl)
            sport_client.Move(self.w_v_x_,self.w_v_y_,self.w_yaw_speed_)
            if timer.time_is_up:
                self.micro_s = 7
                timer.start_timer()
        elif self.micro_s == 7:
            timer.get_elapsed(0.4)
            sport_client.Move(0,-0.35,0)
            if timer.time_is_up:
                self.micro_s = 8
                timer.start_timer()
        elif self.micro_s == 8:
            timer.get_elapsed(3.2)
            sport_client.Move(0,0,-1)
            if timer.time_is_up:
                self.micro_s = 9
                timer.start_timer()
        elif self.micro_s == 9:
            timer.get_elapsed(3.3)
            #self.w_v_x_ = 0.35
            self.w_v_x_ = 0.5
            self.w_v_y_ = pid_cir_go_vy.compute(img_processor.left_even_x)
            if img_processor.left_even_agl > 0:
                self.w_yaw_speed_ = -pid_cir_go_ys.compute(left_even_agl)
            else:
                self.w_yaw_speed_ = pid_cir_go_ys.compute(left_even_agl)
            sport_client.Move(self.w_v_x_,self.w_v_y_,self.w_yaw_speed_)
            if timer.time_is_up:
                self.micro_s = 10
                timer.start_timer()
        elif self.micro_s == 10:
            timer.get_elapsed(1.8)
            sport_client.Move(0.35, 0, 0)
            if timer.time_is_up:
                self.sm = "s_in_crossing1"
                self.micro_s = 0
        # elif self.micro_s == 11:
        #     timer.get_elapsed(1)
        #     sport_client.Move(0, 0, 0)
        #     if img_processor.ids == 3:
        #         self.sm = "s_in_crossing1"
        #         self.micro_s = 0
        #     elif timer.time_is_up:
        #         self.sm = "s_ign_crossing"
        #         self.micro_s = 0
        # modified by Lugenbot
        #elif self.micro_s == 11:
        #    timer.get_elapsed(1)
        #    sport_client.Move(0, 0, 0)
        #    if timer.time_is_up:
        #        self.sm = "s_in_crossing1"
        #        self.micro_s = 0


             
                
              
    def f_in_crossing1(self,img_processor: Img_Processor,sport_client: SportClient,armcontrol: ArmControl,ser1,ser2):
    #def f_in_crossing1(self,img_processor: Img_Processor,sport_client: SportClient):
        #ser1 = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        left_even_agl = abs(img_processor.left_even_agl)
        if self.micro_s == 0:
            timer.start_timer()
            self.micro_s = 1
        elif self.micro_s == 1:
            timer.get_elapsed(0.8)
            sport_client.Move(0.35,0,0)
            if timer.time_is_up:
                self.micro_s = 2
                timer.start_timer()
        elif self.micro_s == 2:
            timer.get_elapsed(7.8)
            img_processor.ids = 0
            self.w_v_y_ = pid_cro1_vy.compute(img_processor.left_even_x)
            if img_processor.left_even_agl > 0:
                self.w_yaw_speed_ = -pid_cro1_ys.compute(left_even_agl)
            else:
                self.w_yaw_speed_ = pid_cro1_ys.compute(left_even_agl)
            sport_client.Move(0.35,self.w_v_y_,self.w_yaw_speed_)
            if timer.time_is_up:
                self.micro_s = 3
                timer.start_timer()
        elif self.micro_s == 3:
            timer.get_elapsed(1.8)
            sport_client.Move(0,0.2,0)
            if timer.time_is_up:
                self.micro_s = 4
                #timer.start_timer()
        elif self.micro_s == 4:
            #timer.get_elapsed(3)
            sport_client.Move(0,0,0)
            #if timer.time_is_up:
            armcontrol.arm_put_fixed(ser2)
            if armcontrol.fin_crossing == True:
                self.micro_s = 5
                timer.start_timer()
        elif self.micro_s == 5:
            timer.get_elapsed(1.5)
            sport_client.Move(0,-0.2,0)
            if timer.time_is_up:
                self.micro_s = 6
                timer.start_timer()
        elif self.micro_s == 6:
            timer.get_elapsed(7.4)
            self.w_v_x_ = 0.35
            self.w_v_y_ = pid_cro1_vy.compute(img_processor.left_even_x)
            if img_processor.left_even_agl > 0:
                self.w_yaw_speed_ = -pid_cro1_ys.compute(left_even_agl)
            else:
                self.w_yaw_speed_ = pid_cro1_ys.compute(left_even_agl)
            sport_client.Move(self.w_v_x_,self.w_v_y_,self.w_yaw_speed_)
            if timer.time_is_up:
                self.micro_s = 7
                timer.start_timer()
        elif self.micro_s == 7:
            timer.get_elapsed(2.2)
            sport_client.Move(0.35,0,0)
            if timer.time_is_up:
                self.sm = "s_in_crossing2"
                img_processor.ids = 0
                self.micro_s = 0

    def f_in_crossing2(self,img_processor: Img_Processor,sport_client: SportClient,armcontrol: ArmControl,ser1,ser2):
    #def f_in_crossing2(self,img_processor: Img_Processor,sport_client: SportClient):
        #ser1 = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        img_processor.recog_tag()
        left_even_agl = abs(img_processor.left_even_agl)
        if self.micro_s == 0:
            timer.start_timer()
            self.micro_s = 1
        elif self.micro_s == 1:
            timer.get_elapsed(4.4)
            #self.w_v_x_ = 0.35
            self.w_v_x_ = 0.5
            self.w_v_y_ = pid_cro2_go_vy.compute(img_processor.left_even_x)
            if img_processor.left_even_agl > 0:
                self.w_yaw_speed_ = -pid_cro2_go_ys.compute(left_even_agl)
            else:
                self.w_yaw_speed_ = pid_cro2_go_ys.compute(left_even_agl)
            sport_client.Move(self.w_v_x_,self.w_v_y_,self.w_yaw_speed_)
            if timer.time_is_up:
                self.micro_s = 2
                timer.start_timer()
        elif self.micro_s == 2:
            timer.get_elapsed(1.5)
            sport_client.Move(0.35,0,0)
            if timer.time_is_up:
                self.sm = 's_ign_crossing'
                self.micro_s = 0
                
                
                #self.micro_s = 3
                #timer.start_timer()
       # elif self.micro_s == 3:
            #timer.get_elapsed(1)
            #sport_client.Move(0,0,0)
            # if img_processor.ids == 3:
            #     self.micro_s = 4
            #     timer.start_timer()
            # elif timer.time_is_up:
            #     self.sm = 's_ign_crossing'
            #     self.micro_s = 0 
            
       
       
        ''' 
        elif self.micro_s == 4:
            timer.get_elapsed(0.5)
            sport_client.Move(0.35,0,0)
            if timer.time_is_up:
                self.micro_s = 5
                timer.start_timer()
        elif self.micro_s == 5:
            timer.get_elapsed(8.5)
            self.w_v_x_ = 0.35
            self.w_v_y_ = pid_cro2_vy.compute(img_processor.left_even_x)
            if img_processor.left_even_agl > 0:
                self.w_yaw_speed_ = -pid_cro2_ys.compute(left_even_agl)
            else:
                self.w_yaw_speed_ = pid_cro2_ys.compute(left_even_agl)
            sport_client.Move(self.w_v_x_,self.w_v_y_,self.w_yaw_speed_)
            if timer.time_is_up:
                self.micro_s = 6
                timer.start_timer()
        elif self.micro_s == 6:
            timer.get_elapsed(1.8)
            sport_client.Move(0,0.2,0)
            if timer.time_is_up:
                self.micro_s = 7
                timer.start_timer()
        elif self.micro_s == 7:
            timer.get_elapsed(3)
            sport_client.Move(0,0,0)
            armcontrol.arm_put_fixed(ser2)
            #if timer.time_is_up:
            if armcontrol.fin_crossing == True:
                self.micro_s = 8
                timer.start_timer()
        elif self.micro_s == 8:
            timer.get_elapsed(1.2)
            sport_client.Move(0,-0.15,0)
            if timer.time_is_up:
                self.micro_s = 9
                timer.start_timer()
        elif self.micro_s == 9:
            timer.get_elapsed(7)
            self.w_v_x_ = 0.35
            self.w_v_y_ = pid_cro2_vy.compute(img_processor.left_even_x)
            if img_processor.left_even_agl > 0:
                self.w_yaw_speed_ = -pid_cro2_ys.compute(left_even_agl)
            else:
                self.w_yaw_speed_ = pid_cro2_ys.compute(left_even_agl)
            sport_client.Move(self.w_v_x_,self.w_v_y_,self.w_yaw_speed_)
            if timer.time_is_up:
                self.micro_s = 10
                timer.start_timer()
        elif self.micro_s == 10:
            timer.get_elapsed(3)
            sport_client.Move(0.35,0,0)
            if timer.time_is_up:
                self.sm = "s_up"
                self.micro_s = 0
        '''
        
    def f_ignore_crossing(self,img_processor: Img_Processor,sport_client: SportClient):
        left_even_agl = abs(img_processor.left_even_agl)
        if self.micro_s == 0:
            timer.start_timer()
            self.micro_s = 1
        elif self.micro_s == 1:
            timer.get_elapsed(0.8)
            sport_client.Move(0.1,0,0)
            if timer.time_is_up:
                self.micro_s = 2
                timer.start_timer()
        elif self.micro_s == 2:
            timer.get_elapsed(1.6)
            sport_client.Move(0,0,-1)
            if timer.time_is_up:
                self.micro_s = 3
                timer.start_timer()
        elif self.micro_s == 3:
            timer.get_elapsed(1)
            sport_client.Move(0.35,0,0)
            if timer.time_is_up:
                self.micro_s = 4
                timer.start_timer()
        elif self.micro_s == 4:
            self.sm = "s_up"
            self.micro_s = 0

    def f_up(self,img_processor: Img_Processor,sport_client: SportClient,pitch):
    #def f_up(self,sport_client: SportClient,pitch):
        #img_processor.finish_visual = True
        #img_processor.recog_stair()
        #print(f"f:{img_processor.gray_top_pnt}")
        left_even_agl = abs(img_processor.left_even_agl)
        print(f"pitch:{pitch}")
        if self.micro_s == 0:
            timer.start_timer()
            self.micro_s = 1
        elif self.micro_s == 1:
            timer.get_elapsed(6)
            self.w_v_x_ = 0.35
            #self.w_v_x_ = 0.5
            self.w_v_y_ = pid_up_go_vy.compute(img_processor.left_even_x)
            if img_processor.left_even_agl > 0:
                self.w_yaw_speed_ = -pid_up_go_ys.compute(left_even_agl)
            else:
                self.w_yaw_speed_ = pid_up_go_ys.compute(left_even_agl)
            sport_client.Move(self.w_v_x_,self.w_v_y_,self.w_yaw_speed_)
            if timer.time_is_up:
                self.micro_s = 2
        elif self.micro_s == 2:
            sport_client.SwitchGait(1)
            sport_client.Move(0.37,0,0)
            if abs(pitch) > 0.38:
                self.micro_s = 3      
        elif self.micro_s == 3:
            sport_client.Move(0.37,0,0)
            if abs(pitch) < 0.1:
                self.micro_s = 4
                timer.start_timer()
        elif self.micro_s == 4:
            timer.get_elapsed(2.2)
            sport_client.Move(0,0.1,0.8)
            if timer.time_is_up:
                self.micro_s = 5
                timer.start_timer()
        elif self.micro_s == 5:
            timer.get_elapsed(2)
            sport_client.Move(0.3,0,0)
            if timer.time_is_up:
                self.micro_s = 6
        elif self.micro_s == 6:
            sport_client.Move(0.3,0,0)
            if abs(pitch) < 0.07:
                self.micro_s = 7
                timer.start_timer()
        elif self.micro_s == 7:
            timer.get_elapsed(0.1)
            sport_client.Move(0,0,0)
            if timer.time_is_up:
                self.micro_s = 8
                timer.start_timer()
        elif self.micro_s == 8:
            timer.get_elapsed(2)
            sport_client.Move(0,0,0.5)
            if timer.time_is_up:
                self.sm = 's_finish'
                self.micro_s = 0
            '''
            if img_processor.get_r_contour == False:
                timer.get_elapsed(2.5)
                sport_client.Move(0,0.3,0)
                if img_processor.get_r_contour == True and img_processor.right_even_agl < 260:
                    self.sm = 's_finish'
                    self.micro_s = 0
                if timer.time_is_up:
                    sport_client.Move(0,-0.3,0)
                    if img_processor.get_r_contour == True and img_processor.right_even_agl < 260:
                        self.sm = 's_finish'
                        self.micro_s = 0
            else:
                self.sm = 's_finish'
                self.micro_s = 0
            '''
                          
    def f_finish(self,img_processor: Img_Processor,sport_client: SportClient):
        #img_processor.recog_s_e_r()
      #  img_processor.finish_visual == False
        left_even_agl = abs(img_processor.right_even_agl)
        if self.micro_s == 0:
            timer.start_timer()
            self.micro_s = 1
        elif self.micro_s == 1:
            timer.get_elapsed(5)
            self.w_v_x_ = 0.35
            self.w_v_y_ = pid_fin_vy.compute(img_processor.left_even_x)
            if img_processor.right_even_agl > 0:
                self.w_yaw_speed_ = -pid_fin_ys.compute(left_even_agl)
            else:
                self.w_yaw_speed_ = pid_fin_ys.compute(left_even_agl)
            sport_client.Move(self.w_v_x_,self.w_v_y_,self.w_yaw_speed_)
            if timer.time_is_up:
                self.micro_s = 2
                timer.start_timer()
        elif self.micro_s == 2:
            timer.get_elapsed(1)
            sport_client.Move(0.35,0,0)
            if timer.time_is_up:
                self.micro_s = 3 
                timer.start_timer()
        elif self.micro_s == 3:
            #timer.get_elapsed(4)
            #sport_client.Move(0,0.2,0)
            timer.get_elapsed(2.5)
            sport_client.Move(0,0.35,0)
            if timer.time_is_up:
                self.micro_s = 4
                timer.start_timer()
        elif self.micro_s == 4:
            timer.get_elapsed(3.2)
            sport_client.Move(0.5,0,0)
            if timer.time_is_up:
                self.micro_s = 5
                timer.start_timer()
        elif self.micro_s == 5:
            #timer.get_elapsed(4.2)
            #sport_client.Move(0,-0.2,0)
            timer.get_elapsed(2.5)
            sport_client.Move(0,-0.35,0)
            if timer.time_is_up:
                self.micro_s = 6
                timer.start_timer()
        elif self.micro_s == 6:
            sport_client.Move(0,0,0)
        
        
        
                
            
          
        

