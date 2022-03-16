#!/usr/bin/python3
# coding=utf8
import logging
logging_format = "%(asctime)s: %(message)s"
import atexit
# from logdecorator import log_on_start, log_on_end, log_on_error
import cv2
import time
import math
import numpy as np
import threading
import sys
sys.path.append('/home/pi/RoboticSystemArm/ArmPi/')
import Camera
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *
sys.path.append('/home/nidhi/RoboticSystemArm/ArmPi/Function')
from perception import Perception

class Motion(Perception):
    def __init__(self, servo1=500, task='sorting', logging_level='INFO'):
        super().__init__()
        #self.setTargetColor(['red', 'blue', 'green'])
        print("Motion starting")
        if logging_level == 'INFO':
            logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="%H:%M:%S")
        elif logging_level == 'DEBUG':
            logging.basicConfig(format=logging_format, level=logging.DEBUG, datefmt="%H:%M:%S")
        self.AK = ArmIK()
        self.servo1 = servo1 # angle it which the gripper closes
        self.task = task #sorting or stacking
        self.unreachable = False
        self.stop = False
        self.game_status= False
        self.did_i_win = False
        self.size = (640, 480)
        self.curr_x =0
        self.curr_y=0
        self.Move()
        time.sleep(1.5)

    # actual code for playing with random strategy
    def Play(self):
        Board.setBusServoPulse(1, self.servo1 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        Board.setBusServoPulse(1,self.servo1 -200, 500)
        time.sleep(3)
        self.AK.setPitchRangeMoving((0, 14, 0), -90, -90, 0, 1000)
        time.sleep(5)
        Board.setBusServoPulse(1,self.servo1, 500)
        self.AK.setPitchRangeMoving((0, 14, 3), -90, -90, 0, 1000)
        time.sleep(5)
        while (self.game_status==False):
            next_move=randfom.randint(0, len(self.curr_cubes))
            self.curr_x , self.curr_y = convertCoordinate(self.curr_cubes[next_move][0], self.curr_cubes[next_move][0], self.size)
            self.AK.setPitchRangeMoving((0, self.curr_x *self.scale_x, self.curr_y *self.scale_y), -90, -90, 0, 1000)
            time.sleep(5)
            self.AK.setPitchRangeMoving((0, 14, 3), -90, -90, 0, 1000)
            time.sleep(10)
            self.game_over()
        Board.setBusServoPulse(1,self.servo1 -200, 500)



    def game_over(self):
        print(" Is the gave over? type 'n' for no or 'y' for yes")
        game_status =input()
        if game_over == 'n':
            self.game_status = False
        else:
            self.game_status= True
            winner=input()
            print("winner is")
            print(winner)





### hard code game strategy for video
    def Move(self):
        Board.setBusServoPulse(1, self.servo1 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        Board.setBusServoPulse(1,self.servo1 -200, 500)
        time.sleep(3)
        self.AK.setPitchRangeMoving((0, 14, 0), -90, -90, 0, 1000)
        time.sleep(5)
        Board.setBusServoPulse(1,self.servo1, 500)
        print('origin')
        self.AK.setPitchRangeMoving((0, 14, 3), -90, -90, 0, 1000)
        time.sleep(10)
        print('Along x')
        self.AK.setPitchRangeMoving((3.5,25.5, 3), -90, -90, 0, 1000)
        time.sleep(5)
        self.AK.setPitchRangeMoving((0, 14, 3), -90, -90, 0, 1000)
        time.sleep(10)
        print('Along y')
        self.AK.setPitchRangeMoving((-1, 17.5, 3), -90, -90, 0, 1000)
        time.sleep(5)
        self.AK.setPitchRangeMoving((0, 14, 3), -90, -90, 0, 1000)
        time.sleep(10)
        self.AK.setPitchRangeMoving((-1, 22.5, 3), -90, -90, 0, 1000)
        time.sleep(5)
        self.AK.setPitchRangeMoving((0, 14, 3), -90, -90, 0, 1000)
        time.sleep(3)
        Board.setBusServoPulse(1,self.servo1 -200, 500)
        time.sleep(10)











if __name__ == '__main__':

    motion = Motion()

    while True:
        key = cv2.waitKey(1)
        if key == 27:
            motion.stop_motion()
            break
