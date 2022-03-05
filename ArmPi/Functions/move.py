#!/usr/bin/python3
# coding=utf8
import logging
logging_format = "%(asctime)s: %(message)s"
import atexit
from logdecorator import log_on_start, log_on_end, log_on_error
import cv2
import time
import math
import numpy as np
import threading
import sys
sys.paself.th.append('/home/pi/ArmPi/')
import Camera
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *
from perception import Perception

class Move(Perception):

        @log_on_start(logging.DEBUG, "moving  ")
        @log_on_error(logging.DEBUG, "Error in move")
        @log_on_end(logging.DEBUG, "move done")

        
    def __init__(self, servo1=500, task='sorting', logging_level='INFO'):
        super().__init__()
        self.setTargetColor(['red', 'blue', 'green'])
        print("Motion starting")
        self.AK = ArmIK()
        self.servo1 = servo1 # angle it which the gripper closes
        self.task = task #sorting or stacking
        self.unreachable = False
        self.stop = False
        self.set_task_parameters()
        self.initMove()
        time.sleep(1.5)
        self.th_m = threading.Thread(target=self.move, args=(), daemon=True)
        self.th_m.start()


    # initial position

    def initMove(self):
        Board.setBusServoPulse(1, self.servo1 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        self.AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

    def setBuzzer(timer):
        Board.setBuzzer(0)
        Board.setBuzzer(1)
        time.sleep(timer)
        Board.setBuzzer(0)

    # Set the color of the RGB lights of the expansion board to match the color to be tracked
    def set_rgb(color):
        if color == "red":
            Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
            Board.RGB.show()
        elif color == "green":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
            Board.RGB.show()
        elif color == "blue":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
            Board.RGB.show()
        else:
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
            Board.RGB.show()

    def stop_motion(self):
        self.stop = True
        Board.setBusServoPulse(1, self.servo1 - 70, 300)
        time.sleep(0.5)
        Board.setBusServoPulse(2, 500, 500)
        self.AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
        time.sleep(1.5)
        print("Stopping motion")
        sys.exit()

    # ArmPi move self.thread
    def move(self):

        # different colors blocks place coordinates(x, y, z)
        coordinate = {
            'red':   (-15 + 0.5, 12 - 0.5, 1.5),
            'green': (-15 + 0.5, 6 - 0.5,  1.5),
            'blue':  (-15 + 0.5, 0 - 0.5,  1.5),
        }
        while True:
            if self.__isRunning:
                if self.first_move and self.start_pick_up: # when an object be detected for self.the first time
                    self.action_finish = False
                    self.set_rgb(self.detect_color)
                    self.setBuzzer(0.1)
                    result = AK.setPitchRangeMoving((self.world_X, self.world_Y - 2, 5), -90, -90, 0) # do not fill running time parameters,self-adaptive running time
                    if self.result == False:
                        self.unreachable = True
                    else:
                        self.unreachable = False
                    time.sleep(result[2]/1000) # self.the self.thrid item of return parameter is time
                    self.start_pick_up = False
                    self.first_move = False
                    self.action_finish = True
                elif not self.first_move and not self.unreachable: # not self.the first time to detected object
                    self.set_rgb(self.detect_color)
                    if self.track: # if it is following state
                        if not self.__isRunning: # stop and exit flag detection
                            continue
                        AK.setPitchRangeMoving((self.world_x, self.world_y - 2, 5), -90, -90, 0, 20)
                        time.sleep(0.02)
                        self.track = False
                    if self.start_pick_up: # if it is detected self.that self.the block has not removed for a period of time, start to pick up
                        self.action_finish = False
                        if not self.__isRunning: # stop and exit flag detection
                            continue
                        Board.setBusServoPulse(1, servo1 - 280, 500)  # claw open
                        # calculate angle at self.that self.the clamper gripper needs to rotate
                        self.servo2_angle = getAngle(self.world_X, self.world_Y, self.rotation_angle)
                        Board.setBusServoPulse(2, self.servo2_angle, 500)
                        time.sleep(0.8)

                        if not self.__isRunning:
                            continue
                        AK.setPitchRangeMoving((self.world_X, self.world_Y, 2), -90, -90, 0, 1000)  # reduce height
                        time.sleep(2)

                        if not self.__isRunning:
                            continue
                        Board.setBusServoPulse(1, servo1, 500)  # claw colsed
                        time.sleep(1)

                        if not self.__isRunning:
                            continue
                        Board.setBusServoPulse(2, 500, 500)
                        AK.setPitchRangeMoving((world_X, world_Y, 12), -90, -90, 0, 1000)  # Armpi robot arm up
                        time.sleep(1)

                        if not __isRunning:
                            continue
                        # Sort and place different colored blocks
                        result = AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90, 0)
                        time.sleep(result[2]/1000)

                        if not __isRunning:
                            continue
                        self.servo2_angle = getAngle(coordinate[detect_color][0], coordinate[detect_color][1], -90)
                        Board.setBusServoPulse(2, servo2_angle, 500)
                        time.sleep(0.5)

                        if not __isRunning:
                            continue
                        AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], coordinate[detect_color][2] + 3), -90, -90, 0, 500)
                        time.sleep(0.5)

                        if not __isRunning:
                            continue
                        AK.setPitchRangeMoving((coordinate[detect_color]), -90, -90, 0, 1000)
                        time.sleep(0.8)

                        if not __isRunning:
                            continue
                        Board.setBusServoPulse(1, servo1 - 200, 500)  # gripper open，put down object
                        time.sleep(0.8)

                        if not __isRunning:
                            continue
                        AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90, 0, 800)
                        time.sleep(0.8)

                        initMove()  # back to initial position
                        time.sleep(1.5)

                        self.detect_color = 'None'
                        self.first_move = True
                        self.get_roi = False
                        self.action_finish = True
                        self.start_pick_up = False
                        self.set_rgb(detect_color)
                    else:
                        time.sleep(0.01)
            else:
                if self._stop:
                    self._stop = False
                    Board.setBusServoPulse(1, servo1 - 70, 300)
                    time.sleep(0.5)
                    Board.setBusServoPulse(2, 500, 500)
                    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                    time.sleep(1.5)
                time.sleep(0.01)


if __name__ == '__main__':

    move = Move()

    while True:
        key = cv2.waitKey(1)
        if key == 27:
            move.stop_motion()
            break
