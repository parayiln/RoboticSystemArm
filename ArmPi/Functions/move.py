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
sys.path.append('/home/nidhi/RoboticSystemArm/ArmPi/')
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
        self.setTargetColor(['red', 'blue', 'green'])
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

    @staticmethod
    def setBuzzer(timer):
        Board.setBuzzer(0)
        Board.setBuzzer(1)
        time.sleep(timer)
        Board.setBuzzer(0)

    # Set the color of the RGB lights of the expansion board to match the color to be tracked
    @staticmethod
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


    # @log_on_error(logging.DEBUG, "Select either sorting or stacking")
    def set_task_parameters(self):
        if self.task == 'sorting':
            self.coordinate = {
                'red': (-15 + 0.5, 12 - 0.5, 1.5),
                'green': (-15 + 0.5, 6 - 0.5, 1.5),
                'blue': (-15 + 0.5, 0 - 0.5, 1.5),
            }
        elif self.task == 'stacking':
            self.coordinate = {
                'red': (-15 + 1, -7 - 0.5, 1.5),
                'green': (-15 + 1, -7 - 0.5, 1.5),
                'blue': (-15 + 1, -7 - 0.5, 1.5),
            }
            self.z_r = self.coordinate['red'][2]
            self.z_g = self.coordinate['green'][2]
            self.z_b = self.coordinate['blue'][2]
            self.z = self.z_r
            self.dz = 2.5
        else:
            raise IOError("Task not supported")


    def move(self):
        while True:
            if self.isRunning and not self.stop:
                if self.detect_color != 'None' and self.start_pick_up:
                    self.set_rgb(self.detect_color)
                    self.setBuzzer(0.1)
                    if self.task == 'stacking':
                        self.z = self.z_r
                        self.z_r += self.dz
                        if self.z == 2 * self.dz + self.coordinate['red'][2]:
                            self.z_r = self.coordinate['red'][2]
                        if self.z == self.coordinate['red'][2]:
                            self.move_square = True
                            time.sleep(3)
                            self.move_square = False
                    result = self.AK.setPitchRangeMoving((self.world_X, self.world_Y, 7), -90, -90, 0)
                    if result == False:
                        self.unreachable = True

                    else:
                        self.unreachable = False
                        time.sleep(result[2] / 1000)  # If the specified location can be reached, get the running time

                        if not self.isRunning:
                            continue
                        servo2_angle = getAngle(self.world_X, self.world_Y, self.rotation_angle)  # Calculate the angle by which the gripper needs to be rotated
                        Board.setBusServoPulse(1, self.servo1 - 280, 500)
                        Board.setBusServoPulse(2, servo2_angle, 500)
                        time.sleep(0.5)

                        if not self.isRunning:
                            continue
                        self.AK.setPitchRangeMoving((self.world_X, self.world_Y, 1.5), -90, -90, 0, 1000)
                        time.sleep(1.5)

                        if not self.isRunning:
                            continue
                        Board.setBusServoPulse(1, self.servo1, 500)  # Gripper closed
                        time.sleep(0.8)

                        if not self.isRunning:
                            continue
                        Board.setBusServoPulse(2, 500, 500)
                        self.AK.setPitchRangeMoving((self.world_X, self.world_Y, 12), -90, -90, 0, 1000)  # The robotic arm is raised
                        time.sleep(1)

                        if not self.isRunning:
                            continue
                        result = self.AK.setPitchRangeMoving((self.coordinate[self.detect_color][0], self.coordinate[self.detect_color][1], 12), -90, -90, 0)
                        time.sleep(result[2] / 1000)

                        if not self.isRunning:
                            continue
                        servo2_angle = getAngle(self.coordinate[self.detect_color][0], self.coordinate[self.detect_color][1], -90)
                        Board.setBusServoPulse(2, servo2_angle, 500)
                        time.sleep(0.5)

                        if self.task == 'sorting':
                            if not self.isRunning:
                                continue
                            self.AK.setPitchRangeMoving((self.coordinate[self.detect_color][0], self.coordinate[self.detect_color][1], self.coordinate[self.detect_color][2] + 3), -90, -90, 0, 500)
                        elif self.task == 'stacking':
                            if not self.isRunning:
                                continue
                            self.AK.setPitchRangeMoving((self.coordinate[self.detect_color][0], self.coordinate[self.detect_color][1], self.z + 3), -90, -90, 0, 500)
                        time.sleep(0.5)

                        if self.task == 'sorting':
                            if not self.isRunning:
                                continue
                            self.AK.setPitchRangeMoving((self.coordinate[self.detect_color]), -90, -90, 0, 1000)
                        elif self.task == 'stacking':
                            if not self.isRunning:
                                continue
                            self.AK.setPitchRangeMoving((self.coordinate[self.detect_color][0], self.coordinate[self.detect_color][1], self.z), -90, -90, 0, 500)
                        time.sleep(0.5)

                        if not self.isRunning:
                            continue
                        Board.setBusServoPulse(1, self.servo1 - 200, 500)  # Claws open to drop objects
                        time.sleep(0.8)

                        if not self.isRunning:
                            continue
                        self.AK.setPitchRangeMoving((self.coordinate[self.detect_color][0], self.coordinate[self.detect_color][1], 12), -90, -90, 0, 800)
                        time.sleep(0.8)

                        self.initMove()  # return to original position
                        time.sleep(1.5)

                        self.detect_color = 'None'
                        self.get_roi = False
                        self.start_pick_up = False
                        self.set_rgb(self.detect_color)
            else:
                time.sleep(0.01)

if __name__ == '__main__':

    motion = Motion()

    while True:
        key = cv2.waitKey(1)
        if key == 27:
            motion.stop_motion()
            break
