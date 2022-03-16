
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


class Perception(object):

    # @log_on_start(logging.DEBUG, "Constructor called ")
    # @log_on_error(logging.DEBUG, "Error in constructor call")
    # @log_on_end(logging.DEBUG, "Constructor finished")
    def __init__(self, logging_level='INFO'):
        print("Perception starting")
        if logging_level == 'INFO':
            logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="%H:%M:%S")
        elif logging_level == 'DEBUG':
            logging.basicConfig(format=logging_format, level=logging.DEBUG, datefmt="%H:%M:%S")
        self.camera = Camera.Camera()
        self.camera.camera_open()
        time.sleep(1) #Time delay for thread in camera class to start generating frames
        self.target_color = ['red']
        self.isRunning = False
        self.size = (640, 480)
        self.get_roi = False
        self.roi = ()
        self.start_pick_up = False
        self.center_list = []
        self.last_x, self.last_y = 0, 0
        self.world_X, self.world_Y = 0, 0
        self.world_x, self.world_y = 0, 0
        self.range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255),
        }
        self.rect = None
        self.count = 0
        self.rotation_angle = 0
        self.start_count_t1 = True
        self.t1 = 0
        self.detect_color = 'None'
        self.draw_color = self.range_rgb["black"]
        self.color_list = []
        self.move_square = False
        self.image = None
        self.th_p = threading.Thread(target=self.run_perception, args=(), daemon=True)
        self.th_p.start()
        self.cubelist=[(0,0), (0,0), (0,0), (0,0), (0,0), (0,0), (0,0), (0,0), (0,0)]
        self.curr_cubes=[]

    # @log_on_error(logging.DEBUG, "Can't save, Image empty")
    def sense(self):
        img = self.camera.frame
        if img is not None:
            self.image = img.copy()
            self.isRunning = True
        else:
            raise IOError("Camera frame empty")


    def stop_perception(self):
        self.isRunning = False
        print("Stopping perception")
        self.camera.camera_close()
        cv2.destroyAllWindows()

    # @log_on_error(logging.DEBUG, "Can't show, Image empty")
    def process(self):
        frame = self.image
        if frame is not None:
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            # Red color
            low_red = np.array([0, 120, 70])
            high_red = np.array([180, 255, 255])
            red_mask = cv2.inRange(hsv_frame, low_red, high_red)
            red = cv2.bitwise_and(frame, frame, mask=red_mask)
            imgray = cv2.cvtColor(red, cv2.COLOR_BGR2GRAY)
            ret, thresh = cv2.threshold(imgray, 0, 255, 0)
            contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            # cv2.drawContours(red, contours, -1, (0,255,0), 1)
            mu = [None]*len(contours)
            for i in range(len(contours)):
                mu[i] = cv2.moments(contours[i])
                mc = [None]*len(contours)
            for i in range(len(contours)):
                # add 1e-5 to avoid division by zero
                mc[i] = (mu[i]['m10'] / (mu[i]['m00'] + 1e-5), mu[i]['m01'] / (mu[i]['m00'] + 1e-5))
            # Draw contours
            drawing = np.zeros((thresh.shape[0], thresh.shape[1], 3), dtype=np.uint8)
            for i in range(len(contours)):
                color = (0,0,0)
                cv2.drawContours(red, contours, i, color, 2)
                cv2.circle(red, (int(mc[i][0]), int(mc[i][1])), 4, color, -1)
                for k in range(len(self.cubelist)):
                    if mc[i] == self.cubelist[k]:
                        self.curr_cubes.append(mc[i])
            # cv2.circle(red (cX, cY), 7, (255, 255, 255), -1)
            #cv2.imshow("Red",red)

            cv2.imshow(name, red)
        else:
            raise IOError("Input frame empty")


    def run_perception(self, name='frame'):
        while True:
            self.sense()
            if self.image is not None:
                frame = self.process()
                key = cv2.waitKey(1)
                if key == 27:
                    self.stop_perception()
                    break


if __name__ == '__main__':

    percept = Perception('DEBUG')
    percept.setTargetColor(['red','blue','green'])
    while True:
        key = cv2.waitKey(1)
        if key == 27:
            print("main code end")
            break
