
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
    def show(self, name='frame', frame=None):
        if frame is not None:
            cv2.imshow(name, frame)
        else:
            raise IOError("Input frame empty")


    def process(self):
        img = self.image
        img_copy = self.image.copy()
        img_h, img_w = img.shape[:2]
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)

        if not self.isRunning:
            return img

        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        # If an area is detected with a recognized object, the area is detected until there are none
        if self.get_roi and not self.start_pick_up:
            self.get_roi = False
            frame_gb = getMaskROI(frame_gb, self.roi, self.size)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # Convert image to LAB space

        color_area_max = None
        max_area = 0
        areaMaxContour_max = 0

        if not self.start_pick_up:
            for i in color_range:
                if i in self.target_color:
                    frame_mask = cv2.inRange(frame_lab, color_range[i][0], color_range[i][1])  # Bitwise operations on the original image and mask
                    opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # open operation
                    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # closed operation
                    contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[
                        -2]  # find the outline
                    areaMaxContour, area_max = self.getAreaMaxContour(contours)  # find the largest contour
                    if areaMaxContour is not None:
                        if area_max > max_area:  # find the largest area
                            max_area = area_max
                            color_area_max = i
                            areaMaxContour_max = areaMaxContour
            if max_area > 2500:  # have found the largest area
                self.rect = cv2.minAreaRect(areaMaxContour_max)
                box = np.int0(cv2.boxPoints(self.rect))

                self.roi = getROI(box)  # get roi region
                self.get_roi = True
                img_centerx, img_centery = getCenter(self.rect, self.roi, self.size,
                                                     square_length)  # Get the coordinates of the center of the block

                self.world_x, self.world_y = convertCoordinate(img_centerx, img_centery, self.size)  # Convert to real world coordinates

                cv2.drawContours(img, [box], -1, self.range_rgb[color_area_max], 2)
                cv2.putText(img, '(' + str(self.world_x) + ',' + str(self.world_y) + ')',
                            (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.range_rgb[color_area_max], 1)  # draw center point

                distance = math.sqrt(pow(self.world_x - self.last_x, 2) + pow(self.world_y - self.last_y, 2))  # Compare the last coordinates to determine whether to move
                self.last_x, self.last_y = self.world_x, self.world_y
                if not self.start_pick_up:
                    if color_area_max == 'red':  # red max
                        color = 1
                    elif color_area_max == 'green':  # green max
                        color = 2
                    elif color_area_max == 'blue':  # blue max
                        color = 3
                    else:
                        color = 0
                    self.color_list.append(color)
                    # Cumulative judgment
                    if distance < 0.5:
                        self.count += 1
                        self.center_list.extend((self.world_x, self.world_y))
                        if self.start_count_t1:
                            self.start_count_t1 = False
                            self.t1 = time.time()
                        if time.time() - self.t1 > 1:
                            self.rotation_angle = self.rect[2]
                            self.start_count_t1 = True
                            self.world_X, self.world_Y = np.mean(np.array(self.center_list).reshape(self.count, 2), axis=0)
                            self.center_list = []
                            self.count = 0
                            self.start_pick_up = True
                    else:
                        self.t1 = time.time()
                        self.start_count_t1 = True
                        self.center_list = []
                        self.count = 0

                    if len(self.color_list) == 3:  # multiple judgments
                        # take the average
                        color = int(round(np.mean(np.array(self.color_list))))
                        self.color_list = []
                        if color == 1:
                            self.detect_color = 'red'
                            self.draw_color = self.range_rgb["red"]
                        elif color == 2:
                            self.detect_color = 'green'
                            self.draw_color = self.range_rgb["green"]
                        elif color == 3:
                            self.detect_color = 'blue'
                            self.draw_color = self.range_rgb["blue"]
                        else:
                            self.detect_color = 'None'
                            self.draw_color = self.range_rgb["black"]
            else:
                if not self.start_pick_up:
                    self.draw_color = (0, 0, 0)
                    self.detect_color = "None"
        if self.move_square:
            cv2.putText(img, "Make sure no blocks in the stacking area", (15, int(img.shape[0] / 2)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

        cv2.putText(img, "Color: " + self.detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, self.draw_color, 2)
        return img

    def setTargetColor(self, target_color):
        self.target_color = target_color


    # Find the contour with the largest area
    # argument is a list of contours to compare
    @staticmethod
    def getAreaMaxContour(contours):
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours:  # iterate over all contours
            contour_area_temp = math.fabs(cv2.contourArea(c))  # Calculate the contour area
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  # The contour with the largest area is valid only if the area is greater than 300 to filter out the noise
                    area_max_contour = c

        return area_max_contour, contour_area_max  # returns the largest contour

    def run_perception(self, name='frame'):
        while True:
            self.sense()
            if self.image is not None:
                frame = self.process()
                self.show(name, frame)
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
