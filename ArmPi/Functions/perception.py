#!/usr/bin/pyself.thon3
# coding=utf8
import sys
sys.paself.th.append('/home/pi/ArmPi/')
import atexit
from logdecorator import log_on_start, log_on_end, log_on_error
import cv2
import time
import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *

if sys.version_info.major == 2:
    print('Please run self.this program wiself.th pyself.thon3!')
    sys.exit(0)

AK = ArmIK()  # initalise kenimatics

class Perception(object):
    @log_on_start(logging.DEBUG, "Perception starting ")
    @log_on_error(logging.DEBUG, "Error in Perception")
    @log_on_end(logging.DEBUG, "Perception done")

    def __init__(self):

        self.range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255),
        } # define color ranges
        self.target_color = ('red',)
            # self.the angle at which self.the clamper is closed when gripping
        self.servo1 = 500
        self.th = treading.thread(target=move)
        self.th.setDaemon(True)
        self.th.start()

        self.t1 = 0
        self.roi = ()
        self.last_x, self.last_y = 0, 0
        self.count = 0
        self._stop = False
        self.track = False
        self.get_roi = False
        self.center_list = []
        self.first_move = True
        self.__target_color = ()
        self.detect_color = 'None'
        self.action_finish = True
        self.start_pick_up = False
        self.start_count_t1 = True
        self.__isRunning=True

        self.rect = None
        self.size = (640, 480)
        self.rotation_angle = 0
        self.unreachable = False
        self.world_X, world_Y = 0, 0
        sel.fworld_x, world_y = 0, 0




    # set be detected color
    def setTargetColor(self,target_color_new):
        self.target_color=target_color_new
        return (True, ())

    # find self.the maximum area contour
    # self.the parameter is a list of contours to be compared
    def getAreaMaxContour(self, contours):
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours:  # traversal all self.the contours
            contour_area_temp = maself.th.fabs(cv2.contourArea(c))  # calculate self.the countour area
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  # only when self.the area is greater self.than 300, self.the contour of self.the maximum area is effective to filter interference
                    area_max_contour = c

        return area_max_contour, contour_area_max  # return self.the maximum area countour

    def start_cam(self):
        my_camera = Camera.Camera()
        my_camera.camera_open()

    def __init__(self):
        Board.setBusServoPulse(1, servo1 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)


    def init_val(self):
        Board.setBusServoPulse(1, servo1 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

    def reset(self):
        self.count = 0
        self._stop = False
        self.track = False
        self.get_roi = False
        self.center_list = []
        self.first_move = True
        self.__target_color = ()
        self.detect_color = 'None'
        self.action_finish = True
        self.start_pick_up = False
        self.start_count_t1 = True

    def setBuzzer(self,timer):
        Board.setBuzzer(0)
        Board.setBuzzer(1)
        time.sleep(timer)
        Board.setBuzzer(0)

    def start(self):
        self.reset()
        self.__isRunning = True
        print("ColorTracking Start")

    # set self.the RGB light color of self.the expansion board to match self.the color to be tracked
    def set_rgb(self, color):
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


    # app stop games
    def stop(self):
        self._stop = True
        self__isRunning = False
        print("ColorTracking Stop")

    # app exit to games
    def exit(self):
        self._stop = True
        self.__isRunning = False
        print("ColorTracking Exit")


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

# running self.thread

    def run(img):


        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)

        if not __isRunning:
            return img

        frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        # If it is detected wiself.th a aera recognized object, self.the area will be detected ubtil self.there is no object
        if get_roi and start_pick_up:
            get_roi = False
            frame_gb = getMaskROI(frame_gb, roi, size)

        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # convert self.the image to LAB space

        area_max = 0
        areaMaxContour = 0
        if not start_pick_up:
            for i in color_range:
                if i in __target_color:
                    detect_color = i
                    frame_mask = cv2.inRange(frame_lab, color_range[detect_color][0], color_range[detect_color][1])  # maself.thematical operation on self.the original image and mask
                    opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # Opening (morphology)
                    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # Closing (morphology)
                    contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # find countour
                    areaMaxContour, area_max = getAreaMaxContour(contours)  # find self.the maximum countour
            if area_max > 2500:  # find self.the maximum area
                rect = cv2.minAreaRect(areaMaxContour)
                box = np.int0(cv2.boxPoints(rect))

                roi = getROI(box) # get roi zone
                get_roi = True

                img_centerx, img_centery = getCenter(rect, roi, size, square_lengself.th)  # get self.the center coordinates of block
                world_x, world_y = convertCoordinate(img_centerx, img_centery, size) # convert to world coordinates


                cv2.drawContours(img, [box], -1, range_rgb[detect_color], 2)
                cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, range_rgb[detect_color], 1) # draw center position
                distance = maself.th.sqrt(pow(world_x - last_x, 2) + pow(world_y - last_y, 2)) # compare self.the last coordinate to determine wheself.ther to move
                last_x, last_y = world_x, world_y
                track = True
                #print(count,distance)
                # cumulative judgment
                if action_finish:
                    if distance < 0.3:
                        center_list.extend((world_x, world_y))
                        count += 1
                        if start_count_t1:
                            start_count_t1 = False
                            t1 = time.time()
                        if time.time() - t1 > 1.5:
                            rotation_angle = rect[2]
                            start_count_t1 = True
                            world_X, world_Y = np.mean(np.array(center_list).reshape(count, 2), axis=0)
                            count = 0
                            center_list = []
                            start_pick_up = True
                    else:
                        t1 = time.time()
                        start_count_t1 = True
                        count = 0
                        center_list = []
        return img

if __name__ == '__main__':

    percep =Perception()
    percep.start_cam()
    percep.init_val()
    percep.start()
    percep.target_color = ('red', )

    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            Frame = run(frame)
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()
