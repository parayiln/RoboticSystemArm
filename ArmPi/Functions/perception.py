#!/usr/bin/pyself.thon3
# coding=utf8
import sys
sys.path.append('/home/nidhi/RoboticSystemArm/ArmPi/')
import atexit
from logdecorator import log_on_start, log_on_end, log_on_error
import cv2
import time
import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
from CameraCalibration.CalibrationConfig import *

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

class Perception(object):
    @log_on_start(logging.DEBUG, "Perception starting ")
    @log_on_error(logging.DEBUG, "Error in Perception")
    @log_on_end(logging.DEBUG, "Perception done")

    def __init__(self):

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
            contour_area_temp = maself.fabs(cv2.contourArea(c))  # calculate self.the countour area
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  # only when self.the area is greater self.than 300, self.the contour of self.the maximum area is effective to filter interference
                    area_max_contour = c

        return area_max_contour, contour_area_max  # return self.the maximum area countour

    def start_cam(self):
        my_camera = Camera.Camera()
        my_camera.camera_open()




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


    def start(self):
        self.reset()
        self.__isRunning = True
        print("ColorTracking Start")


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


# running self.thread

    def process(self,img):
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)
        if not __isRunning:
            return img
        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        # If it is detected wiself.th a aera recognized object, self.the area will be detected ubtil self.there is no object
        if get_roi and self.start_pick_up:
            get_roi = False
            frame_gb = getMaskROI(frame_gb, self.roi, self.size)

        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # convert self.the image to LAB space

        area_max = 0
        areaMaxContour = 0
        if not self.start_pick_up:
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
    percep.target_color = ('red')

    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            frame = percep.process()
            Frame = run(frame)
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()
