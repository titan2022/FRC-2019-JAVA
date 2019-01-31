#!/usr/bin/env python3

import cv2
import numpy as np
import math
import sys
import json
import time
from networktables import NetworkTablesInstance
from cscore import CameraServer, VideoSource
import cscore
import datetime


# To see messages from networktables, you must setup logging
#import logging

#logging.basicConfig(level=logging.DEBUG)

# As a client to connect to a robot
class CameraConfig: pass


centerx = 640
centery = 360 
camera_height = 6
target_bottom_height=25.5
target_height = 6
target_width = 13
camera_fov_x = 60
camera_fov_y= 40
camera_angle = 30


team = None
server = False
cameraConfigs = []
configFile = "/boot/frc.json"

"""Report parse error."""
def parseError(str):
        print("config error in '" + configFile + "': " + str, file=sys.stderr)

"""Read single camera configuration."""
def readCameraConfig(config):
        cam = CameraConfig()

        # name
        try:
                cam.name = config["name"]
        except KeyError:
                parseError("could not read camera name")
                return False

        # path
        try:
                cam.path = config["path"]
        except KeyError:
                parseError("camera '{}': could not read path".format(cam.name))
                return False

        cam.config = config

        cameraConfigs.append(cam)
        return True


def readConfig():
        global team
        global server

        # parse file
        try:
                with open(configFile, "rt") as f:
                        j = json.load(f)
        except OSError as err:
                print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
                return False

        # top level must be an object
        if not isinstance(j, dict):
                parseError("must be JSON object")
                return False

        # team number   
        try:
                team = j["team"]
        except KeyError:
                parseError("could not read team number")
                return False

        # ntmode (optional)
        if "ntmode" in j:
                str = j["ntmode"]
                if str.lower() == "client":
                        server = False
                elif str.lower() == "server":
                        server = True
                else:
                        parseError("could not understand ntmode value '{}'".format(str))

        # cameras
        try:
                cameras = j["cameras"]
        except KeyError:
                parseError("could not read cameras")
                return False
        for camera in cameras:
                if not readCameraConfig(camera):
                        return False

        return True
def rgb_threshold(input, red, green, blue):
        """Segment an image based on color ranges.
        Args:
            input: A BGR numpy.ndarray.
            red: A list of two numbers the are the min and max red.
            green: A list of two numbers the are the min and max green.
            blue: A list of two numbers the are the min and max blue.
        Returns:
            A black and white numpy.ndarray.
        """
        out = cv2.cvtColor(input, cv2.COLOR_BGR2RGB)
        return cv2.inRange(out, (red[0], green[0], blue[0]),  (red[1], green[1], blue[1]))


def find_contours(input, external_only):
	"""Sets the values of pixels in a binary image to their distance to the nearest black pixel.
	Args:
		input: A numpy.ndarray.
		external_only: A boolean. If true only external contours are found.
	Return:
		A list of numpy.ndarray where each one represents a contour.
	"""
	if(external_only):
		mode = cv2.RETR_EXTERNAL
	else:
		mode = cv2.RETR_LIST
	method = cv2.CHAIN_APPROX_SIMPLE
	im2, contours, hierarchy = cv2.findContours(input, mode=mode, method=method)
	return contours


def filter_contours(input_contours, min_area, min_perimeter, min_width, max_width,
					min_height, max_height, solidity, max_vertex_count, min_vertex_count,
					min_ratio, max_ratio):
	"""Filters out contours that do not meet certain criteria.
	Args:
		input_contours: Contours as a list of numpy.ndarray.
		min_area: The minimum area of a contour that will be kept.
		min_perimeter: The minimum perimeter of a contour that will be kept.
		min_width: Minimum width of a contour.
		max_width: MaxWidth maximum width.
		min_height: Minimum height.
		max_height: Maximimum height.
		solidity: The minimum and maximum solidity of a contour.
		min_vertex_count: Minimum vertex Count of the contours.
		max_vertex_count: Maximum vertex Count.
		min_ratio: Minimum ratio of width to height.
		max_ratio: Maximum ratio of width to height.
	Returns:
		Contours as a list of numpy.ndarray.
	"""
	output = []
	for contour in input_contours:
		x,y,w,h = cv2.boundingRect(contour)
		if (w < min_width or w > max_width):
			continue
		if (h < min_height or h > max_height):
			continue
		area = cv2.contourArea(contour)
		if (area < min_area):
			continue
		if (cv2.arcLength(contour, True) < min_perimeter):
			continue
		hull = cv2.convexHull(contour)
		solid = 100 * area / cv2.contourArea(hull)
		if (solid < solidity[0] or solid > solidity[1]):
			continue
		if (len(contour) < min_vertex_count or len(contour) > max_vertex_count):
			continue
		ratio = (float)(w) / h
		if (ratio < min_ratio or ratio > max_ratio):
			continue
		output.append(contour)

	return output
def getAngleX(center):
        center = center - 1280/2
        return math.atan(2*center*math.tan((60 * math.pi / 180 )/2)/1280) * 180 / math.pi;
def getAngleY(center):
        center = center - 720/2
        return math.atan(2*center*math.tan((camera_fov_y * math.pi / 180 )/2)/720) * 180 / math.pi * -1 + camera_angle;
def getDistance(angle):
        return (35 / math.tan(angle* math.pi / 180))

#code moved from process function
im = cv2.imread('images/rect.jpg')

rgb_threshold_red = [0.0, 68.0]
rgb_threshold_green = [92, 255.0]
rgb_threshold_blue = [0.0, 92]
rgb_threshold_output = None
#code moved from process function end

def process(img):
        """
        Runs the pipeline and sets all outputs to new values.
        """
        # Step HSL_Threshold0:
        rgb_threshold_input = img
        (rgb_threshold_output) = rgb_threshold(rgb_threshold_input, rgb_threshold_red, rgb_threshold_green, rgb_threshold_blue)

        find_contours_input = rgb_threshold_output
        find_contours_external_only = False

        find_contours_output = None

        # Step Find_Contours0:
        find_contours_input = rgb_threshold_output
        (find_contours_output) = find_contours(find_contours_input, find_contours_external_only)

        filter_contours_contours = find_contours_output
        filter_contours_min_area = 60
        filter_contours_min_perimeter = 10.0
        filter_contours_min_width = 0.0
        filter_contours_max_width = 1000.0
        filter_contours_min_height = 0.0
        filter_contours_max_height = 1000.0
        filter_contours_solidity = [0.0, 100.0]
        filter_contours_max_vertices = 5000.0
        filter_contours_min_vertices = 0.0
        filter_contours_min_ratio = 0.0
        filter_contours_max_ratio = 1000.0


        filter_contours_output = None

        # Step Filter_Contours0:
        filter_contours_contours = find_contours_output
        (filter_contours_output) = filter_contours(filter_contours_contours, filter_contours_min_area, filter_contours_min_perimeter, filter_contours_min_width, filter_contours_max_width, filter_contours_min_height, filter_contours_max_height, filter_contours_solidity, filter_contours_max_vertices, filter_contours_min_vertices, filter_contours_min_ratio, filter_contours_max_ratio)

        #print(len(filter_contours_output))
        contours= filter_contours_output
        output = []
        for contour in contours:
            output.append(cv2.convexHull(contour))
        contours = output
        #cv2.drawContours(img, contours, -1, (255,255,0), 3)
        ret = 1000
        cnt = None
        cnt2 = None
        
        
        imgray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(imgray, 127, 255, 0)
        im2, rectcontours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        rect = rectcontours[0]

        double thres = 1;
        
        
        for i in contours:
                ret = cv2.matchShapes(rect,i,1,0.0)
                if(ret < cntscore):
                        if cnt is not None:
                                cnt2score = cntscore
                                cnt2 = cnt
                        cntscore = ret
                        cnt = i
                elif(ret < cnt2score):
                        cnt2score = ret
                        cnt2 = i
                        
        
        if cnt is not None and cnt2 is not None:
                
                cv2.drawContours(img, [cnt], 0, (0,0,122), 3)

                cv2.drawContours(img, [cnt2], 0, (0,0,122), 3)
                
                x,y,w,h = cv2.boundingRect(cnt)
                x2,y2,w2,h2 = cv2.boundingRect(cnt2)
                if(x<x2):
                        if(y<y2):
                                
                                cv2.rectangle(img,(x,y),(x2+w2,y2+h2),(0,255,0),2)
                                cx = x + (x2+w2-x)/2
                                cy = y + (y2+h2-y)/2
                                
                        else:
                                cv2.rectangle(img,(x,y2),(x2+w2,y+h2),(0,255,0),2)
                                cx = x + (x2+w2-x)/2
                                cy = y + (y2+h2-y)/2

                        
                                        
                else:
                        cv2.rectangle(img,(x2,y2),(x+w,y+h),(0,255,0),2)
                        cx = x2 + (x+w-x2)/2
                        cy = y2 + (y+h-y2)/2 
                cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
                cv2.rectangle(img,(x2,y2),(x2+w2,y2+h2),(0,0,255),2)
                #print(getAngleX(cx))
                y_angle = getAngleY(cy)
                #print("y_angle" +str(y_angle))
                #print("x_angle" + str(getAngleX(cx)))
                sd.putNumber("y_angle", y_angle)
                sd.putNumber("x_angle", getAngleX(cx))

                distance = getDistance(y_angle)
                #print("distance" + str(distance))

                sd.putNumber("distance", distance)
                #print(distance)
        return img

def startCamera(config):
        print("Starting camera '{}' on {}".format(config.name, config.path))
        #camera = CameraServer.getInstance().startAutomaticCapture(name=config.name, path=config.path)
        camera = cscore.UsbCamera(name=config.name, path=config.path)
        camera.setConfigJson(json.dumps(config.config))

        

        return camera
if __name__ == "__main__":
        if len(sys.argv) >= 2:
                configFile = sys.argv[1]

        # read configuration
        if not readConfig():
                sys.exit(1)
        ntinst = NetworkTablesInstance.getDefault()
        if server:
                print("Setting up NetworkTables server")
                ntinst.startServer()
        else:
                print("Setting up NetworkTables client for team {}".format(team))
                ntinst.startClientTeam(team)

        sd = ntinst.getTable('SmartDashboard')
        #cap = cv2.VideoCapture(0)
        #cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        #cap.set(cv2.CAP_PROP_EXPOSURE, -50)
        #cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        #cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cameras = []
        for cameraConfig in cameraConfigs:
                cameras.append(startCamera(cameraConfig))
        cam = cameras[0]
        #cam.setResolution(1280, 720)
        cs = CameraServer.getInstance()
        cvSink = cs.getVideo(camera=cam)

        rawPropertyStream = cs.putVideo("Stream", 1280, 720)
                 = cs.putVideo("StreanProcessed", 1280, 720)

        img = np.zeros([1280,720,3],dtype=np.uint8)
        while(True):
                t1 = datetime.datetime.now()
                print("grabbing frame")

                time, img2 = cvSink.grabFrame(img, 1)
                #rawPropertyStream.putFrame(img2)
                t2 = datetime.datetime.now()
                d1 = t2-t1
                print("Grabbed Frame: " +  str(d1.total_seconds()))
                process(img2)

                t3 = datetime.datetime.now()
                d2 = t3-t2
                print("process frame: " + str(d2.total_seconds()))

                processedstream.putFrame(process(img2))
                t4 = datetime.datetime.now()
                d3 = t4-t3

                print("put frame: " + str(d3.total_seconds()))

                #cv2.imshow('frame',img)
                
        # When everything done, release the capture
        cap.release()
