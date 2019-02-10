import cv2 as cv2
import numpy as np
import math
   
centerx = 640
centery = 360
focal = 640
camera_height = 30
target_bottom_height=40
target_height = 5
target_width = 15
camera_fov = 70

def hsl_threshold(input, hue, sat, lum):
	"""Segment an image based on hue, saturation, and luminance ranges.
	Args:
		input: A BGR numpy.ndarray.
		hue: A list of two numbers the are the min and max hue.
		sat: A list of two numbers the are the min and max saturation.
		lum: A list of two numbers the are the min and max luminance.
	Returns:
		A black and white numpy.ndarray.
	"""
	out = cv2.cvtColor(input, cv2.COLOR_BGR2HLS)
	return cv2.inRange(out, (hue[0], lum[0], sat[0]),  (hue[1], lum[1], sat[1]))


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
	im2, contours, hierarchy =cv2.findContours(input, mode=mode, method=method)
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
def getAngle(center):
        return math.atan(center/focal) * 180 / math.pi;
def getDistance(angle):
        return (target_bottom_height + (target_height/2) - camera_height) / math.tan((verticalangle + camera_angle) * math.pi / 180) ;
def process(img):

        hsl_threshold_hue = [1.0791356615025363, 74.53925305259105]
        hsl_threshold_saturation = [30.575547587099685, 123.00341850254723]
        hsl_threshold_luminance = [214.02876531477455, 255.0]

        threshold_output = None




        """
        Runs the pipeline and sets all outputs to new values.
        """
        # Step HSL_Threshold0:
        hsl_threshold_input = img
        (hsl_threshold_output) = hsl_threshold(hsl_threshold_input, hsl_threshold_hue, hsl_threshold_saturation, hsl_threshold_luminance)

        find_contours_input = hsl_threshold_output
        find_contours_external_only = False

        find_contours_output = None

        # Step Find_Contours0:
        find_contours_input = hsl_threshold_output
        (find_contours_output) = find_contours(find_contours_input, find_contours_external_only)

        filter_contours_contours = find_contours_output
        filter_contours_min_area = 100.0
        filter_contours_min_perimeter = 0
        filter_contours_min_width = 0
        filter_contours_max_width = 1000
        filter_contours_min_height = 0
        filter_contours_max_height = 1000
        filter_contours_solidity = [79.73620617132391, 100.0]
        filter_contours_max_vertices = 200.0
        filter_contours_min_vertices = 4.0
        filter_contours_min_ratio = 0
        filter_contours_max_ratio = 1000

        filter_contours_output = None

        # Step Filter_Contours0:
        filter_contours_contours = find_contours_output
        (filter_contours_output) = filter_contours(filter_contours_contours, filter_contours_min_area, filter_contours_min_perimeter, filter_contours_min_width, filter_contours_max_width, filter_contours_min_height, filter_contours_max_height, filter_contours_solidity, filter_contours_max_vertices, filter_contours_min_vertices, filter_contours_min_ratio, filter_contours_max_ratio)

        #print(len(filter_contours_output))
        contours= filter_contours_output
        cv2.drawContours(img, contours, -1, (0,255,0), 3)
        if(len(contours)==2):
                cnt = contours[0]
                cnt2 = contours[1]
                x,y,w,h = cv2.boundingRect(cnt)
                x2,y2,w2,h2 = cv2.boundingRect(cnt2)
                if(x<x2):
                        cv2.rectangle(img,(x,y),(x2+w2,y2+h2),(0,255,0),2)
                        cx = x + (x2+w2-x)/2
                        cy = y + (y2+h2-y)/2
                        
                else:
                        cv2.rectangle(img,(x2,y2),(x+w,y+h),(0,255,0),2)
                        cx = x2 + (x+w-x2)/2
                        cy = y2 + (y+h-y2)/2 
                cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
                cv2.rectangle(img,(x2,y2),(x2+w2,y2+h2),(0,0,255),2)

                print(getAngle(cx))

        return img

cap = cv2.VideoCapture(2)
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    # Our operations on the frame come here
    img = process(frame)
    cv2.imshow('frame',img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
