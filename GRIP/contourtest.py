import numpy as np
import cv2

im = cv2.imread('images/2019-01-09_00-57-35-005.png')
imgray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
ret,thresh = cv2.threshold(imgray,127,255,0)
im2,contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(im, contours, -1, (0,255,0), 3)
cv2.imshow('frame',im)
if cv2.waitKey(1) & 0xFF == ord('q'):
    
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
