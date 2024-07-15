import cv2
import numpy as np
from picamera2 import Picamera2

picam = Picamera2()
picam.preview_configuration.main.size=(640, 360)
picam.preview_configuration.main.format="RGB888"
picam.preview_configuration.align()
picam.configure("preview")
picam.start()

def find_red(frame):
    hsv_roi = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    lower_red = np.array([150, 140, 1])
    upper_red = np.array([190, 255, 255])
    
    mask1 = cv2.inRange(hsv_roi, lower_red, upper_red)
    
    mask = mask1
    
    mask = cv2.erode(mask, np.ones((3, 3), np.uint8))
    mask = cv2.dilate(mask, np.ones((8, 8), np.uint8))
    
    #cv2.imshow('mask', mask)  
    
    return mask

def find_blob(blob):
    largest_contour = 0
    cont_idx = 0
    contours, hierarchy = cv2.findContours(blob, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    
    for idx, contour in enumerate(contours):
        area=cv2.contourArea(contour)
        if (area >largest_contour) :
            largest_contour=area
            cont_index=idx
                    
    r=(0,0,2,2)
    
    if len(contours) > 0:
        r = cv2.boundingRect(contours[cont_index])
     
    return r,largest_contour

center_x = 0
center_y = 0
left_range = 200
right_range = 440
while True:
    video = picam.capture_array()
    red_pixels = find_red(video)
    loct, area = find_blob(red_pixels)
    x, y, w, h = loct
    
    if (w*h) > 10000:
        simg2 = cv2.rectangle(video, (x,y), (x+w,y+h), 255,2)
        center_x=x+((w)/2)
        center_y=y+((h)/2)
        cv2.circle(video,(int(center_x),int(center_y)),3,(0,110,255),-1)
        print("ball found")
    else:
        if center_x > right_range:
            print("ball not found, turning right")
        elif center_x < left_range:
            print("ball not found, turning left")
        else:
            print("balal not found")
        
    cv2.imshow('video', video)
        
    if cv2.waitKey(1) == ord('q'):
        break
        
cv2.destroyAllWindows()
    
    
    
