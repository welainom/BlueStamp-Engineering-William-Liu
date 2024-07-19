from flask import Flask, Response, render_template
import cv2
from picamera2 import Picamera2
import numpy as np
import time 
import serial
import RPi.GPIO as GPIO

app = Flask(__name__)

picam = Picamera2()
picam.preview_configuration.main.size = (640, 360)
picam.preview_configuration.main.format = "RGB888"
picam.preview_configuration.align()
picam.configure("preview")
picam.start()

#GPIO Setup
GPIO.setwarnings(False)
# Left Motor
in1 = 17 # Forward 
in2 = 27 # Backward
ena = 4
# Right Motor
in3 = 2 # Foward 
in4 = 3 # Backward
enb = 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(ena, GPIO.OUT)
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
GPIO.setup(enb, GPIO.OUT)

power_a = GPIO.PWM(ena, 100)
power_a.start(60)

power_b = GPIO.PWM(enb, 100)
power_b.start(60)

TRIG_L = 23
ECHO_L = 24
TRIG_C = 16
ECHO_C = 26
TRIG_R = 5
ECHO_R = 6

GPIO.setup(TRIG_L, GPIO.OUT)
GPIO.setup(ECHO_L, GPIO.IN)
GPIO.setup(TRIG_C, GPIO.OUT)
GPIO.setup(ECHO_C, GPIO.IN)
GPIO.setup(TRIG_R, GPIO.OUT)
GPIO.setup(ECHO_R, GPIO.IN)

GPIO.output(TRIG_L, GPIO.LOW)
GPIO.output(TRIG_C, GPIO.LOW)
GPIO.output(TRIG_R, GPIO.LOW)
time.sleep(1)

ser = serial.Serial(
    port='/dev/serial0',
    baudrate=9600,        
    timeout=1             
)

def find_distance(trig, echo):
	start = 0
	stop = 0
	
	GPIO.setup(trig, GPIO.OUT)
	GPIO.setup(echo, GPIO.IN)
	
	GPIO.output(trig, GPIO.LOW)
	time.sleep(0.01)
	
	GPIO.output(trig, GPIO.HIGH)
	time.sleep(0.00001)
	GPIO.output(trig, GPIO.LOW)
	begin = time.time()
	while GPIO.input(echo) == 0 and time.time() < begin + 0.05:
		start = time.time()
	while GPIO.input(echo) == 1 and time.time() < begin + 0.1:
		stop = time.time()
	
	elapsed = stop - start
	distance = elapsed * 34300
	distance = distance / 2
	distance = round(distance, 2)
	
	return distance

def forward():
	GPIO.output(in1, GPIO.HIGH)
	GPIO.output(in2, GPIO.LOW)
	
	GPIO.output(in3, GPIO.HIGH)
	GPIO.output(in4, GPIO.LOW)

def reverse():
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)

def stop():
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.LOW)
    
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.LOW)
    
def turn_left():
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
    
def turn_right():
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)
    
def find_red(frame):
    hsv_roi = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    lower_red1 = np.array([150, 140, 1])
    upper_red1 = np.array([190, 255, 255])
    
    mask1 = cv2.inRange(hsv_roi, lower_red1, upper_red1)
    
    mask = mask1
    
    mask = cv2.erode(mask, np.ones((3, 3), np.uint8))
    mask = cv2.dilate(mask, np.ones((8, 8), np.uint8))
    
    # cv2.imshow('mask', mask)  
    
    return mask
    
def find_blob(blob):
    largest_contour = 0
    cont_idx = 0
    contours, hierarchy = cv2.findContours(blob, cv2.RETR_CCOMP, 
                                           cv2.CHAIN_APPROX_SIMPLE)
    
    for idx, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > largest_contour):
            largest_contour = area
            cont_index = idx
                    
    r = (0, 0, 2, 2)
    
    if len(contours) > 0:
        r = cv2.boundingRect(contours[cont_index])
     
    return r, largest_contour


center_x = 0
center_y = 0
x = 0
y = 0
w = 0 
h = 0

def generate_frames():
    left_range = 100
    right_range = 540
    automatic = True
    received_data = "A"
    global center_x, center_y
    global x, y, w, h
    
    while True:
        frame = picam.capture_array()
        
        red_pixels = find_red(frame)
        loct, area = find_blob(red_pixels)
        x, y, w, h = loct

        left = find_distance(TRIG_L, ECHO_L)
        center = find_distance(TRIG_C, ECHO_C)
        right = find_distance(TRIG_R, ECHO_R)
        min_distance = min(left, center, right)
        area = w*h

        cv2.rectangle(frame, (x,y), (x+w,y+h), 255,2)
        cv2.circle(frame,(int(center_x),int(center_y)),3,(0,110,255),-1)
        ret, buffer = cv2.imencode('.jpg', frame)
        mask = buffer.tobytes()
        yield (b'--mask\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + mask + b'\r\n')    
        if ser.in_waiting > 1:
            received_data = ser.read().decode('utf-8') 
        if received_data == "A":
            automatic = True
            #print("Automatic")
        elif received_data == "M":
            automatic = False
            #print("Manual")
        
        if automatic:
            if area > 20000:
                center_x = x + (w)/2
                center_y = y + (h)/2
            if area > 200000 or min_distance < 7:
                print("object too close, reversing")
                reverse()
            elif 20000 < area and area < 100000 and left_range <= center_x <= right_range:
                forward()
                print("ball found, moving forward")
            elif 100000 < area and area < 200000:
                print("ball found, stopped")
                stop() 
            elif center_x > right_range:
                print("ball not found, turning right")
                turn_right()
                time.sleep(0.2)
                stop()
            elif center_x < left_range:
                print("ball not found, turning left")
                turn_left()
                time.sleep(0.2)
                stop()
            else:
                stop()
        else:
            received_data = ser.read().decode('utf-8')
            print(received_data)
            if received_data == "F":
                print("Foward")
                forward()
            elif received_data == "S":
                print("STOP")
                stop()
            elif received_data == "R":
                print("TURN right")
                turn_right()
            elif received_data == "L":
                print("TURN left")
                turn_left()
            elif received_data == "B":
                print("Reverse")
                reverse()
            elif received_data == "A":
                automatic = True
                print("Automatic")
            elif received_data == "M":
                automatic = False
                print("Manual")

def generate_frames2():
    while True:
        frame = picam.capture_array()
        red = find_red(frame)
        cv2.rectangle(red, (x,y), (x+w,y+h), 255,2)
        cv2.circle(red,(int(center_x),int(center_y)),3,(0,110,255),-1)
        ret, buffer = cv2.imencode('.jpg', red)
        mask = buffer.tobytes()
        yield (b'--mask\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + mask + b'\r\n')            

@app.route('/video_feed')
def video_feed_stream():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=mask')

@app.route('/video_feed_red')
def video_feed_stream2():
    return Response(generate_frames2(), mimetype='multipart/x-mixed-replace; boundary=mask')

@app.route('/side_by_side.html')
def side_by_side_html():
    return render_template('side_by_side.html')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
