import RPi.GPIO as GPIO
import cv2
import numpy as np
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Left Motor
in1 = 17 # Forward 
in2 = 27 # Backward
ena = 4

# Right Motor
in3 = 2 # Foward 
in4 = 3 # Backward
enb = 14

GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(ena, GPIO.OUT)

GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
GPIO.setup(enb, GPIO.OUT)

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

power_a = GPIO.PWM(ena, 100)
power_a.start(100)

power_b = GPIO.PWM(enb, 100)
power_b.start(97)

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

while(True):
	user_input = input()
	if user_input == 'c':
		forward()
	elif user_input == 'l':
		turn_left()
	elif user_input == 'r':
		turn_right()
	elif user_input == 'x':
		stop()
	
	
	
