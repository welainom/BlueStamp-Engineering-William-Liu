from flask import Flask, Response, render_template
import cv2
from picamera2 import Picamera2
import numpy as np

app = Flask(__name__)

picam = Picamera2()
picam.preview_configuration.main.size = (640, 360)
picam.preview_configuration.main.format = "RGB888"
picam.preview_configuration.align()
picam.configure("preview")
picam.start()

def find_red(frame):
    hsv_roi = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    lower_red1 = np.array([150, 140, 1])
    upper_red1 = np.array([190, 255, 255])
    
    mask1 = cv2.inRange(hsv_roi, lower_red1, upper_red1)
    
    mask = mask1
    
    mask = cv2.erode(mask, np.ones((3, 3), np.uint8))
    mask = cv2.dilate(mask, np.ones((8, 8), np.uint8))
    
    return mask

def generate_frames():
    while True:
        frame = picam.capture_array()
        ret, buffer = cv2.imencode('.jpg', frame)
        mask = buffer.tobytes()
        yield (b'--mask\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + mask + b'\r\n')

def generate_frames2():
    while True:
        frame = picam.capture_array()
        red = find_red(frame)
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
