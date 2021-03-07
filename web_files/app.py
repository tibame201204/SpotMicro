#!/usr/bin/env python
from flask import Flask, render_template, Response
import RPi.GPIO as GPIO
from time import sleep

# emulated camera
#from camera import Camera

# Raspberry Pi camera module (requires picamera package)
from camera_pi import Camera

app = Flask(__name__)

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

in1 = 18
in2 = 23
in3 = 24
in4 = 25

delay = 1

GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)

def stop():
    GPIO.output(in1, False)
    GPIO.output(in2, False)
    GPIO.output(in3, False)
    GPIO.output(in4, False)
    
def forward():
    GPIO.output(in1, True)
    GPIO.output(in2, False)
    GPIO.output(in3, True)
    GPIO.output(in4, False)

def backward():
    GPIO.output(in1, False)
    GPIO.output(in2, True)
    GPIO.output(in3, False)
    GPIO.output(in4, True)
    
def turn_right():
    GPIO.output(in1, False)
    GPIO.output(in2, False)
    GPIO.output(in3, True)
    GPIO.output(in4, False)
    
def turn_left():
    GPIO.output(in1, True)
    GPIO.output(in2, False)
    GPIO.output(in3, False)
    GPIO.output(in4, False)

@app.route('/')
def index():
    """Video streaming home page."""
    return render_template('index.html')

@app.route("/f")
def go_forward():
    forward()
    sleep(delay)
    stop()
    return "Forward..."

@app.route("/b")
def go_backward():
    backward()
    sleep(delay)
    stop()
    return "Backward..."

@app.route("/r")
def go_turn_right():
    turn_right()
    sleep(delay)
    stop()
    return "Turn Right..."
    
@app.route("/l")
def go_turn_left():
    turn_left()
    sleep(delay)
    stop()
    return "Turn Left..."

@app.route("/s")
def go_stope():
    stop()
    return "Stop..."

def gen(camera):
    """Video streaming generator function."""
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/video_feed')
def video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(gen(Camera()),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True, threaded=True)
