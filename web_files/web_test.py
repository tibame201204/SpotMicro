#!/usr/bin/env python
from flask import Flask, render_template, Response
from time import sleep
import cv2

app = Flask(__name__)

@app.route('/')
def index():
    """Video streaming home page."""
    return render_template('index.html')

@app.route("/f")
def go_forward():
    # forward()
    # sleep(delay)
    # stop()
    print('Forward...')
    return "Forward..."

@app.route("/b")
def go_backward():
    # backward()
    # sleep(delay)
    # stop()
    print('Backward...')
    return "Backward..."

@app.route("/r")
def go_turn_right():
    # turn_right()
    # sleep(delay)
    # stop()
    print('Turn Right...')
    return "Turn Right..."
    
@app.route("/l")
def go_turn_left():
    # turn_left()
    # sleep(delay)
    # stop()
    print('Turn Left...')
    return "Turn Left..."

@app.route("/s")
def go_stope():
    pritn('Stop...')
    # stop()
    return "Stop..."

def gen():
    cap = cv2.VideoCapture(0)
    """Video streaming generator function."""
    while True:
        # frame = camera.get_frame()
        ret, frame = cap.read()
        cv2.imshow("basic_test", frame)
        # 按下 q 鍵離開迴圈
        if cv2.waitKey(33) == ord('q'):break
        # 釋放該攝影機裝置
        # cap.release()
        # cv2.destroyWindow("basic_test")

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/video_feed')
def video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True, threaded=True)
