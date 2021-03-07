from flask import Flask, render_template
import RPi.GPIO as GPIO

app = Flask(__name__)

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

in1 = 18 # 左前輪
in2 = 23 # 左後輪
in3 = 24 # 右前輪
in4 = 25 # 右後輪

GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
  
@app.route("/")
@app.route("/<cmd>")
def main(cmd=None):
    status = "None..."
    # Forward    
    if cmd == 'f':
      # 左前輪true + 左後輪true = 前進 
      status = "Forward..."
      GPIO.output(in1, True)
      GPIO.output(in2, False)
      GPIO.output(in3, True)
      GPIO.output(in4, False)
    # Backward    
    if cmd == 'b':
      # 右前輪true + 右後輪true = 後退 
      status = "Backward..."
      GPIO.output(in1, False)
      GPIO.output(in2, True)
      GPIO.output(in3, False)
      GPIO.output(in4, True)
    # Turn Right   
    if cmd == 'r':
      # 左前輪true = 右轉
      status = "Turn Right..."
      GPIO.output(in1, True)
      GPIO.output(in2, False)
      GPIO.output(in3, False)
      GPIO.output(in4, False)
    # Trun Left    
    if cmd == 'l':
      # 右前輪true = 左轉
      status = "Turn Left..."
      GPIO.output(in1, False)
      GPIO.output(in2, False)
      GPIO.output(in3, True)
      GPIO.output(in4, False)
    # Stop    
    if cmd == 's':
      # 全部FALSE = 停止
      status = "Stop..."
      GPIO.output(in1, False)
      GPIO.output(in2, False)
      GPIO.output(in3, False)
      GPIO.output(in4, False)	
    templateData = {
        'title': 'Motor Control',
        'status' : status
    }
    return render_template('motor.html', **templateData)

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080, debug=True)