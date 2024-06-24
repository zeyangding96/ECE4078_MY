from simple_pid import PID
from picamera2 import Picamera2, Preview
from flask import Flask, Response, request
from gpiozero import Robot, Motor, DigitalInputDevice
import io
import time
import threading


# Constants
in1 = 17
in2 = 27
ena = 18
in3 = 23
in4 = 24
enb = 25
enc_a = 26
enc_b = 16

# Initialize robot and encoders
mbot = Robot(right=Motor(forward=in1, backward=in2, enable=ena), left=Motor(forward=in3, backward=in4, enable=enb))

# Initialize PID controllers for wheels
left_encoder = DigitalInputDevice(enc_a)
right_encoder = DigitalInputDevice(enc_b)
left_encoder.when_activated = left_tick
right_encoder.when_activated = right_tick
left_count = 0
right_count = 0
pid_left = PID(1, 0.1, 0.05, setpoint=0)
pid_right = PID(1, 0.1, 0.05, setpoint=0)

# Initialize the PiCamera
picam2 = Picamera2()
config = picam2.create_preview_configuration(lores={"size": (640,480)})
picam2.configure(config)
picam2.start()
time.sleep(2)

# Initialize flask
app = Flask(__name__)

@app.route('/image')
def capture_image():
    stream = io.BytesIO()
    picam2.capture_file(stream, format='jpeg')
    stream.seek(0)
    return Response(stream, mimetype='image/jpeg')
    
# @app.route('/move')
# def move():
    # l_val, r_val = request.args.get('l_val'), request.args.get('r_val')
    # l_val, r_val = float(l_val), float(r_val)
    # mbot.value = (l_val, r_val)
    # return ""
    
@app.route('/move')
def move():
    l_val, r_val = request.args.get('l_val'), request.args.get('r_val')
    l_val, r_val = float(l_val), float(r_val)
    
    # if stop or turn
    if (l_val == 0 and r_val == 0) or (l_val > 0 and r_val < 0) or (l_val < 0 and r_val > 0):
        mbot.value = (l_val, r_val)
    
    # if forward
    elif (l_val > 0 and r_val > 0):
        move_robot(forward=True)
    
    # if backward
    elif (l_val < 0 and r_val < 0):
        move_robot(forward=False)  
    
    return ""
    
def left_tick():
    global left_count
    left_count += 1
    
def right_tick():
    global right_count
    right_count += 1
    
def move_robot(forward=True):
    global stop_event
    stop_event.clear()
    
    global left_count, right_count
    left_count = 0
    right_count = 0
    
    pid_left.setpoint = 9999
    pid_right.setpoint = 9999
    
    while not stop_event.is_set():
        l_val = pid_left(left_count)
        r_val = pid_right(right_count)
        
        l_val = max(min(l_val, 1), -1)
        r_val = max(min(r_val, 1), -1)
        print(l_val, r_val)
        
        if forward:
            mbot.value = (l_val, r_val)
        else:
            mbot.value = (-l_val, -r_val)
        
        time.sleep(0.01)
    
    mbot.stop()



# Run Flask in a separate thread
def run_flask():
    app.run(host='0.0.0.0', port=8000)

stop_event = threading.Event()
flask_thread = threading.Thread(target=run_flask)
flask_thread.daemon = True
flask_thread.start()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    stop_event.set()
    mbot.stop()
    picame2.stop()
    print("Program interrupted by user.")