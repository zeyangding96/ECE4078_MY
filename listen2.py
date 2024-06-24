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
def left_tick():
    global left_count
    left_count += 1
    
def right_tick():
    global right_count
    right_count += 1

left_encoder.when_activated = left_tick
right_encoder.when_activated = right_tick
left_count = 0
right_count = 0
# pid_left = PID(0.0005, 0, 0.0001, setpoint=0)
pid_right = PID(0.005, 0, 0.001, setpoint=0, output_limits=(0,1), starting_output=0.9)
flag_pid = 0
flag_forward = 1

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
    global flag_pid
    
    l_val, r_val = request.args.get('l_val'), request.args.get('r_val')
    l_val, r_val = float(l_val), float(r_val)
    # print(l_val, r_val)
    
    # if stop or turn
    if (l_val == 0 and r_val == 0) or (l_val > 0 and r_val < 0) or (l_val < 0 and r_val > 0):
        flag_pid = False
        # time.sleep(0.01)
        mbot.value = (l_val, r_val)
        
    
    # if forward
    elif (l_val > 0 and r_val > 0) and not flag_pid:
        flag_forward = True
        flag_pid = True
    
    # if backward
    elif (l_val < 0 and r_val < 0) and not flag_pid:
        flag_forward = False
        flag_pid = True
    
    return ""
    
def move_robot():
    
    global left_count, right_count, flag_pid, flag_forward
    
    # pid_left.setpoint = 9999
    # pid_right.setpoint = 9999
    
    while True:
    
        # reset pid controller (when robot stops or turn)
        if not flag_pid:
            left_count = 0
            right_count = 0
            
        else:
        
            # l_val = pid_left(left_count)
            l_val = 0.9
            pid_right.setpoint = left_count
            r_val = pid_right(right_count)
            # print(left_count, right_count)
            # print(l_val, r_val)
            
            # l_val = max(min(l_val, 1), -1)
            # r_val = max(min(r_val, 1), -1)
            
        
            if flag_forward:
                mbot.value = (l_val, r_val)
            else:
                mbot.value = (-l_val, -r_val)
        
        time.sleep(0.005)


# Run Flask in a separate thread
def run_flask():
    app.run(host='0.0.0.0', port=8000)

flask_thread = threading.Thread(target=run_flask)
flask_thread.daemon = True
flask_thread.start()

try:
    while True:
        move_robot()
except KeyboardInterrupt:
    mbot.stop()
    picam2.stop()
    print("Program interrupted by user.")