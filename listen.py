from simple_pid import PID
from picamera2 import Picamera2
from flask import Flask, Response, request
from gpiozero import Robot, Motor, DigitalInputDevice
import io
import time
import threading


app = Flask(__name__)


class Encoder(object):
    def __init__(self, pin):
        self._value = 0
        self.encoder = DigitalInputDevice(pin)
        self.encoder.when_activated = self._increment
        self.encoder.when_deactivated = self._increment
    
    def reset(self):
        self._value = 0
    
    def _increment(self):
        self._value += 1
        #print(self._value)
        
    @property
    def value(self):
        return self._value
        

# main function to control the robot wheels
def move_robot():
    
    global left_count, right_count, use_pid, flag_forward
    
    # pid_left.setpoint = 9999
    # pid_right.setpoint = 9999
    
    while True:
    
        # reset pid controller (when robot stops or turn)
        if not use_pid:
            left_count = 0
            right_count = 0
            
        else:       
            # l_vel = pid_left(left_count)
            l_vel = 0.9
            pid_right.setpoint = left_count
            r_vel = pid_right(right_count)
            # print(left_count, right_count)
            # print(l_vel, r_vel)
            
            # l_vel = max(min(l_vel, 1), -1)
            # r_vel = max(min(r_vel, 1), -1)
            
        
            if flag_forward:
                pibot.value = (l_vel, r_vel)
            else:
                pibot.value = (-l_vel, -r_vel)
        
        time.sleep(0.005)
    
    
# Receive confirmation whether to use pid or not to control the wheels (forward & backward)
@app.route('/pid')
def set_pid():
    global use_pid, kp, ki, kd
    use_pid = bool(request.args.get('use_pid'))
    print(use_pid)
    print(type(use_pid))
    if use_pid:
        kp, ki, kd = float(request.args.get('kp')), float(request.args.get('ki')), float(request.args.get('kd'))
        print("Using PID")
        return "Using PID"
    else:
        print("Not using PID")
        return "Not using PID"
    
# Receive a request to capture and send a snapshot of the picamera
@app.route('/image')
def capture_image():
    stream = io.BytesIO()
    picam2.capture_file(stream, format='jpeg')
    stream.seek(0)
    return Response(stream, mimetype='image/jpeg')
    

 # Receive command to move the pibot
@app.route('/move')
def move():
    
    l_vel, r_vel = float(request.args.get('l_vel')), float(request.args.get('r_vel'))
    
    # if 'time' in request.args:
    
    if use_pid:
     
        # stop or turn
        if (l_vel == 0 and r_vel == 0) or (l_vel != r_vel ):
            use_pid = False
            pibot.value = (l_vel, r_vel)
                    
        # forward
        elif (l_vel > 0 and r_vel > 0) and not use_pid:
            flag_forward = True
            use_pid = True
        
        # backward
        elif (l_vel < 0 and r_vel < 0) and not use_pid:
            flag_forward = False
            use_pid = True
    
    # Not using pid
    else:
        pibot.value = (l_vel, r_vel)
    
    return ""


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
pibot = Robot(right=Motor(forward=in1, backward=in2, enable=ena), left=Motor(forward=in3, backward=in4, enable=enb))

# Initialize PID controllers for wheels
left_encoder = Encoder(enc_a)
right_encoder = Encoder(enc_b)
# pid_left = PID(0.001, 0, 0.0005, setpoint=0)
pid_right = PID(0.001, 0, 0.0005, setpoint=0, output_limits=(0,1), starting_output=0.9)
use_pid = 0
kp, ki, kd = 0, 0, 0
flag_forward = 1

# Initialize the PiCamera
picam2 = Picamera2()
config = picam2.create_preview_configuration(lores={"size": (640,480)})
picam2.configure(config)
picam2.start()
time.sleep(2)

# Initialize flask
def run_flask():
    app.run(host='0.0.0.0', port=5000)
flask_thread = threading.Thread(target=run_flask)
flask_thread.daemon = True
flask_thread.start()

try:
    while True:
        move_robot()
        # time.sleep(2)
except KeyboardInterrupt:
    pibot.stop()
    picam2.stop()
    print("Program interrupted by user.")