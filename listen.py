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
        # self.encoder.when_deactivated = self._increment
    
    def reset(self):
        self._value = 0
    
    def _increment(self):
        self._value += 1
        
    @property
    def value(self):
        return self._value
        

# main function to control the robot wheels
def move_robot():
    global use_pid, l_vel, r_vel
    flag_new_pid_cycle = True
    while True:
        ### if not using pid, just move the wheels as commanded
        if not use_pid:
            pibot.value = (l_vel, r_vel)          
        
        ### with pid, left wheel is set as reference, and right wheel will try to match the encoder counter of left wheel
        ### pid only runs when robot moves forward or backward. Turning does not use pid
        else:
            if (motion == 'stop') or (motion == 'turning'):
                pibot.value = (l_vel, r_vel) 
                left_encoder.reset()
                right_encoder.reset()
                flag_new_pid_cycle = True          
            else:
                l_vel, r_vel = abs(l_vel), abs(r_vel)
                if flag_new_pid_cycle:
                    pid_right = PID(kp, ki, kd, setpoint=left_encoder.value, output_limits=(0,1), starting_output=r_vel)
                    flag_new_pid_cycle = False
                pid_right.setpoint = left_encoder.value
                r_vel = pid_right(right_encoder.value)
                if motion == 'forward': pibot.value = (l_vel, r_vel)
                else: pibot.value = (-l_vel, -r_vel)
                print('Value', left_encoder.value, right_encoder.value)
                print('Speed', l_vel, r_vel)
        time.sleep(0.01)
    
    
# Receive confirmation whether to use pid or not to control the wheels (forward & backward)
@app.route('/pid')
def set_pid():
    global use_pid, kp, ki, kd
    use_pid = int(request.args.get('use_pid'))
    if use_pid:
        kp, ki, kd = float(request.args.get('kp')), float(request.args.get('ki')), float(request.args.get('kd'))
        return "Using PID"
    else:
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
    global l_vel, r_vel, motion
    l_vel, r_vel = float(request.args.get('l_vel')), float(request.args.get('r_vel'))
    
    if (l_vel == 0 and r_vel == 0):
        motion = 'stop'
    elif (l_vel != r_vel ):
        motion = 'turning'
    elif (l_vel > 0 and r_vel > 0):
        motion = 'forward'
    elif (l_vel < 0 and r_vel < 0):
        motion = 'backward'
    return motion
    
    # if 'time' in request.args:


# Constants
in1 = 27 # may have to change this
in2 = 17 # may have to change this
ena = 18
in3 = 23 # may have to change this
in4 = 24 # may have to change this
enb = 25
enc_a = 26
enc_b = 16

# Initialize robot and encoders
pibot = Robot(right=Motor(forward=in1, backward=in2, enable=ena), left=Motor(forward=in3, backward=in4, enable=enb))

# Initialize PID controllers for wheels
left_encoder = Encoder(enc_a)
right_encoder = Encoder(enc_b)
use_pid = 0
kp = 0
ki = 0
kd = 0
l_vel, r_vel = 0, 0
motion = ''

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
except KeyboardInterrupt:
    pibot.stop()
    picam2.stop()
    print("Program interrupted by user.")