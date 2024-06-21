#from simple_pid import PID
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

# Robot object
mbot = Robot(right=Motor(forward=in1, backward=in2, enable=ena), left=Motor(forward=in3, backward=in4, enable=enb))

# Initialize the PiCamera
picam2 = Picamera2()
config = picam2.create_preview_configuration(lores={"size": (640,480)})
picam2.configure(config)
# picam2.resolution = (640, 480)
# picam2.framerate = 24
# picam2.start_preview()
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
    
@app.route('/move')
def move():
    l_val, r_val = request.args.get('l_val'), request.args.get('r_val')
    l_val, r_val = float(l_val), float(r_val)
    mbot.value = (l_val, r_val)
    return


# Run Flask in a separate thread
def run_flask():
    app.run(host='0.0.0.0', port=8000)

flask_thread = threading.Thread(target=run_flask)
flask_thread.daemon = True
flask_thread.start()

# Main script example usage
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    mbot.stop()
    print("Program interrupted by user.")