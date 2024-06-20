#from simple_pid import PID
from picamera2 import Picamera2, Preview
from flask import Flask, Response, request
#from gpiozero import Robot, Motor, DigitalInputDevice
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
# mbot = Robot(right=Motor(forward=in1, backward=in2, enable=ena), left=Motor(forward=in3, backward=in4, enable=enb))

# Initialize the PiCamera
print('a')
picam2 = Picamera2()
print('b')
camera_config = picam2.create_preview_configuration()
picam2.configure(camera_config)
picam2.resolution = (640, 480)
picam2.framerate = 24
picam2.start_preview(Preview.DRM)
picam2.start()
print('c')
time.sleep(2)

def capture_image():
    stream = io.BytesIO()
    camera.capture(stream, 'jpeg')
    stream.seek(0)
    return stream.read()

@app.route('/image')
def serve_image():
    return Response(capture_image(), mimetype='image/jpeg')
    


# Run Flask in a separate thread
def run_flask():
    app.run(host='0.0.0.0', port=5000)

flask_thread = threading.Thread(target=run_flask)
flask_thread.daemon = True
flask_thread.start()

# Main script example usage
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    robot.stop()
    print("Program interrupted by user.")