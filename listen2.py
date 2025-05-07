import socket
import struct
import io
import threading
import time
import RPi.GPIO as GPIO
import numpy as np
from picamera2 import Picamera2
from libcamera import Transform

# Network Configuration
HOST = '0.0.0.0'  # Listen on all interfaces
PORT = 8000       # Main Command port
IMAGE_PORT = 8001 # Camera stream port
PID_CONFIG_PORT = 8002   # PID configuration port

# Motor Control Pins
LEFT_MOTOR_ENA = 18
LEFT_MOTOR_IN1 = 17
LEFT_MOTOR_IN2 = 27
RIGHT_MOTOR_ENB = 25
RIGHT_MOTOR_IN3 = 23
RIGHT_MOTOR_IN4 = 24

# Encoder Pins
LEFT_ENCODER = 5
RIGHT_ENCODER = 6

# PID Constants (default values, will be overridden by client)
use_PID = False
KP = 0.5
KI = 0.1
KD = 0.01
MAX_CORRECTION = 20  # Maximum PWM correction value

# Global variables
running = True
left_pwm = 0
right_pwm = 0
left_count = 0
right_count = 0

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Motor
    GPIO.setup(LEFT_MOTOR_ENA, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_IN1, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_IN2, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_ENB, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_IN3, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_IN4, GPIO.OUT)
    
    # Encoder
    GPIO.setup(LEFT_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(RIGHT_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    # Encoder interrupt (when activated)
    GPIO.add_event_detect(LEFT_ENCODER, GPIO.FALLING, callback=left_encoder_callback)
    GPIO.add_event_detect(RIGHT_ENCODER, GPIO.FALLING, callback=right_encoder_callback)
    
    # Initialize PWM (frequency: 100Hz)
    global left_motor_pwm, right_motor_pwm
    left_motor_pwm = GPIO.PWM(LEFT_MOTOR_ENA, 100)
    right_motor_pwm = GPIO.PWM(RIGHT_MOTOR_ENB, 100)
    left_motor_pwm.start(0)
    right_motor_pwm.start(0)

def left_encoder_callback(channel):
    global left_count
    left_count += 1

def right_encoder_callback(channel):
    global right_count
    right_count += 1

def set_motors(left, right):
    if left >= 0:
        GPIO.output(LEFT_MOTOR_IN1, GPIO.HIGH)
        GPIO.output(LEFT_MOTOR_IN2, GPIO.LOW)
    else:
        GPIO.output(LEFT_MOTOR_IN1, GPIO.LOW)
        GPIO.output(LEFT_MOTOR_IN2, GPIO.HIGH)
    
    if right >= 0:
        GPIO.output(RIGHT_MOTOR_IN3, GPIO.HIGH)
        GPIO.output(RIGHT_MOTOR_IN4, GPIO.LOW)
    else:
        GPIO.output(RIGHT_MOTOR_IN3, GPIO.LOW)
        GPIO.output(RIGHT_MOTOR_IN4, GPIO.HIGH)
    
    left_motor_pwm.ChangeDutyCycle(min(abs(left), 100))
    right_motor_pwm.ChangeDutyCycle(min(abs(right), 100))

def pid_control():
    # Only when moving forward or backward, not turning
    global left_pwm, right_pwm, left_count, right_count, KP, KI, KD
    
    integral = 0
    last_error = 0
    last_time = time.time()
    
    while running:
        # Calculate time delta
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        
        # If both motors are supposed to move forward or backward (not turning)
        if (left_pwm > 0 and right_pwm > 0) or (left_pwm < 0 and right_pwm < 0):
            # Calculate error (difference between encoder counts)
            error = left_count - right_count
            
            # Calculate PID terms
            proportional = KP * error
            integral += KI * error * dt
            integral = max(-MAX_CORRECTION, min(integral, MAX_CORRECTION))  # Anti-windup
            derivative = KD * (error - last_error) / dt if dt > 0 else 0
            
            # Calculate correction
            correction = proportional + integral + derivative
            correction = max(-MAX_CORRECTION, min(correction, MAX_CORRECTION))
            
            # Apply correction to motor speeds
            target_left = left_pwm
            target_right = right_pwm
            
            # Apply correction
            actual_left = target_left - correction
            actual_right = target_right + correction
                
            # Apply the corrected values
            set_motors(actual_left, actual_right)
            
            # Store current error for next iteration
            last_error = error
        else:
            # Reset integral when stopped or turning
            integral = 0
            last_error = 0
            set_motors(left_pwm, right_pwm)
        
        # Use a smaller delay to make PID more responsive
        time.sleep(0.01)


def stream_camera():
    # Initialize camera
    picam2 = Picamera2()
    camera_config = picam2.create_video_configuration(
        main={"size": (640, 480), "format": "RGB888"},
        transform=Transform(hflip=True, vflip=True)
    )
    picam2.configure(camera_config)
    picam2.start()
    
    # Create socket for streaming
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, IMAGE_PORT))
    server_socket.listen(1)
    
    print(f"Camera stream server started on port {IMAGE_PORT}")
    
    while running:
        try:
            client_socket, addr = server_socket.accept()
            print(f"Camera client connected: {addr}")
            
            while running:
                # Capture frame and convert to bytes
                stream = io.BytesIO()
                picam2.capture_file(stream, format='jpeg')
                stream.seek(0)
                jpeg_data = stream.getvalue()
                jpeg_size = len(jpeg_data)
                
                try:
                    client_socket.sendall(struct.pack("!I", jpeg_size))
                    client_socket.sendall(jpeg_data)
                except:
                    print("Camera client disconnected")
                    break
                
                # Small delay to avoid hogging CPU
                time.sleep(0.01)
                
        except Exception as e:
            print(f"Camera server error: {str(e)}")
        
        if 'client_socket' in locals() and client_socket:
            client_socket.close()
    
    server_socket.close()
    picam2.stop()

# PID configuration server
def pid_config_server():
    global use_PID, KP, KI, KD
    
    # Create socket for receiving PID configuration
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PID_CONFIG_PORT))
    server_socket.listen(1)
    
    print(f"PID config server started on port {PID_CONFIG_PORT}")
    
    while running:
        try:
            client_socket, addr = server_socket.accept()
            print(f"PID config client connected: {addr}")
            
            try:
                # Receive PID constants (4 floats)
                data = client_socket.recv(16)
                if data and len(data) == 16:
                    # Unpack PID values
                    use_PID, KP, KI, KD = struct.unpack("!ffff", data)
                    if use_PID: print("The robot is not using PID.")
                    else: print(f"Updated PID constants: KP={KP}, KI={KI}, KD={KD}")
                    
                    # Send acknowledgment (1 for success)
                    response = struct.pack("!i", 1)
                else:
                    # Send failure response
                    response = struct.pack("!i", 0)
                
                client_socket.sendall(response)
                    
            except Exception as e:
                print(f"PID config error: {str(e)}")
                try:
                    response = struct.pack("!i", 0)
                    client_socket.sendall(response)
                except:
                    pass
                    
            client_socket.close()
                    
        except Exception as e:
            print(f"PID config server error: {str(e)}")
    
    server_socket.close()
    

def command_server():
    global left_pwm, right_pwm, running, left_count, right_count
    
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)
    
    print(f"Command server started on port {PORT}")
    
    while running:
        try:
            client_socket, addr = server_socket.accept()
            print(f"Command client connected: {addr}")
            
            while running:
                try:
                    # Receive command (4 bytes for each PWM value)
                    data = client_socket.recv(8)
                    if not data or len(data) != 8:
                        print("Command client disconnected")
                        break
                    
                    # Unpack speed values and convert to PWM
                    left_speed, right_speed = struct.unpack("!ii", data)
                    print(f"Received command: left_speed={left_speed}, right_speed={right_speed}")
                    left_pwm, right_pwm = left_speed*100, right_speed*100
                    
                    # Send encoder counts back
                    response = struct.pack("!ii", left_count, right_count)
                    client_socket.sendall(response)
                    
                except Exception as e:
                    print(f"Command error: {str(e)}")
                    break
                    
        except Exception as e:
            print(f"Command server error: {str(e)}")
        
        if 'client_socket' in locals() and client_socket:
            client_socket.close()
    
    server_socket.close()


def main():
    try:
        setup_gpio()
        
        # Start PID control thread
        pid_thread = threading.Thread(target=pid_control)
        pid_thread.daemon = True
        pid_thread.start()
        
        # Start camera streaming thread
        camera_thread = threading.Thread(target=stream_camera)
        camera_thread.daemon = True
        camera_thread.start()
        
        # Start PID configuration server thread
        pid_config_thread = threading.Thread(target=pid_config_server)
        pid_config_thread.daemon = True
        pid_config_thread.start()
        
        # Start command server (main thread)
        command_server()
        
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        global running
        running = False
        GPIO.cleanup()
        print("Cleanup complete")


if __name__ == "__main__":
    main()