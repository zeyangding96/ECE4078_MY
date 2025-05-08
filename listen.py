import socket
import struct
import io
import threading
import time
import RPi.GPIO as GPIO
import numpy as np
from picamera2 import Picamera2
from simple_pid import PID

# Network Configuration
HOST = '0.0.0.0'
WHEEL_PORT = 8000
CAMERA_PORT = 8001
PID_CONFIG_PORT = 8002

# Pins
LEFT_MOTOR_ENA = 18
LEFT_MOTOR_IN1 = 17
LEFT_MOTOR_IN2 = 27
RIGHT_MOTOR_ENB = 25
RIGHT_MOTOR_IN3 = 23
RIGHT_MOTOR_IN4 = 24
LEFT_ENCODER = 26
RIGHT_ENCODER = 16

# PID Constants (default values, will be overridden by client)
use_PID = 0
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
    
    # This prevents slight motor jerk when connection is established
    GPIO.output(LEFT_MOTOR_ENA, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_ENB, GPIO.LOW)
    
    # Encoder setup and interrupt (both activated and deactivated)
    GPIO.setup(LEFT_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(RIGHT_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(LEFT_ENCODER, GPIO.BOTH, callback=left_encoder_callback)
    GPIO.add_event_detect(RIGHT_ENCODER, GPIO.BOTH, callback=right_encoder_callback)
    
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
    
def reset_encoder():
    global left_count, right_count
    left_count, right_count = 0, 0

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
    # Only applies for forward/backward, not turning
    global left_pwm, right_pwm, left_count, right_count, use_PID, KP, KI, KD
    
    # integral = 0
    # last_error = 0
    # last_time = time.time()
    flag_new_pid_cycle = True
    while running:
        # Calculate time delta
        # current_time = time.time()
        # dt = current_time - last_time
        # last_time = current_time
        
        if not use_PID:
            set_motors(left_pwm, right_pwm)
        else:
            if (left_pwm > 0 and right_pwm > 0) or (left_pwm < 0 and right_pwm < 0):
                ### Calculate error (difference between encoder counts)
                # error = left_count - right_count
                
                ## Calculate PID terms
                # proportional = KP * error
                # integral += KI * error * dt
                # integral = max(-MAX_CORRECTION, min(integral, MAX_CORRECTION))  # Anti-windup
                # derivative = KD * (error - last_error) / dt if dt > 0 else 0
                
                ## Calculate correction
                # correction = proportional + integral + derivative
                # correction = max(-MAX_CORRECTION, min(correction, MAX_CORRECTION))
                            
                ## Apply correction
                # print('correct', correction)
                # actual_left = left_pwm - correction
                # actual_right = right_pwm + correction
                # print('actual', actual_left, actual_right)
                    
                ## Apply the corrected values
                # set_motors(actual_left, actual_right)
                # last_error = error
                
                left, right = abs(left_pwm), abs(right_pwm)
                if flag_new_pid_cycle:
                    pid_right = PID(KP, KI, KD, setpoint=left_count, output_limits=(0,1), starting_output=right/100)
                    flag_new_pid_cycle = False
                pid_right.setpoint = left_count
                right = pid_right(left_count)*100
                print(left, right)
                if (left_pwm > 0 and right_pwm > 0): set_motors(left, right)
                else: set_motors(-left, -right)
                
            else:
                # Reset integral when stopped or turning
                # integral = 0
                # last_error = 0
                reset_encoder()
                set_motors(left_pwm, right_pwm)
                flag_new_pid_cycle = True
        
        # Use a smaller delay to make PID more responsive
        time.sleep(0.01)


def camera_stream_server():
    # Initialize camera
    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(lores={"size": (640,480)})
    picam2.configure(camera_config)
    picam2.start()
    
    # Create socket for streaming
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, CAMERA_PORT))
    server_socket.listen(1)
    
    print(f"Camera stream server started on port {CAMERA_PORT}")
    
    while running:
        try:
            client_socket, _ = server_socket.accept()
            print(f"Camera stream client connected")
            
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
                    print("Camera stream client disconnected")
                    break
                
                # Small delay to avoid hogging CPU
                time.sleep(0.01)
                
        except Exception as e:
            print(f"Camera stream server error: {str(e)}")
        
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
            client_socket, _ = server_socket.accept()
            print(f"PID config client connected")
            
            try:
                # Receive PID constants (4 floats)
                data = client_socket.recv(16)
                if data and len(data) == 16:
                    use_PID, KP, KI, KD = struct.unpack("!ffff", data)
                    if use_PID: print(f"Updated PID constants: KP={KP}, KI={KI}, KD={KD}")
                    else: print("The robot is not using PID.")
                    
                    # Send acknowledgment (1 for success)
                    response = struct.pack("!i", 1)
                else:
                    # Send failure response
                    response = struct.pack("!i", 0)
                
                client_socket.sendall(response)
                    
            except Exception as e:
                print(f"PID config socket error: {str(e)}")
                try:
                    response = struct.pack("!i", 0)
                    client_socket.sendall(response)
                except:
                    pass
                    
            client_socket.close()
                    
        except Exception as e:
            print(f"PID config server error: {str(e)}")
    
    server_socket.close()
    

def wheel_server():
    global left_pwm, right_pwm, running, left_count, right_count
    
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, WHEEL_PORT))
    server_socket.listen(1)
    
    print(f"Wheel server started on port {WHEEL_PORT}")
    
    while running:
        try:
            client_socket, _ = server_socket.accept()
            print(f"Wheel client connected")
            
            while running:
                try:
                    # Receive speed (4 bytes for each value)
                    data = client_socket.recv(8)
                    if not data or len(data) != 8:
                        print("Wheel client sending speed error")
                        break
                    
                    # Unpack speed values and convert to PWM
                    left_speed, right_speed = struct.unpack("!ff", data)
                    print(f"Received wheel: left_speed={left_speed:.4f}, right_speed={right_speed:.4f}")
                    left_pwm, right_pwm = left_speed*100, right_speed*100
                    
                    # Send encoder counts back
                    response = struct.pack("!ii", left_count, right_count)
                    client_socket.sendall(response)
                    
                except Exception as e:
                    print(f"Wheel client disconnected")
                    break
                    
        except Exception as e:
            print(f"Wheel server error: {str(e)}")
        
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
        camera_thread = threading.Thread(target=camera_stream_server)
        camera_thread.daemon = True
        camera_thread.start()
        
        # Start PID configuration server thread
        pid_config_thread = threading.Thread(target=pid_config_server)
        pid_config_thread.daemon = True
        pid_config_thread.start()
        
        # Start wheel server (main thread)
        wheel_server()
        
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        global running
        running = False
        GPIO.cleanup()
        print("Cleanup complete")


if __name__ == "__main__":
    main()