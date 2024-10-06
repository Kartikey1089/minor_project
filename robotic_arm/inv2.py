import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import math

# Initialize I2C bus and PCA9685 module
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50

# Initialize the servos
servo_roll = servo.Servo(pca.channels[4])
servo_pitch = servo.Servo(pca.channels[6])

# Link lengths in mm
L4 = 30  # roll to pitch
L5 = 120  # pitch to gripper end

# Function to smoothly move the servos
def move_servo_smooth(servo_motor, target_angle, step_delay=0.02):
    current_angle = servo_motor.angle
    if current_angle is None:
        current_angle = 90  # Default to 90 degrees if undefined

    step_size = 1 if target_angle > current_angle else -1
    
    while abs(target_angle - current_angle) > abs(step_size):
        current_angle += step_size
        servo_motor.angle = current_angle
        time.sleep(step_delay)
    
    servo_motor.angle = target_angle

# Calculate the pitch angle for the desired z-coordinate (height)
def calculate_pitch_angle(z):
    # Ensure the requested z value is achievable within the limits
    if z > (L4 + L5):
        print(f"z is too high, max achievable is {L4 + L5} mm.")
        return None
    if z < L4:
        print(f"z is too low, min achievable is {L4} mm.")
        return None
    
    # Calculate the pitch angle using inverse kinematics
    pitch_angle = math.degrees(math.acos((z - L4) / L5))
    
    return pitch_angle

# Function to move the arm to a desired z height
def move_gripper_to_height(z):
    pitch_angle = calculate_pitch_angle(z)
    
    if pitch_angle is not None:
        print(f"Moving to height: {z} mm, calculated pitch angle: {pitch_angle:.2f} degrees")
        move_servo_smooth(servo_pitch, pitch_angle)
        time.sleep(2)
    else:
        print("Cannot move to the specified height.")

# Main execution
if __name__ == "__main__":
    try:
        # Example: Move the gripper to a height of 50 mm
        move_gripper_to_height(50)
        
    except KeyboardInterrupt:
        print("\nProcess interrupted.")
    finally:
        pca.deinit()
