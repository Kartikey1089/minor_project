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
servo_roll = servo.Servo(pca.channels[3])  # Roll servo
servo_pitch = servo.Servo(pca.channels[4])  # Pitch servo

# Link lengths (in mm)
link_roll_to_pitch = 30  # Roll to pitch link length
link_pitch_to_gripper = 120  # Pitch to gripper end link length

# Smooth movement function
def move_servo_smooth(servo_motor, target_angle, step_delay=0.02):
    current_angle = servo_motor.angle
    if current_angle is None:
        current_angle = 90  # Default angle
    step_size = 1 if target_angle > current_angle else -1
    
    while abs(target_angle - current_angle) > abs(step_size):
        current_angle += step_size
        servo_motor.angle = current_angle
        time.sleep(step_delay)
    
    servo_motor.angle = target_angle

# Function to calculate pitch and roll angles to reach the desired z-coordinate
def calculate_pitch_roll_for_height(z):
    # Total length from roll to gripper end
    total_length = link_roll_to_pitch + link_pitch_to_gripper

    # z should be less than or equal to the total length
    if z > total_length:
        print("Error: The z-coordinate exceeds the maximum reach of the arm.")
        return None, None

    # Calculate pitch using inverse kinematics (in degrees)
    theta_pitch = math.acos(z / total_length) * 180 / math.pi

    # Assuming roll remains constant (you can adjust this if needed)
    theta_roll = 90  # Default roll angle

    return theta_roll, theta_pitch

# Main function to move the gripper to a given height
def move_gripper_to_height(z):
    theta_roll, theta_pitch = calculate_pitch_roll_for_height(z)

    if theta_roll is None or theta_pitch is None:
        return

    print(f"Moving to height: {z} mm")
    print(f"Calculated roll angle: {theta_roll} degrees")
    print(f"Calculated pitch angle: {theta_pitch} degrees")

    # Move servos smoothly to the calculated angles
    move_servo_smooth(servo_roll, theta_roll)
    move_servo_smooth(servo_pitch, theta_pitch)

# Main loop
try:
    z_height = float(input("Enter the desired height (z-coordinate) in mm: "))
    move_gripper_to_height(z_height)
except KeyboardInterrupt:
    print("Program interrupted.")
except Exception as e:
    print(f"Error: {e}")
finally:
    pca.deinit()  # Safely deinitialize the PCA9685 module
