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
servo_base = servo.Servo(pca.channels[0])
servo_shoulder = servo.Servo(pca.channels[1])
servo_wrist = servo.Servo(pca.channels[2])
servo_roll = servo.Servo(pca.channels[3])
servo_pitch = servo.Servo(pca.channels[4])
servo_gripper = servo.Servo(pca.channels[5])

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

# Inverse kinematics function
def calculate_angles(x, y, z):
    # Physical link lengths based on user inputs
    link_base_to_shoulder = 40
    link_shoulder_to_wrist = 137
    link_wrist_to_gripper = 110
    link_roll_to_pitch = 30
    link_pitch_to_gripper_end = 120

    # Ensure the gripper stays above 50 mm from the ground
    if z < 50:
        z = 50

    # Calculate base angle (rotation around Z-axis)
    base_angle = math.degrees(math.atan2(y, x))

    # Calculate shoulder and wrist angles based on 2D projection on X-Z plane
    distance_shoulder_to_gripper = math.sqrt(x**2 + z**2)
    if distance_shoulder_to_gripper > (link_shoulder_to_wrist + link_wrist_to_gripper):
        distance_shoulder_to_gripper = link_shoulder_to_wrist + link_wrist_to_gripper

    shoulder_angle = math.degrees(math.atan2(z, x))
    wrist_angle = 90 - shoulder_angle

    # Limit the shoulder and wrist angles between 40° and 150°
    shoulder_angle = min(max(shoulder_angle, 40), 150)
    wrist_angle = min(max(wrist_angle, 40), 150)

    # Gripper pitch and roll, assuming no rotation for now
    gripper_pitch_angle = 90  # Assuming straight orientation
    gripper_roll_angle = 150  # Based on your shared measurements

    return base_angle, shoulder_angle, wrist_angle, gripper_roll_angle, gripper_pitch_angle

# Function to move the arm based on calculated angles
def move_arm_to_position(x, y, z):
    base_angle, shoulder_angle, wrist_angle, roll_angle, pitch_angle = calculate_angles(x, y, z)

    # Move servos to calculated positions
    move_servo_smooth(servo_base, base_angle)
    move_servo_smooth(servo_shoulder, shoulder_angle)
    move_servo_smooth(servo_wrist, wrist_angle)
    move_servo_smooth(servo_roll, roll_angle)
    move_servo_smooth(servo_pitch, pitch_angle)

# Main execution function
def perform_inverse_kinematics(x, y, z):
    try:
        print(f"Moving to position X: {x}, Y: {y}, Z: {z}")
        move_arm_to_position(x, y, z)
        print("Movement complete.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        pca.deinit()

# Example usage
if __name__ == "__main__":
    perform_inverse_kinematics(140, 100, 70)  # Example coordinates
