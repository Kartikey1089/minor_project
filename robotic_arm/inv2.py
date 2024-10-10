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

# Smooth movement function
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

# Inverse kinematics calculation for base, shoulder, and wrist
def calculate_inverse_kinematics(x, y, z):
    if z < 50:
        raise ValueError("Gripper height cannot be below 50 mm.")

    # Base motor angle (left and right movement, Y-axis)
    base_angle = math.degrees(math.atan2(y, x))

    # Distance from base to the end effector projection on the ground plane (XZ-plane)
    distance_xy = math.sqrt(x**2 + y**2)
    
    # Lengths of the links involved (using the measurements you provided)
    l1 = 137  # Shoulder to elbow
    l2 = 110  # Elbow to wrist
    l3 = 150  # Wrist to gripper end (including roll and pitch links)
    
    # Inverse kinematics to calculate shoulder and wrist angles
    shoulder_angle = math.degrees(math.atan2(z, distance_xy))
    shoulder_wrist_distance = math.sqrt(distance_xy**2 + z**2)

    # Applying cosine law to calculate joint angles
    cos_wrist = (l1**2 + l2**2 - shoulder_wrist_distance**2) / (2 * l1 * l2)
    wrist_angle = math.degrees(math.acos(cos_wrist))  # Elbow joint angle

    # Adjust angles to account for physical limits
    shoulder_angle = min(max(40, shoulder_angle), 150)
    wrist_angle = min(max(40, wrist_angle), 150)
    
    return base_angle, shoulder_angle, wrist_angle

# Function to move the arm to calculated positions
def move_arm_to_position(x, y, z):
    try:
        base_angle, shoulder_angle, wrist_angle = calculate_inverse_kinematics(x, y, z)
        
        print(f"Moving to X: {x} mm, Y: {y} mm, Z: {z} mm")
        print(f"Base angle: {base_angle}, Shoulder angle: {shoulder_angle}, Wrist angle: {wrist_angle}")

        # Move the servos with smooth function
        move_servo_smooth(servo_base, base_angle)
        move_servo_smooth(servo_shoulder, shoulder_angle)
        move_servo_smooth(servo_wrist, wrist_angle)

        # Pitch and roll will remain static for now, you can adjust them based on your needs
        move_servo_smooth(servo_roll, 90)  # Example default
        move_servo_smooth(servo_pitch, 90)  # Example default
        
    except ValueError as e:
        print(e)

# Example of calling the inverse function
if __name__ == "__main__":
    # Example target coordinates (X, Y, Z)
    move_arm_to_position(150, 100, 70)
    time.sleep(2)
    move_arm_to_position(200, 50, 80)
