import math
import time
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685

# I2C bus setup for the PCA9685
i2c_bus = busio.I2C(SCL, SDA)
pwm = PCA9685(i2c_bus)
pwm.frequency = 50

# Define link lengths (in mm)
link1 = 137  # Shoulder to Elbow
link2 = 110  # Elbow to Wrist
link3 = 30   # Wrist to Roll
link4 = 30   # Roll to Pitch
link5 = 120  # Pitch to Gripper end

# Servo smooth movement function
def smooth_servo_move(servo_channel, start_angle, end_angle, duration=1):
    steps = 100  # Number of steps for smooth motion
    for i in range(steps + 1):
        angle = start_angle + (end_angle - start_angle) * (i / steps)
        pulse_length = int((angle / 180.0) * (600 - 150) + 150)  # Adjust based on your servos' pulse range
        pwm.channels[servo_channel].duty_cycle = pulse_length
        time.sleep(duration / steps)

# Inverse kinematics calculation for the XZ plane
def calculate_inverse_kinematics(x, z):
    # Calculate the distance from shoulder to target point
    target_dist = math.sqrt(x**2 + z**2)

    # Ensure the target is reachable
    if target_dist > (link1 + link2):
        raise ValueError("Target position out of reach.")

    # Law of cosines for shoulder and wrist angles
    cos_angle_wrist = (link1**2 + link2**2 - target_dist**2) / (2 * link1 * link2)
    wrist_angle = math.acos(cos_angle_wrist) * 180 / math.pi

    cos_angle_shoulder = (target_dist**2 + link1**2 - link2**2) / (2 * target_dist * link1)
    shoulder_angle = math.acos(cos_angle_shoulder) * 180 / math.pi

    # Adjust shoulder angle based on target height (z)
    shoulder_angle = math.degrees(math.atan2(z, x)) - shoulder_angle

    return shoulder_angle, wrist_angle

# Update current angles
current_shoulder_angle = 90
current_wrist_angle = 90

# Move arm function
def move_arm_to_position(x, z):
    global current_shoulder_angle, current_wrist_angle
    
    try:
        shoulder_angle, wrist_angle = calculate_inverse_kinematics(x, z)

        # Check angle limits
        if shoulder_angle < 40 or shoulder_angle > 150:
            raise ValueError("Shoulder angle out of bounds (40-150 degrees).")
        if wrist_angle < 40 or wrist_angle > 150:
            raise ValueError("Wrist angle out of bounds (40-150 degrees).")
        if z < 50:
            raise ValueError("Z coordinate too low (minimum 50mm height).")

        # Smoothly move servos
        smooth_servo_move(0, current_shoulder_angle, shoulder_angle)
        smooth_servo_move(1, current_wrist_angle, wrist_angle)

        # Update current angles
        current_shoulder_angle = shoulder_angle
        current_wrist_angle = wrist_angle

    except ValueError as e:
        print(f"Error: {e}")

# Main code to input target position
if __name__ == "__main__":
    while True:
        x = float(input("Enter target X coordinate: "))
        z = float(input("Enter target Z coordinate: "))
        move_arm_to_position(x, z)
