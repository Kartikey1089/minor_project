import math
import time
from adafruit_pca9685 import PCA9685

# Define link lengths (in mm)
link1 = 137  # Shoulder to Elbow
link2 = 110  # Elbow to Wrist
link3 = 30   # Wrist to Roll
link4 = 30   # Roll to Pitch
link5 = 120  # Pitch to Gripper end

# Servo driver setup (for PCA9685)
pwm = PCA9685()
pwm.frequency = 50

# Servo smooth function
def smooth_servo_move(servo_channel, start_angle, end_angle, duration=1):
    steps = 100
    for i in range(steps + 1):
        angle = start_angle + (end_angle - start_angle) * (i / steps)
        pulse_length = int((angle / 180.0) * (600 - 150) + 150)  # Adjust for your servos' pulse range
        pwm.channels[servo_channel].duty_cycle = pulse_length
        time.sleep(duration / steps)

# Inverse kinematics function for the XZ plane
def calculate_inverse_kinematics(x, z):
    # Calculate the distance from the shoulder to the target point
    target_dist = math.sqrt(x**2 + z**2)
    
    # Ensure the target is reachable
    if target_dist > (link1 + link2):
        raise ValueError("Target is out of reach")

    # Calculate the angle of the shoulder (theta1)
    cos_theta1 = (link1**2 + target_dist**2 - link2**2) / (2 * link1 * target_dist)
    theta1 = math.acos(cos_theta1)  # Shoulder angle (in radians)
    
    # Calculate the wrist angle (theta2)
    cos_theta2 = (link1**2 + link2**2 - target_dist**2) / (2 * link1 * link2)
    theta2 = math.acos(cos_theta2)  # Wrist angle (in radians)

    # Convert angles from radians to degrees
    shoulder_angle = math.degrees(theta1)
    wrist_angle = 180 - math.degrees(theta2)

    # Return the angles
    return shoulder_angle, wrist_angle

# Main control loop
def move_to_xyz(x, z):
    try:
        shoulder_angle, wrist_angle = calculate_inverse_kinematics(x, z)

        # Set limits for shoulder and wrist angles
        shoulder_angle = max(40, min(150, shoulder_angle))
        wrist_angle = max(40, min(150, wrist_angle))

        # Smoothly move the shoulder and wrist to the calculated angles
        smooth_servo_move(0, current_shoulder_angle, shoulder_angle)
        smooth_servo_move(1, current_wrist_angle, wrist_angle)

        # Update current angles
        global current_shoulder_angle, current_wrist_angle
        current_shoulder_angle = shoulder_angle
        current_wrist_angle = wrist_angle

    except ValueError as e:
        print(e)

# Initialize current angles for shoulder and wrist
current_shoulder_angle = 70  # Starting at 70 degrees (perpendicular to ground)
current_wrist_angle = 180    # Starting at 180 degrees (perpendicular to ground)

# Example: Move the gripper to a position (X, Z)
move_to_xyz(150, 100)
