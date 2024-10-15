import time
import numpy as np
from pca9685 import PCA9685  # Ensure you have this library installed

# Define link lengths (in mm)
L1 = 120  # Base to Shoulder
L2 = 120  # Shoulder to Elbow
L3 = 35   # Elbow to Wrist

# Initialize PCA9685 driver
pwm = PCA9685()
pwm.set_pwm_freq(60)  # Set frequency to 60 Hz

# Servo angle limits
ANGLE_LIMITS = {
    'base': (0, 180),     # Base motor
    'shoulder': (40, 150), # Shoulder motor
    'wrist': (40, 150)    # Wrist motor
}

def set_servo_angle(channel, angle):
    # Limit the angle within bounds
    if angle < ANGLE_LIMITS['shoulder'][0]:
        angle = ANGLE_LIMITS['shoulder'][0]
    elif angle > ANGLE_LIMITS['shoulder'][1]:
        angle = ANGLE_LIMITS['shoulder'][1]
    
    # Map angle to PWM pulse width
    pulse_length = 4096  # Full scale for 12-bit resolution
    pulse_width = int((angle / 180.0) * pulse_length)
    pwm.set_pwm(channel, 0, pulse_width)

def smooth_movement(current_angle, target_angle, speed=1):
    # Function to move the servos smoothly from current_angle to target_angle
    step = speed if target_angle > current_angle else -speed
    for angle in range(int(current_angle), int(target_angle), step):
        yield angle
        time.sleep(0.01)  # Small delay for smooth transition

def inverse_kinematics(x, y, z):
    # Calculate the base joint angle (thetal)
    thetal = np.arctan2(y, x)

    # Calculate the distance r from the base to the projection of point P on the XY plane
    r = np.sqrt(x**2 + y**2)

    # Calculate the distance d (hypotenuse in the triangle formed)
    d = np.sqrt(r**2 + (z - L1)**2)

    # Calculate theta2 (shoulder joint angle)
    A = (L1**2 + L2**2 - d**2) / (2 * L1 * L2)
    theta2 = np.arccos(A)

    # Calculate theta3 (elbow joint angle)
    B = (L1**2 + d**2 - L2**2) / (2 * L1 * d)
    theta3 = np.arccos(B)

    # Calculate the angle for the wrist (theta)
    thetae = np.arctan2(z - L1, r)

    return np.degrees(thetae), np.degrees(thetal), np.degrees(theta2), np.degrees(theta3)

def move_to_coordinates(x, y, z):
    # Calculate joint angles for the desired coordinates
    thetae, thetal, theta2, theta3 = inverse_kinematics(x, y, z)

    # Get current angles for smooth movement (this would require tracking current angles)
    current_base_angle = 90  # Set a default or track from actual movement
    current_shoulder_angle = 90  # Default or actual
    current_wrist_angle = 90  # Default or actual

    # Move base motor smoothly
    for angle in smooth_movement(current_base_angle, thetal):
        set_servo_angle(0, angle)

    # Move shoulder motor smoothly
    for angle in smooth_movement(current_shoulder_angle, theta2):
        set_servo_angle(1, angle)

    # Move wrist motor smoothly
    for angle in smooth_movement(current_wrist_angle, theta3):
        set_servo_angle(2, angle)

# Example usage
if __name__ == "__main__":
    try:
        # Move the robotic arm to specified coordinates
        target_x = 140  # Replace with your target X coordinate
        target_y = 100  # Replace with your target Y coordinate
        target_z = 70   # Replace with your target Z coordinate

        move_to_coordinates(target_x, target_y, target_z)

        time.sleep(2)  # Wait for 2 seconds to observe the position

    except KeyboardInterrupt:
        print("Program interrupted.")
    finally:
        pwm.set_pwm(0, 0, 0)  # Turn off all servos
