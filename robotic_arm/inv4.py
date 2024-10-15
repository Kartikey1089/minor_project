import time
import numpy as np
from pca9685 import PCA9685  # Ensure you have this library installed

# Define link lengths (in mm)
L1 = 40    # Base to Shoulder
L2 = 140   # Shoulder to Elbow
L3 = 100   # Elbow to Wrist
L4 = 140   # Wrist to End of Gripper

# Initialize PCA9685 driver
pwm = PCA9685()
pwm.set_pwm_freq(60)  # Set frequency to 60 Hz

def set_servo_angle(channel, angle):
    pulse_length = 4096  # Full scale for 12-bit resolution
    pulse_width = (angle / 180.0) * pulse_length
    pwm.set_pwm(channel, 0, int(pulse_width))

def inverse_kinematics(x, y, z):
    # Calculate the base joint angle (theta_base)
    theta_base = np.arctan2(y, x)

    # Calculate the projection to the XY plane
    r = np.sqrt(x**2 + y**2)

    # Calculate the distance d (hypotenuse in the triangle formed)
    d = np.sqrt(r**2 + (z - L1)**2)

    # Calculate shoulder joint angle (theta_shoulder)
    A = (L1**2 + L2**2 - d**2) / (2 * L1 * L2)
    theta_shoulder = np.arccos(np.clip(A, -1.0, 1.0))  # Clip value to avoid out of bounds

    # Calculate elbow joint angle (theta_elbow)
    B = (L1**2 + d**2 - L2**2) / (2 * L1 * d)
    theta_elbow = np.arccos(np.clip(B, -1.0, 1.0))  # Clip value to avoid out of bounds

    # Convert radians to degrees
    return np.degrees(theta_base), np.degrees(theta_shoulder), np.degrees(theta_elbow)

def move_to_coordinates(x, y, z):
    # Calculate joint angles for the desired coordinates
    theta_base, theta_shoulder, theta_elbow = inverse_kinematics(x, y, z)

    # Send angles to the servo motors
    set_servo_angle(0, theta_base)       # Base motor
    set_servo_angle(1, theta_shoulder)    # Shoulder motor
    set_servo_angle(2, theta_elbow)       # Elbow motor
    # Add code for wrist and gripper if needed
    # Example: set_servo_angle(3, roll_angle)
    # Example: set_servo_angle(4, pitch_angle)
    # Example: set_servo_angle(5, gripper_angle)

# Example usage
if __name__ == "__main__":
    try:
        # Move the robotic arm to specified coordinates
        target_x = 100  # Replace with your target X coordinate
        target_y = 0    # Replace with your target Y coordinate
        target_z = 100  # Replace with your target Z coordinate

        move_to_coordinates(target_x, target_y, target_z)

        time.sleep(2)  # Wait for 2 seconds to observe the position

    except KeyboardInterrupt:
        print("Program interrupted.")
    finally:
        pwm.set_pwm(0, 0, 0)  # Turn off all servos
