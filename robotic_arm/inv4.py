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

def set_servo_angle(channel, angle):
    pulse_length = 4096  # Full scale for 12-bit resolution
    pulse_width = (angle / 180.0) * pulse_length
    pwm.set_pwm(channel, 0, int(pulse_width))

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

    # Send angles to the servo motors
    set_servo_angle(0, thetal)    # Base motor
    set_servo_angle(1, theta2)    # Shoulder motor
    set_servo_angle(2, theta3)    # Elbow motor
    # Add code for wrist and gripper if needed
    # Example: set_servo_angle(3, roll_angle)
    # Example: set_servo_angle(4, pitch_angle)
    # Example: set_servo_angle(5, gripper_angle)

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
