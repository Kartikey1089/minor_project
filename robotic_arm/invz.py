import time
import board
import busio
import numpy as np
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# Initialize I2C bus and PCA9685 module
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50

# Initialize the pitch servo on its respective channel
servo_pitch = servo.Servo(pca.channels[6])  # Adjust the channel number if necessary

# Define the pitch angles and corresponding heights
angles = np.array([0, 25, 50, 100])
heights = np.array([105, 60, 25, 0])

# Interpolation function to get pitch angle for a given z height
def get_pitch_angle(z):
    if z > 105 or z < 0:
        print("Z value out of range")
        return None
    return np.interp(z, heights[::-1], angles[::-1])

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

# Main function to set the desired height
def set_height(z):
    pitch_angle = get_pitch_angle(z)
    
    if pitch_angle is not None:
        print(f"Setting pitch angle to: {pitch_angle:.2f} degrees for z = {z} mm")
        move_servo_smooth(servo_pitch, pitch_angle)
        time.sleep(1)  # Wait for the servo to reach the position
    else:
        print("Could not set the pitch angle due to out of range value.")

# Example usage
if __name__ == "__main__":
    desired_z = 50  # Change this value to the desired height
    set_height(desired_z)

    # Clean up
    pca.deinit()
