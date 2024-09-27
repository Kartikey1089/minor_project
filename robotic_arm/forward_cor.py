import time
import math
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# Initialize I2C bus and PCA9685 module
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50

# Initialize the six servos on their respective channels
servo_base = servo.Servo(pca.channels[0])
servo_shoulder = servo.Servo(pca.channels[1])
servo_wrist = servo.Servo(pca.channels[2])
servo_gripper_roll = servo.Servo(pca.channels[3])
servo_gripper_pitch = servo.Servo(pca.channels[4])
servo_gripper_open_close = servo.Servo(pca.channels[5])

# Link lengths in mm (as given earlier)
link1 = 40    # Base to Shoulder
link2 = 137   # Shoulder to Elbow
link3 = 110   # Elbow to Wrist
link4 = 30    # Wrist to Roll
link5 = 120   # Roll to Gripper Tip

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

# Function to calculate forward kinematics and determine the (X, Y, Z) position
def calculate_forward_kinematics(base_angle, shoulder_angle, wrist_angle, roll_angle, pitch_angle):
    # Convert degrees to radians for calculation
    base_rad = math.radians(base_angle)
    shoulder_rad = math.radians(shoulder_angle)
    wrist_rad = math.radians(wrist_angle)
    pitch_rad = math.radians(pitch_angle)

    # Compute the position using trigonometry
    x = (link2 * math.cos(shoulder_rad) + link3 * math.cos(shoulder_rad + wrist_rad)) * math.cos(base_rad)
    y = (link2 * math.cos(shoulder_rad) + link3 * math.cos(shoulder_rad + wrist_rad)) * math.sin(base_rad)
    z = link1 + link2 * math.sin(shoulder_rad) + link3 * math.sin(shoulder_rad + wrist_rad) + link4 + (link5 * math.sin(pitch_rad))

    return round(x, 2), round(y, 2), round(z, 2)

# Function to move the arm to a specific position using smooth function
def move_arm_to_position(angles):
    move_servo_smooth(servo_base, angles[0])
    move_servo_smooth(servo_shoulder, angles[1])
    move_servo_smooth(servo_wrist, angles[2])
    move_servo_smooth(servo_gripper_roll, angles[3])
    move_servo_smooth(servo_gripper_pitch, angles[4])
    move_servo_smooth(servo_gripper_open_close, angles[5])
    
    # Calculate and print the (X, Y, Z) coordinates
    x, y, z = calculate_forward_kinematics(angles[0], angles[1], angles[2], angles[3], angles[4])
    print(f"Gripper's Position: X = {x} mm, Y = {y} mm, Z = {z} mm")
    time.sleep(1)  # Small delay after movement

# Define the angle arrays for different positions
# Angles array: [base, shoulder, wrist, roll, pitch, gripper]
home_position_angles = [90, 90, 90, 90, 90, 0]  # Adjust as needed for your home position
pickup_position_angles = [90, 45, 30, 90, 60, 0]  # Adjust these angles for the object pickup
return_position_angles = [90, 90, 90, 90, 90, 0]  # Return to the home position

# Function to perform the entire task sequence
def perform_pick_and_place():
    try:
        print("Moving to home position...")
        move_arm_to_position(home_position_angles)

        time.sleep(2)

        print("Moving to the pickup position...")
        move_arm_to_position(pickup_position_angles)

        # Close the gripper to pick up the object
        print("Closing gripper to pick up object...")
        move_servo_smooth(servo_gripper_open_close, 90)  # Adjust this angle to close the gripper properly

        time.sleep(2)

        print("Returning to home position with the object...")
        move_arm_to_position(return_position_angles)
    
    except KeyboardInterrupt:
        print("\nProcess interrupted by the user.")
        move_arm_to_position(home_position_angles)  # Return to home position if interrupted
    except Exception as e:
        print(f"An error occurred: {e}")
        move_arm_to_position(home_position_angles)  # Return to home position in case of an error
    finally:
        pca.deinit()  # Safely deinitialize the PCA9685 module

# Main execution
if __name__ == "__main__":
    perform_pick_and_place()
    print("Pick and place sequence complete.")
