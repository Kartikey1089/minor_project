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

# Initialize the six servos on their respective channels
servo_base = servo.Servo(pca.channels[0])
servo_shoulder = servo.Servo(pca.channels[1])
servo_wrist = servo.Servo(pca.channels[2])
servo_gripper_roll = servo.Servo(pca.channels[3])
servo_gripper_pitch = servo.Servo(pca.channels[4])
servo_gripper_open_close = servo.Servo(pca.channels[5])

# Arm link lengths in mm (as per your arm)
link1 = 40   # Base to Shoulder
link2 = 137  # Shoulder to Elbow
link3 = 110  # Elbow to Wrist
link4 = 30   # Wrist to Roll
link5 = 120  # Roll to Gripper Tip

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

# Function to move the arm to a specific position using smooth function
def move_arm_to_position(angles):
    move_servo_smooth(servo_base, angles[0])
    move_servo_smooth(servo_shoulder, angles[1])
    move_servo_smooth(servo_wrist, angles[2])
    move_servo_smooth(servo_gripper_roll, angles[3])
    move_servo_smooth(servo_gripper_pitch, angles[4])
    move_servo_smooth(servo_gripper_open_close, angles[5])
    time.sleep(1)  # Small delay after movement

# Define the angle arrays for the home position
home_position_angles = [90, 90, 90, 90, 90, 0]  # Adjust as needed for your home position

# Function to calculate inverse kinematics for the robotic arm
def calculate_inverse_kinematics(x, y, z):
    # Calculate the planar distance from the base to the target point
    d = math.sqrt(x**2 + y**2)
    
    # Calculate the wrist position (subtracting link5 because it's the gripper length)
    d_wrist = d - link5
    z_wrist = z - link1
    
    # Use inverse kinematics to calculate shoulder and elbow angles
    if d_wrist <= 0 or z_wrist <= 0:
        raise ValueError("Target out of reach or not physically possible for the arm.")
    
    L = math.sqrt(d_wrist**2 + z_wrist**2)
    
    cos_angle1 = (link2**2 + L**2 - link3**2) / (2 * link2 * L)
    cos_angle2 = (link2**2 + link3**2 - L**2) / (2 * link2 * link3)
    
    if abs(cos_angle1) > 1 or abs(cos_angle2) > 1:
        raise ValueError("Target out of reach or not physically possible for the arm.")
    
    angle1 = math.degrees(math.acos(cos_angle1))
    angle2 = math.degrees(math.acos(cos_angle2))
    
    shoulder_angle = 90 - angle1  # Adjusting relative to the arm's natural position
    wrist_angle = 180 - angle2    # Wrist angle relative to the elbow position

    # Calculate base rotation angle
    base_angle = math.degrees(math.atan2(y, x))

    # Roll and pitch might be predefined based on the orientation of your object
    gripper_roll_angle = 90  # Adjust as necessary
    gripper_pitch_angle = 90  # Adjust as necessary

    # Open/Close gripper (for now we'll keep it open)
    gripper_open_close_angle = 0

    return [base_angle, shoulder_angle, wrist_angle, gripper_roll_angle, gripper_pitch_angle, gripper_open_close_angle]

# Function to perform inverse kinematics movement
def perform_inverse_kinematics(x, y, z):
    try:
        print("Moving to home position...")
        move_arm_to_position(home_position_angles)

        time.sleep(2)

        print(f"Calculating angles for target position (X={x}, Y={y}, Z={z})...")
        target_angles = calculate_inverse_kinematics(x, y, z)

        print(f"Moving to target position with calculated angles: {target_angles}")
        move_arm_to_position(target_angles)

        time.sleep(2)

        print("Returning to home position...")
        move_arm_to_position(home_position_angles)

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
    # Example coordinates; you can change these to test with other coordinates
    target_x = 100  # X-coordinate in mm
    target_y = 50   # Y-coordinate in mm
    target_z = 100  # Z-coordinate in mm

    perform_inverse_kinematics(target_x, target_y, target_z)
    print("Inverse kinematics sequence complete.")
