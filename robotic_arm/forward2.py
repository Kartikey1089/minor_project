import time
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
