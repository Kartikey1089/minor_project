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

# Initialize the roll, pitch, and gripper servos on their respective channels
servo_gripper_roll = servo.Servo(pca.channels[3])
servo_gripper_pitch = servo.Servo(pca.channels[4])

# Link lengths in mm (based on your measurements)
link_4_length = 30   # From wrist to roll
link_5_length = 120  # From roll to gripper tip

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

# Function to calculate the position of the gripper tip using roll and pitch
def calculate_gripper_position(pitch_angle, roll_angle):
    # Convert angles to radians for trigonometric functions
    pitch_rad = math.radians(pitch_angle)
    
    # Since roll is aligned vertically, we focus on Z and Y movement
    # Calculate vertical (Z-axis) and horizontal (Y-axis) displacement using pitch angle
    z_gripper = link_4_length + link_5_length * math.cos(pitch_rad)  # Vertical position
    y_gripper = link_5_length * math.sin(pitch_rad)  # Horizontal position

    return z_gripper, y_gripper

# Function to move the roll and pitch servos with smooth movement
def move_roll_and_pitch(roll_angle, pitch_angle):
    move_servo_smooth(servo_gripper_roll, roll_angle)
    move_servo_smooth(servo_gripper_pitch, pitch_angle)
    
    # Calculate the position after movement
    z_gripper, y_gripper = calculate_gripper_position(pitch_angle, roll_angle)
    
    print(f"Gripper Position (Roll-Pitch): Z = {z_gripper:.2f} mm, Y = {y_gripper:.2f} mm")

# Example sequence where the roll is aligned vertically and pitch moves
def perform_gripper_move_sequence():
    try:
        # Initial roll is aligned vertically at 90 degrees
        roll_angle = 90

        # Example pitch movement
        pitch_angle_start = 0  # Start at 0 degrees (gripper straight up)
        pitch_angle_end = 90   # End at 90 degrees (gripper fully extended)

        # Move the roll and pitch smoothly
        print("Moving roll and pitch servos...")
        move_roll_and_pitch(roll_angle, pitch_angle_start)
        time.sleep(1)  # Allow some time before moving

        # Now extend the gripper by changing the pitch
        move_roll_and_pitch(roll_angle, pitch_angle_end)
        time.sleep(1)

    except KeyboardInterrupt:
        print("\nProcess interrupted by the user.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        pca.deinit()  # Safely deinitialize the PCA9685 module

# Main execution
if __name__ == "__main__":
    perform_gripper_move_sequence()
    print("Gripper movement sequence complete.")
