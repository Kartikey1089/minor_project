from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import board
import busio
import time

# Initialize I2C and PCA9685
i2c = busio.I2C(board.SCL, board.SDA)
pca9685 = PCA9685(i2c)
pca9685.frequency = 50

# Create servo objects for each channel
servos = [servo.Servo(pca9685.channels[i]) for i in range(6)]

def set_servo_angle_smooth(servo_index, target_angle, step_delay=0.02):
    """Move the servo to the target angle smoothly."""
    current_angle = servos[servo_index].angle or 0  # Get current angle or assume 0
    step = 1 if target_angle > current_angle else -1  # Determine step direction

    for angle in range(int(current_angle), int(target_angle), step):
        servos[servo_index].angle = angle
        time.sleep(step_delay)  # Delay between each step for smooth movement
    
    # Set the final target angle (to make sure it exactly reaches the target)
    servos[servo_index].angle = target_angle

def main():
    while True:
        try:
            # Ask for angles for each servo
            base_angle = float(input("Enter angle for Base Servo (0-180 degrees): "))
            set_servo_angle_smooth(0, base_angle)
            
            shoulder_angle = float(input("Enter angle for Shoulder Servo (0-180 degrees): "))
            set_servo_angle_smooth(1, shoulder_angle)
            
            wrist_angle = float(input("Enter angle for Wrist Servo (0-180 degrees): "))
            set_servo_angle_smooth(2, wrist_angle)
            
            roll_angle = float(input("Enter angle for Roll Servo (0-180 degrees): "))
            set_servo_angle_smooth(3, roll_angle)
            
            pitch_angle = float(input("Enter angle for Pitch Servo (0-180 degrees): "))
            set_servo_angle_smooth(4, pitch_angle)
            
            gripper_angle = float(input("Enter angle for Gripper Servo (0-180 degrees): "))
            set_servo_angle_smooth(5, gripper_angle)
            
            time.sleep(3)  # Delay between movements

        except ValueError:
            print("Invalid input. Please enter a number.")
        except KeyboardInterrupt:
            print("\nExiting program.")
            break

if __name__ == "__main__":
    main()
