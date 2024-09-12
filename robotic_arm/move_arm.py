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

def set_servo_angle(servo_index, angle):
    """Set the angle of a specific servo."""
    if 0 <= servo_index < len(servos) and 0 <= angle <= 180:
        servos[servo_index].angle = angle
    else:
        print("Invalid servo index or angle")

def main():
    while True:
        try:
            # Ask for angles for each servo
            base_angle = float(input("Enter angle for Base Servo (0-180 degrees): "))
            set_servo_angle(0, base_angle)
            
            shoulder_angle = float(input("Enter angle for Shoulder Servo (0-180 degrees): "))
            set_servo_angle(1, shoulder_angle)
            
            wrist_angle = float(input("Enter angle for Wrist Servo (0-180 degrees): "))
            set_servo_angle(2, wrist_angle)
            
            roll_angle = float(input("Enter angle for Roll Servo (0-180 degrees): "))
            set_servo_angle(3, roll_angle)
            
            pitch_angle = float(input("Enter angle for Pitch Servo (0-180 degrees): "))
            set_servo_angle(4, pitch_angle)
            
            gripper_angle = float(input("Enter angle for Gripper Servo (0-180 degrees): "))
            set_servo_angle(5, gripper_angle)
            
            time.sleep(3)  # Delay between movements

        except ValueError:
            print("Invalid input. Please enter a number.")
        except KeyboardInterrupt:
            print("\nExiting program.")
            break

if __name__ == "__main__":
    main()
