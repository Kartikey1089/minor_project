import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# Initialize I2C bus and PCA9685 module
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50

# Initialize the servos
servo_base = servo.Servo(pca.channels[0])
servo_shoulder = servo.Servo(pca.channels[1])
servo_wrist = servo.Servo(pca.channels[2])
servo_gripper_roll = servo.Servo(pca.channels[3])
servo_gripper_pitch = servo.Servo(pca.channels[4])
servo_gripper_open_close = servo.Servo(pca.channels[5])

# Function to smoothly move the servo to a target angle
def move_servo_smooth(servo_motor, target_angle, step_delay=0.02):
    current_angle = servo_motor.angle if servo_motor.angle is not None else 90  # Default to 90 if undefined
    step_size = 1 if target_angle > current_angle else -1
    
    while abs(target_angle - current_angle) > abs(step_size):
        current_angle += step_size
        servo_motor.angle = current_angle
        time.sleep(step_delay)
    
    servo_motor.angle = target_angle

def move_base(angle):
    try:
        if angle < 0 or angle > 180:
            raise ValueError("Angle must be between 0 and 180 degrees.")
        move_servo_smooth(servo_base, angle)
        time.sleep(2)  # Wait for the servo to reach the position
    except Exception as e:
        print(f"An error occurred: {e}")

def move_y_axis(movement):
    current_angle = servo_base.angle if servo_base.angle is not None else 90  # Default to 90 if undefined
    
    # Calculate the new angle for the base servo
    new_angle = current_angle + movement
    
    # Move to the new angle
    move_base(new_angle)

# Main execution
if __name__ == "__main__":
    try:
        print("Current Base Angle:", servo_base.angle)
        
        # User input for movement
        movement = int(input("Enter movement for Y-axis (+ or - value): "))
        move_y_axis(movement)

        print("New Base Angle:", servo_base.angle)

    except KeyboardInterrupt:
        print("\nProcess interrupted by the user.")
