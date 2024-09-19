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

def move_servo_smooth(servo_motor, target_angle, step_delay=0.02, step_size=1):
    """
    Smoothly move the servo motor from its current angle to the target angle.
    """
    target_angle = max(0, min(180, target_angle))  # Clamp target angle between 0 and 180 degrees
    current_angle = servo_motor.angle if servo_motor.angle is not None else 90  # Default to 90 degrees if not set

    step_size = step_size if target_angle > current_angle else -step_size

    for angle in range(int(current_angle), int(target_angle), step_size):
        servo_motor.angle = angle
        time.sleep(step_delay)

    # Ensure the final angle is set correctly
    servo_motor.angle = target_angle

def move_arm_to_position(base_angle, shoulder_angle, wrist_angle, gripper_roll_angle, gripper_pitch_angle, gripper_open_close_angle, step_delay=0.02):
    move_servo_smooth(servo_base, base_angle, step_delay)
    move_servo_smooth(servo_shoulder, shoulder_angle, step_delay)
    move_servo_smooth(servo_wrist, wrist_angle, step_delay)
    move_servo_smooth(servo_gripper_roll, gripper_roll_angle, step_delay)
    move_servo_smooth(servo_gripper_pitch, gripper_pitch_angle, step_delay)
    move_servo_smooth(servo_gripper_open_close, gripper_open_close_angle, step_delay)

# Move the arm to the closed position
def move_to_closed_position():
    move_arm_to_position(base_angle=90, shoulder_angle=90, wrist_angle=45, gripper_roll_angle=0, gripper_pitch_angle=0, gripper_open_close_angle=50)

# Move the arm to the extended position
def move_to_object():
    move_arm_to_position(base_angle=90, shoulder_angle=160, wrist_angle=45, gripper_roll_angle=0, gripper_pitch_angle=0, gripper_open_close_angle=0)
    
def move_backt():
    move_arm_to_position(base_angle=90, shoulder_angle=160, wrist_angle=45, gripper_roll_angle=0, gripper_pitch_angle=0, gripper_open_close_angle=0)

# Main sequence
if __name__ == "__main__":
    print("Moving arm to closed position...")
    move_to_closed_position()

    time.sleep(2)

    print("Moving arm to object...")
    move_to_object()

    time.sleep(2)

    print("Moving back...")
    move_back()
    
    time.sleep(2)
    
    print("Movement sequence complete.")
