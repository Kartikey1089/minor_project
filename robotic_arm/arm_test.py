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

def move_arm_to_position(base_angle, shoulder_angle, wrist_angle, gripper_roll_angle, gripper_pitch_angle, gripper_open_close_angle):
    servo_base.angle = base_angle
    servo_shoulder.angle = shoulder_angle
    servo_wrist.angle = wrist_angle
    servo_gripper_roll.angle = gripper_roll_angle
    servo_gripper_pitch.angle = gripper_pitch_angle
    servo_gripper_open_close.angle = gripper_open_close_angle
    time.sleep(2)  # Wait for 2 seconds to allow the servos to move

# Move the arm to the closed position
def move_to_closed_position():
    move_arm_to_position(base_angle=90, shoulder_angle=90, wrist_angle=90, gripper_roll_angle=0, gripper_pitch_angle=90, gripper_open_close_angle=0)

# Move the arm to the extended position
def move_to_extended_position():
    move_arm_to_position(base_angle=90, shoulder_angle=45, wrist_angle=0, gripper_roll_angle=0, gripper_pitch_angle=0, gripper_open_close_angle=90)

# Main sequence
if __name__ == "__main__":
    print("Moving arm to closed position...")
    move_to_closed_position()

    time.sleep(2)

    print("Moving arm to extended position...")
    move_to_extended_position()

    time.sleep(2)

    print("Movement sequence complete.")
