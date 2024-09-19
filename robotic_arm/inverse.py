import math
import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# Link lengths (in mm)
L1 = 40  # Base to Shoulder
L2 = 137  # Shoulder to Elbow
L3 = 110  # Elbow to Wrist
L4 = 30   # Wrist to Roll
L5 = 120  # Roll to Gripper Tip

# Servo PWM configuration
PWM_FREQUENCY = 50
SERVO_MIN = 0  # Min pulse width (in microseconds)
SERVO_MAX = 180  # Max pulse width (in microseconds)
STEP_DELAY = 0.05  # Delay between steps (in seconds)

# Initialize PCA9685
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = PWM_FREQUENCY

# Initialize servos
servo_base = servo.Servo(pca.channels[0], min_pulse=SERVO_MIN, max_pulse=SERVO_MAX)
servo_shoulder = servo.Servo(pca.channels[1], min_pulse=SERVO_MIN, max_pulse=SERVO_MAX)
servo_elbow = servo.Servo(pca.channels[2], min_pulse=SERVO_MIN, max_pulse=SERVO_MAX)
servo_roll = servo.Servo(pca.channels[3], min_pulse=SERVO_MIN, max_pulse=SERVO_MAX)
servo_pitch = servo.Servo(pca.channels[4], min_pulse=SERVO_MIN, max_pulse=SERVO_MAX)
servo_gripper = servo.Servo(pca.channels[5], min_pulse=SERVO_MIN, max_pulse=SERVO_MAX)

def calculate_angles(x, y, z):
    """
    Calculate the angles for the robotic arm's base, shoulder, and elbow joints 
    to reach the desired (x, y, z) position.
    """
    # Step 1: Calculate base angle (theta1)
    theta1 = math.atan2(y, x)
    
    # Step 2: Calculate wrist position in 2D (r, z) plane
    r = math.sqrt(x**2 + y**2)  # Projected distance in xy-plane
    z_offset = z - L1           # Adjust z to account for base height
    
    # Step 3: Use trigonometry to calculate shoulder and elbow angles
    D = (r**2 + z_offset**2 - L2**2 - L3**2) / (2 * L2 * L3)
    
    if abs(D) > 1:
        raise ValueError("Target point is out of reach.")
    
    # Elbow angle (theta3)
    theta3 = math.atan2(math.sqrt(1 - D**2), D)
    
    # Shoulder angle (theta2)
    theta2 = math.atan2(z_offset, r) - math.atan2(L3 * math.sin(theta3), L2 + L3 * math.cos(theta3))
    
    # Roll and Pitch Calculation (Simplified)
    # Assuming roll and pitch are zero for simplicity
    roll_angle = 0  # degrees
    pitch_angle = 0  # degrees
    
    # Convert from radians to degrees
    theta1_deg = math.degrees(theta1)
    theta2_deg = math.degrees(theta2)
    theta3_deg = math.degrees(theta3)
    
    return theta1_deg, theta2_deg, theta3_deg, roll_angle, pitch_angle

def move_servo_smooth(servo_motor, target_angle, step_delay=STEP_DELAY):
    """
    Move the servo smoothly from its current position to the target angle.
    """
    current_angle = servo_motor.angle
    step_size = 1 if target_angle > current_angle else -1
    
    while abs(target_angle - current_angle) > abs(step_size):
        current_angle += step_size
        servo_motor.angle = current_angle
        time.sleep(step_delay)
    
    servo_motor.angle = target_angle  # Ensure the final angle is set

def move_arm_to_position(x, y, z):
    """
    Moves the robotic arm to the given (x, y, z) position by calculating 
    the joint angles and then sending those angles to the servos.
    """
    try:
        theta1, theta2, theta3, roll_angle, pitch_angle = calculate_angles(x, y, z)
        
        # Smoothly move each servo to its target position
        move_servo_smooth(servo_base, theta1)
        move_servo_smooth(servo_shoulder, theta2)
        move_servo_smooth(servo_elbow, theta3)
        
        # Move roll, pitch, and gripper servos (if needed)
        move_servo_smooth(servo_roll, roll_angle)
        move_servo_smooth(servo_pitch, pitch_angle)
        # Example: Set gripper angle as needed
        gripper_angle = 90  # Adjust as necessary
        move_servo_smooth(servo_gripper, gripper_angle)

        print(f"Base Angle: {theta1:.2f}°")
        print(f"Shoulder Angle: {theta2:.2f}°")
        print(f"Elbow Angle: {theta3:.2f}°")
        print(f"Roll Angle: {roll_angle:.2f}°")
        print(f"Pitch Angle: {pitch_angle:.2f}°")
        
    except ValueError as e:
        print(e)

# Example usage:
x_target = 100  # mm
y_target = 100  # mm
z_target = 150  # mm

move_arm_to_position(x_target, y_target, z_target)
