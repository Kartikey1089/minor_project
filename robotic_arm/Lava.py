import RPi.GPIO as GPIO
import time

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)

# Define GPIO pins for each servo
servo_pins = [17, 18, 27, 22, 23, 24]  # Update these pins according to your wiring

# Set up each pin for output
for pin in servo_pins:
    GPIO.setup(pin, GPIO.OUT)

# Create PWM instances for each servo
pwm_instances = [GPIO.PWM(pin, 50) for pin in servo_pins]  # 50 Hz frequency

for pwm in pwm_instances:
    pwm.start(0)  # Start with 0% duty cycle

def set_servo_angle(pin, angle):
    """Set the angle of the servo connected to the given pin."""
    duty_cycle = angle / 18 + 2  # Convert angle to duty cycle
    GPIO.output(pin, True)
    pwm = GPIO.PWM(pin, 50)
    pwm.start(duty_cycle)
    time.sleep(0.5)  # Allow time for the servo to move
    pwm.stop()
    GPIO.output(pin, False)

def set_servo_angle_smooth(servo_index, target_angle, step_delay=0.02):
    """Move the servo to the target angle smoothly."""
    current_angle = 0  # Assume starting at 0 degrees, or keep track if needed
    step = 1 if target_angle > current_angle else -1

    for angle in range(int(current_angle), int(target_angle), step):
        set_servo_angle(servo_pins[servo_index], angle)
        time.sleep(step_delay)

    # Set the final target angle
    set_servo_angle(servo_pins[servo_index], target_angle)

def main():
    try:
        while True:
            # Ask for angles for each servo
            base_angle = float(input("Enter angle for Base Servo (0-180 degrees): "))
            set_servo_angle_smooth(0, base_angle)  # 0 for Base Servo (pin 17)

            shoulder_angle = float(input("Enter angle for Shoulder Servo (0-180 degrees): "))
            set_servo_angle_smooth(1, shoulder_angle)  # 1 for Shoulder Servo (pin 18)

            wrist_angle = float(input("Enter angle for Wrist Servo (0-180 degrees): "))
            set_servo_angle_smooth(2, wrist_angle)  # 2 for Wrist Servo (pin 27)

            roll_angle = float(input("Enter angle for Roll Servo (0-180 degrees): "))
            set_servo_angle_smooth(3, roll_angle)  # 3 for Roll Servo (pin 22)

            pitch_angle = float(input("Enter angle for Pitch Servo (0-180 degrees): "))
            set_servo_angle_smooth(4, pitch_angle)  # 4 for Pitch Servo (pin 23)

            gripper_angle = float(input("Enter angle for Gripper Servo (0-180 degrees): "))
            set_servo_angle_smooth(5, gripper_angle)  # 5 for Gripper Servo (pin 24)

            time.sleep(3)  # Delay between movements

    except ValueError:
        print("Invalid input. Please enter a number.")
    except KeyboardInterrupt:
        print("\nExiting program.")
    finally:
        # Clean up GPIO settings
        for pwm in pwm_instances:
            pwm.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
