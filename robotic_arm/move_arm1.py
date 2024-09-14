from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import board
import busio
import time

# Initialize I2C and PCA9685
i2c = busio.I2C(board.SCL, board.SDA)
pca9685 = PCA9685(i2c)
pca9685.frequency = 50

# Create servo objects for each channel (assuming 6 servos)
servos = [servo.Servo(pca9685.channels[i]) for i in range(6)]

def move_servos_to_angle(angle, delay=0.05):
    """Move all servos to a specific angle."""
    for i in range(6):
        servos[i].angle = angle
        time.sleep(delay)

def main():
    try:
        while True:
            # Move all servos to 90 degrees
            print("Moving all servos to 90 degrees...")
            move_servos_to_angle(90)

            # Wait for 2 seconds
            time.sleep(2)

            # Move all servos to 0 degrees
            print("Moving all servos to 0 degrees...")
            move_servos_to_angle(0)

            # Wait for 2 seconds
            time.sleep(2)

            # Move back to 90 degrees
            print("Moving all servos back to 90 degrees...")
            move_servos_to_angle(90)

            # Wait for 2 seconds
            time.sleep(2)

    except KeyboardInterrupt:
        print("\nProgram stopped.")

if __name__ == "__main__":
    main()
