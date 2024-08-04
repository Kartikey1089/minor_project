import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# Initialize I2C bus and PCA9685 module
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50

# Initialize servo on channel 0
servo_motor = servo.Servo(pca.channels[0])

# Function to move the servo to specific angles
def move_servo():
    print("Moving servo to 0 degrees")
    servo_motor.angle = 0
    time.sleep(1)
    
    print("Moving servo to 90 degrees")
    servo_motor.angle = 90
    time.sleep(1)
    
    print("Moving servo to 180 degrees")
    servo_motor.angle = 180
    time.sleep(1)
    
    print("Moving servo back to 90 degrees")
    servo_motor.angle = 90
    time.sleep(1)
    
    print("Moving servo back to 0 degrees")
    servo_motor.angle = 0
    time.sleep(1)

# Main loop
while True:
    move_servo()

