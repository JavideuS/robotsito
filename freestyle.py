import time
from adafruit_servokit import ServoKit

# Initialize the PCA9685 and servo kit
kit = ServoKit(channels=16)

# Set frequency of the PWM signal
def init_servo(servo, pulse):
    kit.servo[0].set_pulse_width_range(500, 2500)  # adjust according to your servo's specs
    kit.servo[1].set_pulse_width_range(500, 2500)  # adjust according to your servo's specs
    kit.servo[2].set_pulse_width_range(500, 2500)  # adjust according to your servo's specs
    kit.servo[3].set_pulse_width_range(500, 2500)  # adjust according to your servo's specs

# Move the servo to a specific angle
def move_servo(servo,angle,sleep):
    kit.servo[servo].angle = angle
    time.sleep(sleep)  # adjust as needed

def main():
    # move_servo(0,0,1)
    # move_servo(1,0,1) 
    # move_servo(2,0,1) 
    # move_servo(3,0,1) 

    move_servo(0,92,1)
    move_servo(1,92,1)
    move_servo(2,92,1)
    move_servo(3,92,0.5)
    move_servo(3,0,0.5)
    move_servo(2,0,0.5)
    move_servo(1,0,0.5)
    move_servo(0,0,0.5)



if __name__ == "__main__":
    main()
    exit
# # Example usage
# try:
#     while True:
#         move_servo(0)  # Move to 0 degrees
#         move_servo(180)  # Move to 180 degrees
# except KeyboardInterrupt:
#     pass
