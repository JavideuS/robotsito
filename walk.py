import time
import sys
from adafruit_servokit import ServoKit



# Initialize the PCA9685 and servo kit
kit = ServoKit(channels=16)

#Dictionary with a key_pair value
#servo_num - Angle
#It the servo_number doesnt appear means that it want initialized
servo_status = {}

def print_error(message):
    """
    Prints the given message to standard error (stderr).
    
    Parameters:
    message (str): The error message to print.
    """
    print(message, file=sys.stderr)


def init_servo(servo_num, min_pulse,max_pulse):
    """
    This functions sets the frequency of the PWM signal of the selected servo-motor
    Parameters:
        servo_num (int)-> [0,15] In this case you can initialize the servos in 16 differentes channels from 0 to 15
        min_pulse (int) -> Minimum pulse width in microseconds. Default is 500
        max_pulse (int) -> Maximum pulse width in microseconds. Default is 2500
    """ 

    kit.servo[servo_num].set_pulse_with_range(min_pulse,max_pulse)
    #By default every servo will have the angle set to 0 as inidicator
    servo_status [servo_num] = 0


# Move the servo to a specific angle
def move_servo(servo_num,angle,sleep):
    """
    Moves a specific servo to the desired angle and pauses for the given time
    Parameters:
        servo_num (int): The servo channel number to move [0,15].
        angle (float): The target angle to set for the servo. Range -> [0,180] inclusive
        sleep (float): Time to pause after moving the servo, in seconds

    """
    if(servo_num not in servo_status.keys()):
        print_error("Servo not initialized")
    elif(angle < 0 or angle > 180):
        print_error("Angle out of bounds, range [0,180] inclusive")
    else:
        kit.servo[servo_num].angle = angle
        time.sleep(sleep)  
        servo_status[servo_num] = angle

def walk():
    """"
    This functions make the robot to walk
    It will move dependending on the current angle of the servos
    Parameters:
        None
    Usage:
        The 16 servos need to be initialized
    """
    if(len(servo_status) != 16):
        print_error("Initialize all the servos before walking")
    else:
        if(servo_status[0] == 90):
            move_servo(0,0,0)
        else:
            move_servo(0,180,0)



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
