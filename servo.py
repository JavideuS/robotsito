import time
import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# Initialize I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Create PCA9685 instance
pca = PCA9685(i2c)
pca.frequency = 50  # 50Hz for servos

# Create servo objects for each channel (0-15)
servo0 = servo.Servo(pca.channels[0])
servo1 = servo.Servo(pca.channels[1])
servo2 = servo.Servo(pca.channels[2])
servo3 = servo.Servo(pca.channels[3])

# You can create servos for all 16 channels if needed
servos = []
for i in range(16):
    servos.append(servo.Servo(pca.channels[i]))

def move_servo_to_angle(servo_obj, angle):
    """Move servo to specific angle (0-180 degrees)"""
    if 0 <= angle <= 180:
        servo_obj.angle = angle
    else:
        print(f"Angle {angle} out of range (0-180)")

def move_servo_to_position(servo_obj, position):
    """Move servo to fractional position (0.0-1.0)"""
    if 0.0 <= position <= 1.0:
        servo_obj.fraction = position
    else:
        print(f"Position {position} out of range (0.0-1.0)")

# Example usage - robot leg movement
def move_robot_leg(leg_servos, hip_angle, knee_angle, ankle_angle):
    """Move a robot leg with hip, knee, ankle servos"""
    move_servo_to_angle(leg_servos['hip'], hip_angle)
    move_servo_to_angle(leg_servos['knee'], knee_angle)
    move_servo_to_angle(leg_servos['ankle'], ankle_angle)

# Define robot legs (example for quadruped)
robot_legs = {
    'front_left': {
        'hip': servos[0],
        'knee': servos[1], 
        'ankle': servos[2]
    },
    'front_right': {
        'hip': servos[3],
        'knee': servos[4],
        'ankle': servos[5]
    },
    'back_left': {
        'hip': servos[6],
        'knee': servos[7],
        'ankle': servos[8]
    },
    'back_right': {
        'hip': servos[9],
        'knee': servos[10],
        'ankle': servos[11]
    }
}

# Example movements
if __name__ == "__main__":
    try:
        # Move individual servos
        print("Moving servos to center position...")
        move_servo_to_angle(servo0, 90)
        move_servo_to_angle(servo1, 90)
        time.sleep(1)
        
        # Move using fractional positions
        print("Moving using fractional positions...")
        move_servo_to_position(servo0, 0.0)  # 0 degrees
        move_servo_to_position(servo1, 0.5)  # 90 degrees
        move_servo_to_position(servo2, 1.0)  # 180 degrees
        time.sleep(1)
        
        # Robot walking example
        print("Robot walking sequence...")
        for step in range(3):
            # Lift front left leg
            move_robot_leg(robot_legs['front_left'], 45, 120, 60)
            time.sleep(0.5)
            
            # Put down front left leg
            move_robot_leg(robot_legs['front_left'], 90, 90, 90)
            time.sleep(0.5)
            
            # Lift front right leg
            move_robot_leg(robot_legs['front_right'], 135, 120, 60)
            time.sleep(0.5)
            
            # Put down front right leg
            move_robot_leg(robot_legs['front_right'], 90, 90, 90)
            time.sleep(0.5)
        
        # Smooth movement example
        print("Smooth movement demo...")
        for angle in range(0, 181, 5):
            move_servo_to_angle(servo0, angle)
            time.sleep(0.05)
        
        for angle in range(180, -1, -5):
            move_servo_to_angle(servo0, angle)
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("Stopping servo control...")
    finally:
        # Deinitialize PCA9685
        pca.deinit()

# Additional utility functions
def set_all_servos_to_center():
    """Set all servos to center position (90 degrees)"""
    for servo_obj in servos:
        move_servo_to_angle(servo_obj, 90)

def disable_all_servos():
    """Disable all servo outputs"""
    for i in range(16):
        pca.channels[i].duty_cycle = 0

def create_servo_sequence(servo_obj, angles, delays):
    """Execute a sequence of servo movements"""
    for angle, delay in zip(angles, delays):
        move_servo_to_angle(servo_obj, angle)
        time.sleep(delay)

# Example servo sequence
def leg_stretch_sequence(leg_servos):
    """Example leg stretching sequence"""
    sequences = {
        'hip': ([90, 45, 135, 90], [0.5, 1.0, 1.0, 0.5]),
        'knee': ([90, 60, 120, 90], [0.5, 1.0, 1.0, 0.5]),
        'ankle': ([90, 60, 120, 90], [0.5, 1.0, 1.0, 0.5])
    }
    
    for joint, (angles, delays) in sequences.items():
        create_servo_sequence(leg_servos[joint], angles, delays)
