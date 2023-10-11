import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy  
import Jetson.GPIO as GPIO
import time
from std_msgs.msg import Bool

# from std_msgs.msg import String
# Define the GPIO pins for direction and pulse signals
# self.STEP_PIN = 99
# self.DIR1_PIN = 99
# BRAKE_PIN = 9

#working: 32(dir), 16(dir), 11(24mA for input)

class BaseMotorControl(Node):
    def __init__(self,*args):
        super().__init__("base_motor_control_node")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('step', None),
                ('dir1', None),
                ('dir2', None),
        ])

        self.STEP_PIN = self.get_parameter('step').get_parameter_value().integer_value
        self.DIR1_PIN = self.get_parameter('dir1').get_parameter_value().integer_value
        self.DIR2_PIN = self.get_parameter('dir2').get_parameter_value().integer_value

        # Subscribe to the Joy command topic
        self.subscription = self.create_subscription(
            Joy,
            "joy",
            self.joy_callback,
            2
        )
        self.publisher = self.create_publisher(Bool, 'mode', 10)
        self.steer_mode = 1
        self.mode = True

        # Initialize the GPIO pins
        GPIO.setmode(GPIO.BOARD)

        GPIO.setwarnings(False)

        GPIO.setup(self.STEP_PIN, GPIO.OUT)
        GPIO.setup(self.DIR1_PIN, GPIO.OUT)
        GPIO.setup(self.DIR2_PIN, GPIO.OUT)
        # GPIO.setup(BRAKE_PIN, GPIO.OUT)       

        # Initialize the position variables
        self.position = 0
        
        # Flag to indicate if the motor is currently running
        self.is_running = False  

    def joy_callback(self, msg:Joy):

        # Process the Joy message and generate stepper motor control commands
        # base_vel = msg.angular.x  # Assuming angular.z represents the rotational velocity
        steer = msg.buttons[0]
        steer_back = msg.buttons[1]
        # Set the direction based on the sign of the steps_per_second

        if (steer == 1 and not self.mode):
            GPIO.output(self.DIR1_PIN, GPIO.HIGH)
            GPIO.output(self.DIR2_PIN, GPIO.LOW)
            self.steer()
            self.mode = True
        if (steer_back == 1 and self.mode):
            GPIO.output(self.DIR1_PIN, GPIO.LOW)
            GPIO.output(self.DIR2_PIN, GPIO.HIGH)
            self.steer()
            self.mode = False
        # Output the current mode
        msg_mode = Bool()
        msg_mode.data = self.mode
        self.publisher.publish(msg_mode)
        self.get_logger().info(f"Base position: {self.position}")
        self.get_logger().info(str(self.mode))

    
    def steer(self):
        for i in range(3333):
            GPIO.output(self.STEP_PIN, GPIO.HIGH)
            time.sleep(0.001)  # Adjust sleep time as per your stepper motor driver requirements
            GPIO.output(self.STEP_PIN, GPIO.LOW)
            time.sleep(0.001)
            i += 1

def main(args=None):
    rclpy.init(args=args)

    node = BaseMotorControl()
    
    rclpy.spin(node)
    rclpy.shutdown()
    
    GPIO.cleanup()  # Clean up GPIO pins on program exit

if __name__ == "__main__":
    main()
