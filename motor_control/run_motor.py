import rclpy
from rclpy.node import Node
import canopen
import os

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

package_name = 'motor_control'
PATH = os.getcwd()+'/src/'+package_name

class MotorSubscriber(rclpy.node.Node):

    def __init__(self,*args):
        super().__init__('motor_subscriber')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('id', None),
                ('can', None)
        ])
        motor_id = self.get_parameter('id').get_parameter_value().integer_value
        can = self.get_parameter('can').get_parameter_value().string_value
        self.get_logger().info('Started: "%r"' % motor_id)
        self.id = motor_id
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.set_vel,
            10)
        # Review message before launching
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.start,
            10)
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.stop,
            10)
        # Update current driving mode
        self.subscription = self.create_subscription(
            Bool,
            'mode',
            self.update_mode,
            10)
               
        cfg = PATH +'/cfg/PD4N.eds'
        self.subscription  # prevent unused variable warning
        self.network = canopen.Network()
        self.network.connect(channel=can, bustype='socketcan')
        self.node = self.network.add_node(motor_id, cfg)
        self.node.sdo.download(0x6060, 0, b'\x03')
        #self.node.sdo.download(0x2031, 0, b'\x68\x29\x00\x00')
        #self.node.sdo.download(0x203b, 1, b'\xAC\x0D\x00\x00')
        #self.node.sdo.download(0x6073, 0, b'\xE8\x03')
        #self.node.sdo.download(0x203b, 0, b'\x00\x00\x00\x00')
        self.node.sdo.download(0x6091, 1, b'\x01\x00\x00\x00')
        self.node.sdo.download(0x6091, 2, b'\x01\x00\x00\x00')
        #self.node.sdo.download(0x6080, 0, b'\xFF\xFF\x00\x00')
        #self.node.sdo.download(0x6081, 0, b'\xFF\xFF\x00\x00')


        self.mode = True

    def set_vel(self, msg:Joy):
        # rpm = msg.to_bytes(2,'little',signed=True)
        # self.node.sdo.download(0x6042, 0, rpm)
        rpm = 0
        if msg.buttons[13]:
            rpm = 1
        elif msg.buttons[14]:
            rpm = -1
        #rpm = 0
        #if msg.axes[7]:
        #    rpm = msg.axes[7]
        #    if abs(rpm) < 0.5:
        #        rpm = 0

        if(self.mode and self.id%2):
            rpm *= -1         
        if(rpm > 0):
            # self.node.sdo.download(0x60FF, 0, b'\xD0\x07\x00\x00')
            self.node.sdo.download(0x60FF, 0, b'\x00\x03\x00\x00')
        elif(rpm < 0):
            # self.node.sdo.download(0x607E, 0, b'\x01')
            self.node.sdo.download(0x60FF, 0, b'\xFF\xFC\xFF\xFF')
            # self.node.sdo.download(0x6042, 0, b'\x18\xFC')
        else:
            self.node.sdo.download(0x60FF, 0, b'\x00\x00\x00\x00')
        self.get_logger().info('I heard: "%d"' % msg.buttons[2])

        # if msg.linear.x > 0:
        #     self.is_running = True  # Start the motor
        # elif msg.linear.x > 0:
        #     self.is_running = False  # Stop the motor

        # for _ in range(msg.linear.x):
        #     if not self.is_running:
        #         self.node.sdo.download(0x6042, 0, b'\x18\xFC')
        #     self.node.sdo.download(0x6042, 0, b'\xE8\x03')

        # self.get_logger().info('I heard: "%d"' % msg.linear.x)
 
    
    def start(self, msg:Joy):
        if(msg.buttons[2]):
            self.get_logger().info('Started: "%r"' % msg.buttons[2])
            self.node.sdo.download(0x6040, 0, b'\x06\x00')
            self.node.sdo.download(0x6040, 0, b'\x07\x00')
            self.node.sdo.download(0x6040, 0, b'\x0F\x00')

    def stop(self, msg:Joy):
        if(msg.buttons[3]):
            self.get_logger().info('Started: "%r"' % msg.buttons[3])
            self.node.sdo.download(0x6040, 0, b'\x00\x00')
            
    def disconnect(self):
        self.network.disconnect()

    def update_mode(self, msg):
        self.mode = msg.data



def main(args=None):
    rclpy.init(args=args)
    
    motor_subscriber = MotorSubscriber()
    # arg_value = motor_subscriber.get_parameter('arg_param').value
    rclpy.spin(motor_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_subscriber.disconnect()
    motor_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
