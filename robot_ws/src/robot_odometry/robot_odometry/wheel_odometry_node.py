import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Encoder
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from builtin_interfaces.msg import Time
import math
from tf_transformations import quaternion_from_euler

class WheelOdometryNode(Node):

    def _init_(self):
        super()._init_('wheel_odometry_node')

        

        #Parameters
        self.declare_parameter('robot_name','robot1')
        self.declare_parameter('wheel_radius',0.016)
        self.declare_parameter('wheel_base',0.099)
        self.declare_parameter('ticks_per_revolution', 350)
        
        self.robot_name = self.get_parameter('robot_name').value
        self.wheel_radius = self.get_parameter('wheel_radius').value 
        self.wheel_base = self.get_parameter('wheel_base').value
        self.ticks_per_revolution = self.get_parameter('ticks_per_revolution').value

        #Encoder state 

        self.prev_left_ticks = 0
        self.prev_right_ticks = 0

        #Robot pose 
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        #Topics 
        
        encoder_topic=(f'/{self.robot_name}/encoder')
        odom_topic=(f'/{self.robot_name}/wheel_odom')
        
        #Subscriber 
        
        self.encoder_subscriber = self.create_subscription(Encoder, encoder_topic, self.encoder_callback, 10)

        #Publisher
        self.odom_publisher = self.create_publisher(Odometry, odom_topic, 10)
        
        #Encoder callback function
    def encoder_callback(self, msg):

        current_left_ticks = msg.left_ticks
        current_right_ticks = msg.right_ticks   

        delta_left=(current_left_ticks - self.prev_left_ticks) * (2 * math.pi * self.wheel_radius) / self.ticks_per_revolution
        delta_right=(current_right_ticks - self.prev_right_ticks) * (2 * math.pi * self.wheel_radius) / self.ticks_per_revolution   
  
        