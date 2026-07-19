import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Encoder
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import math
from tf_transformations import quaternion_from_euler

class WheelOdometryNode(Node):

    def __init__(self):
        super().__init__('wheel_odometry_node')
        

        

        #Parameters
        self.declare_parameter('robot_name','robot1')
        self.declare_parameter('wheel_radius',0.016)
        self.declare_parameter('wheel_base',0.099)
        self.declare_parameter('ticks_per_revolution', 350)
        self.first_reading=True
        
        self.robot_name = self.get_parameter('robot_name').value
        self.wheel_radius = self.get_parameter('wheel_radius').value 
        self.wheel_base = self.get_parameter('wheel_base').value
        self.ticks_per_revolution = self.get_parameter('ticks_per_revolution').value
        self.tf_broadcaster = TransformBroadcaster(self)
        #Encoder state 

        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.last_time = self.get_clock().now()

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

        if self.first_reading:
            self.prev_left_ticks = msg.left_ticks
            self.prev_right_ticks = msg.right_ticks
            self.last_time = self.get_clock().now()
            self.first_reading=False
            return

        current_left_ticks = msg.left_ticks
        current_right_ticks = msg.right_ticks   
          
        left_tick_diff = current_left_ticks - self.prev_left_ticks
        right_tick_diff = current_right_ticks - self.prev_right_ticks

        distance_per_tick = 2 * math.pi * self.wheel_radius / self.ticks_per_revolution

        left_distance = left_tick_diff * distance_per_tick
        right_distance = right_tick_diff * distance_per_tick

        distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_base
        current_time = self.get_clock().now()

        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt>0:
         self.linear_velocity = distance / dt

         self.angular_velocity = delta_theta / dt

        self.x += distance * math.cos(self.theta + delta_theta / 2.0)
        self.y += distance * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta

        self.theta=math.atan2(math.sin(self.theta), math.cos(self.theta))  # Normalize theta to [-pi, pi]

        self.prev_left_ticks = current_left_ticks
        self.prev_right_ticks = current_right_ticks 
        self.last_time = current_time

        self.publish_odometry()
        self.publish_tf()

    def publish_odometry(self):

            msg=Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = (
            f'{self.robot_name}/odom'
        )

            msg.child_frame_id = (
            f'{self.robot_name}/base_link'
        )
            #Pose
            msg.pose.pose.position.x = self.x
            msg.pose.pose.position.y = self.y
            msg.pose.pose.position.z = 0.0  
            # covariance added for the uncertainty in the pose and twist estimation
            msg.pose.covariance= [
 0.01, 0,    0,    0,    0,    0,
    0,    0.01, 0,    0,    0,    0,
    0,    0,    999,  0,    0,    0,
    0,    0,    0,    999,  0,    0,
    0,    0,    0,    0,    999,  0,
    0,    0,    0,    0,    0,    0.05
]
            msg.twist.covariance=msg.pose.covariance
            q=quaternion_from_euler(0.0, 0.0, self.theta)
            msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            msg.twist.twist.linear.x = self.linear_velocity
            msg.twist.twist.angular.z = self.angular_velocity



            self.odom_publisher.publish(msg)
    def publish_tf(self):

       t = TransformStamped()

       t.header.stamp = self.get_clock().now().to_msg()

       t.header.frame_id = f'{self.robot_name}/odom'

       t.child_frame_id = f'{self.robot_name}/base_link'


       t.transform.translation.x = self.x
       t.transform.translation.y = self.y
       t.transform.translation.z = 0.0
  

       q = quaternion_from_euler(
        0.0,
        0.0,
        self.theta
    )

       t.transform.rotation.x = q[0]
       t.transform.rotation.y = q[1]
       t.transform.rotation.z = q[2]
       t.transform.rotation.w = q[3]


       self.tf_broadcaster.sendTransform(t)
def main(args=None):

          rclpy.init(args=args)

          node = WheelOdometryNode()

          rclpy.spin(node)

          node.destroy_node()

          rclpy.shutdown()



if __name__ == '__main__':
             main()