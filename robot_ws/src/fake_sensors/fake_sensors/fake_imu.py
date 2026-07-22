import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu


class FakeIMU(Node):

    def __init__(self):

        super().__init__('fake_imu_node')
        


        # Robot namespace 
        self.declare_parameter(
            'robot_name',
            'robot1'
        )

        self.robot_name = self.get_parameter(
            'robot_name'
        ).value


        imu_topic = f'/{self.robot_name}/imu'


        self.publisher = self.create_publisher(
            Imu,
            imu_topic,
            10
        )


        # Fake values
        self.yaw_rate = 0.0


        self.timer = self.create_timer(
            0.1,
            self.publish_imu
        )


        self.get_logger().info(
            f'Fake IMU started for {self.robot_name}'
        )
    def publish_imu(self):
  
     msg = Imu()

     # Timestamp
     msg.header.stamp = self.get_clock().now().to_msg()

     # Frame
     msg.header.frame_id = f'{self.robot_name}/base_link'


     # Orientation unavailable
     msg.orientation.w = 1.0
     msg.orientation_covariance[0] = -1


     # Gyro
     msg.angular_velocity.z = 0.2

     msg.angular_velocity_covariance = [
        0.01,0.0,0.0,
        0.0,0.01,0.0,
        0.0,0.0,0.01
    ]


    # Accelerometer
     msg.linear_acceleration.z = 9.81

     msg.linear_acceleration_covariance = [
        0.1,0.0,0.0,
        0.0,0.1,0.0,
        0.0,0.0,0.1
    ]


     self.publisher.publish(msg)

def main(args=None):

    rclpy.init(args=args)

    node = FakeIMU()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()



if __name__ == '__main__':
    main()