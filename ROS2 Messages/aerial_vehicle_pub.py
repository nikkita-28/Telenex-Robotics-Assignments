import rclpy
from rclpy.node import Node
from custom_interfaces.msg import AerialStatus
from geometry_msgs.msg import Vector3 
from random import uniform
from time import sleep

class AerialVehicle(Node):
    def __init__(self):
        super().__init__('aerial_vehicle')
        self.status_pub = self.create_publisher(AerialStatus, 'aerial_vehicle_status', 10)
        self.timer = self.create_timer(1.0, self.publish_status)

    def publish_status(self):
        msg = AerialStatus()
        msg.velocity = Vector3()
        msg.velocity.x = uniform(-5.0, 5.0)
        msg.velocity.y = uniform(-5.0, 5.0)
        msg.velocity.z = uniform(0.0, 2.0)
        msg.altitude = uniform(0.0, 100.0)
        msg.power_level = uniform(50.0, 100.0)
        self.status_pub.publish(msg)
        self.get_logger().info(f"Published Status: Velocity={msg.velocity}, Altitude={msg.altitude}, Power={msg.power_level}")

def main(args=None):
    rclpy.init(args=args)
    aerial_vehicle = AerialVehicle()
    rclpy.spin(aerial_vehicle)
    aerial_vehicle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
