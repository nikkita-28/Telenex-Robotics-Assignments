import rclpy
from rclpy.node import Node
from custom_interfaces.msg import AerialStatus

class MonitoringStation(Node):
    def __init__(self):
        super().__init__('monitoring_station')

        # Create a subscription to the custom message
        self.create_subscription(
            AerialStatus,
            'aerial_vehicle_status',
            self.aerial_status_callback,
            10
        )

    def aerial_status_callback(self, msg):
        # Print data for debugging (optional)
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(
            f'Time: {current_time}, Altitude: {msg.altitude}, Velocity x: {msg.velocity.x}, Power: {msg.power_level}'
        )

def main(args=None):
    rclpy.init(args=args)
    monitoring_station = MonitoringStation()
    rclpy.spin(monitoring_station)
    monitoring_station.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
