import rclpy
from rclpy.node import Node
from custom_interfaces.srv import HarvestSchedule


class RobotClient(Node):
    def __init__(self):
        super().__init__('robot_client')
        self.cli = self.create_client(HarvestSchedule, 'harvest_schedule')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for farm management service...')
        
        self.req = HarvestSchedule.Request()

    def send_request(self, robot_id, crop_yield, status):
        self.req.robot_id = robot_id
        self.req.crop_yield = crop_yield
        self.req.status = status

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    robot_client = RobotClient()

    response = robot_client.send_request('robot_1', 120.0, 'harvesting')
    if response:
        robot_client.get_logger().info(f'Received schedule for robot_1: {response.schedule}')

    response = robot_client.send_request('robot_2', 30.0, 'idle')
    if response:
        robot_client.get_logger().info(f'Received schedule for robot_2: {response.schedule}')

    response = robot_client.send_request('robot_3', 90.0, 'harvesting')
    if response:
        robot_client.get_logger().info(f'Received schedule for robot_3: {response.schedule}')

    response = robot_client.send_request('robot_4', 200.0, 'maintenance')
    if response:
        robot_client.get_logger().info(f'Received schedule for robot_4: {response.schedule}')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
