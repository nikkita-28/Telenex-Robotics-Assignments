import rclpy
import time
from rclpy.node import Node
from custom_interfaces.srv import CleaningTaskManagement

class CleaningRobotClient(Node):

    def __init__(self):
        super().__init__('cleaning_robot_client')
        self.cli = self.create_client(CleaningTaskManagement, 'manage_cleaning_task')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = CleaningTaskManagement.Request()

    def send_request(self, robot_id, status, area_covered):
        self.req.robot_id = robot_id
        self.req.status = status
        self.req.area_covered = area_covered

        # Log the request being sent
        self.get_logger().info(f"Robot {robot_id} sending status: {status}, area_covered: {area_covered} m²")
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    rclpy.init(args=args)

    # Initialize the CleaningRobotClient node
    cleaning_robot_client = CleaningRobotClient()

    robot_scenarios = [
        {"robot_id": "Robot_A", "status": "cleaning", "area_covered": 30.0},
        {"robot_id": "Robot_B", "status": "cleaning", "area_covered": 55.0},
        {"robot_id": "Robot_C", "status": "charging", "area_covered": 0.0},
        {"robot_id": "Robot_D", "status": "idle", "area_covered": 0.0},
    ]

    for scenario in robot_scenarios:
        robot_id = scenario["robot_id"]
        status = scenario["status"]
        area_covered = scenario["area_covered"]
        
        response = cleaning_robot_client.send_request(robot_id, status, area_covered)

        cleaning_robot_client.get_logger().info(f"Response for {robot_id}: New Status - {response.new_status}")
        time.sleep(3)

    cleaning_robot_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
