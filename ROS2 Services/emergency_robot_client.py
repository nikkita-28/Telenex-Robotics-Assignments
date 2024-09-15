import rclpy
from rclpy.node import Node
from custom_interfaces.srv import EmergencyTaskAssignment
import time

class EmergencyRobotClient(Node):

    def __init__(self):
        super().__init__('emergency_robot_client')
        self.cli = self.create_client(EmergencyTaskAssignment, 'assign_task')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = EmergencyTaskAssignment.Request()

    def send_request(self, robot_id, status, operational_state):
        self.req.robot_id = robot_id
        self.req.status = status
        self.req.operational_state = operational_state


        self.get_logger().info(f"Robot {robot_id} sending status: {status}, operational_state: {operational_state}")
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    rclpy.init(args=args)

    # Initialize the EmergencyRobotClient node
    emergency_robot_client = EmergencyRobotClient()

    robot_scenarios = [
        {"robot_id": "Robot_001", "status": "firefighting", "operational_state": "active"},
        {"robot_id": "Robot_002", "status": "medical aid", "operational_state": "active"},
        {"robot_id": "Robot_003", "status": "none", "operational_state": "charging"},
        {"robot_id": "Robot_004", "status": "patrolling", "operational_state": "standby"},
        {"robot_id": "Robot_005", "status": "none", "operational_state": "standby"},
        {"robot_id": "Robot_006", "status": "evacuation", "operational_state": "active"},
        {"robot_id": "Robot_007", "status": "none", "operational_state": "active"},
        {"robot_id": "Robot_008", "status": "firefighting", "operational_state": "charging"},
    ]

    
    for scenario in robot_scenarios:
        robot_id = scenario["robot_id"]
        status = scenario["status"]
        operational_state = scenario["operational_state"]
        
        response = emergency_robot_client.send_request(robot_id, status, operational_state)

        emergency_robot_client.get_logger().info(f"Response for {robot_id}: Task - {response.new_task}, Priority - {response.priority}")
        time.sleep(3)

    emergency_robot_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
