import rclpy
from rclpy.node import Node
from custom_interfaces.srv import EmergencyTaskAssignment

class CommandCenter(Node):

    def __init__(self):
        super().__init__('command_center_service')
        self.srv = self.create_service(EmergencyTaskAssignment, 'assign_task', self.assign_task_callback)
        self.get_logger().info('Command Center service is ready to assign tasks.')

    def assign_task_callback(self, request, response):
        robot_id = request.robot_id
        status = request.status.lower()
        operational_state = request.operational_state.lower()

        self.get_logger().info(f"Received report from Robot ID: {robot_id}")
        self.get_logger().info(f"Current Task: {status}")
        self.get_logger().info(f"Operational State: {operational_state}")

        if operational_state == 'charging':
            response.new_task = "charging station maintenance"
            response.priority = 1  # Charging robots get low priority
            self.get_logger().info(f"Assigned task '{response.new_task}' with priority {response.priority} to Robot {robot_id}.")

        elif operational_state == 'standby':
            if status == 'none':
                response.new_task = "firefighting standby"
                response.priority = 4  # Standby robots get medium-high priority
                self.get_logger().info(f"Robot {robot_id} is in standby with no task. Assigned task '{response.new_task}' with priority {response.priority}.")
            else:
                response.new_task = f"continue {status}"
                response.priority = 2  # Low priority if they are standby with a task
                self.get_logger().info(f"Robot {robot_id} is on standby with task '{status}'. Continuing task with priority {response.priority}.")

        elif operational_state == 'active':
            if status == 'firefighting':
                response.new_task = "extinguish fire in sector B"
                response.priority = 5  # High urgency
                self.get_logger().info(f"Robot {robot_id} is actively firefighting. Assigned high priority task '{response.new_task}' with priority {response.priority}.")
            elif status == 'medical aid':
                response.new_task = "assist medical team at sector C"
                response.priority = 5  # High urgency
                self.get_logger().info(f"Robot {robot_id} is providing medical aid. Assigned high priority task '{response.new_task}' with priority {response.priority}.")
            elif status == 'none':
                response.new_task = "evacuate civilians"
                response.priority = 4  # Urgent, but not as critical
                self.get_logger().info(f"Robot {robot_id} is active with no task. Assigned task '{response.new_task}' with priority {response.priority}.")
            else:
                response.new_task = "patrol area"
                response.priority = 3  # Medium urgency
                self.get_logger().info(f"Robot {robot_id} is active with task '{status}'. Assigned task '{response.new_task}' with priority {response.priority}.")

        else:
            response.new_task = "standby"
            response.priority = 1  # Default
            self.get_logger().info(f"Robot {robot_id} has unknown operational state. Assigned default task '{response.new_task}' with priority {response.priority}.")

        
        self.get_logger().info(f"Task '{response.new_task}' assigned to Robot {robot_id} with priority {response.priority}.\n")

        return response

def main(args=None):
    rclpy.init(args=args)
    command_center = CommandCenter()
    rclpy.spin(command_center)

    command_center.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
