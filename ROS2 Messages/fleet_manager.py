import rclpy
from rclpy.node import Node
from custom_interfaces.msg import RobotState
from collections import defaultdict

class FleetManager(Node):
    def __init__(self):
        super().__init__('fleet_manager')
        self.robot_data = defaultdict(lambda: {'battery': None, 'position': None, 'velocity': None, 'task_engaged': None, 'temperature': None, 'obstacle': None})

        self.create_subscription(RobotState, 'robot/state', self.robot_state_callback, 10)
        self.timer = self.create_timer(5.0, self.assign_tasks)

    def robot_state_callback(self, msg):
        robot_id = msg.robot_id
        self.robot_data[robot_id] = {
            'battery': msg.battery.percentage,
            'position': (msg.position.x, msg.position.y, msg.position.z),
            'velocity': (msg.velocity.linear.x, msg.velocity.angular.z),
            'task_engaged': msg.task_engaged.data,
            'temperature': msg.temperature.temperature,
            'obstacle': msg.obstacle.ranges
        }

    def assign_tasks(self):
        # Define task locations
        task_1_location = (5.0, 5.0)

        # Task 1: Inventory Scanning
        self.assign_inventory_scanning(task_1_location)

        # Task 2: Docking for Software Update
        self.assign_docking()

        # Task 3: Temperature Monitoring
        self.assign_temperature_monitoring()

        # Task 4: Obstacle Navigation
        self.assign_obstacle_navigation()

        # Task 5: Relocation to Charging Area
        self.assign_relocation_to_charging()

    def assign_inventory_scanning(self, location):
        closest_robot = None
        min_distance = float('inf')

        for robot_id, data in self.robot_data.items():
            if not data['task_engaged'] and data['battery'] > 40:
                distance = self.calculate_distance(data['position'], location)
                if distance < min_distance:
                    min_distance = distance
                    closest_robot = robot_id

        if closest_robot is not None:
            # self.get_logger().info(f'Inventory scanning task assigned to robot {closest_robot}')
            self.robot_data[closest_robot]['task_engaged'] = True
            print(f'Inventory scanning task assigned to robot {closest_robot}')

    def assign_docking(self):
        for robot_id, data in self.robot_data.items():
            if not data['task_engaged'] and 50 <= data['battery'] <= 80:
                # self.get_logger().info(f'Docking for software update task assigned to robot {robot_id}')
                self.robot_data[robot_id]['task_engaged'] = True
                print(f'Docking for software update task assigned to robot {robot_id}')
                break

    def assign_temperature_monitoring(self):
        for robot_id, data in self.robot_data.items():
            if data['temperature'] is not None and not data['task_engaged']:
                # self.get_logger().info(f'Temperature monitoring task assigned to robot {robot_id}')
                self.robot_data[robot_id]['task_engaged'] = True
                print(f'Temperature monitoring task assigned to robot {robot_id}')
                break

    def assign_obstacle_navigation(self):
        for robot_id, data in self.robot_data.items():
            if not data['task_engaged'] and self.has_obstacles(data['obstacle']):
                # self.get_logger().info(f'Obstacle navigation task assigned to robot {robot_id}')
                self.robot_data[robot_id]['task_engaged'] = True
                print(f'Obstacle navigation task assigned to robot {robot_id}')
                break

    def assign_relocation_to_charging(self):
        for robot_id, data in self.robot_data.items():
            if data['battery'] > 90 and not data['task_engaged']:
                # self.get_logger().info(f'Relocation task assigned to robot {robot_id}')
                self.robot_data[robot_id]['task_engaged'] = True
                print(f'Relocation task assigned to robot {robot_id}')
                break

    @staticmethod
    def calculate_distance(robot_position, task_location):
        rx, ry, _ = robot_position
        tx, ty = task_location
        return ((rx - tx) ** 2 + (ry - ty) ** 2) ** 0.5

    @staticmethod
    def has_obstacles(obstacle_ranges):
        # Check if there are obstacles within a 1 meter range
        return any(range < 1.0 for range in obstacle_ranges)

def main(args=None):
    rclpy.init(args=args)
    fleet_manager = FleetManager()
    rclpy.spin(fleet_manager)
    fleet_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
