import rclpy
from rclpy.node import Node
from custom_interfaces.msg import RobotState
import random

class RobotTelemetryPublisher(Node):
    def __init__(self, robot_id):
        super().__init__(f'robot_{robot_id}_publisher')
        self.robot_id = robot_id

        self.state_pub = self.create_publisher(RobotState, 'robot/state', 10)
        self.timer = self.create_timer(1.0, self.publish_telemetry)

    def publish_telemetry(self):
        # Create RobotState message
        state_msg = RobotState()

        # Robot ID
        state_msg.robot_id = self.robot_id

        # Battery state
        if self.robot_id == 3:
            state_msg.battery.percentage = 35.0
        elif self.robot_id==1:
            state_msg.battery.percentage = 70.0
        else:
            state_msg.battery.percentage = random.uniform(0, 100)

        # Position state
        state_msg.position.x = random.uniform(0, 10)
        state_msg.position.y = random.uniform(0, 10)
        state_msg.position.z = 0.0

        # Velocity state
        state_msg.velocity.linear.x = random.uniform(0, 1)
        state_msg.velocity.angular.z = random.uniform(-1, 1)

        # Task engagement
        state_msg.task_engaged.data = False

        # Temperature
        state_msg.temperature.temperature = random.uniform(-10, 40)

        # Obstacle detection (mock example)
        state_msg.obstacle.ranges = [random.uniform(0.1, 10) for _ in range(10)]

        # Publish the message
        self.state_pub.publish(state_msg)
        print(f'Published state for robot {self.robot_id} : {state_msg}')

def main(args=None):
    rclpy.init(args=args)
    nodes = []
    try:
        for i in range(5):
            robot_id = i+1
            robot = RobotTelemetryPublisher(robot_id)
            nodes.append(robot)
            rclpy.spin_once(robot)
    except KeyboardInterrupt:
        pass
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
