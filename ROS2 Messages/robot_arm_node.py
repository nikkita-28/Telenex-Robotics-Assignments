import rclpy
from rclpy.node import Node
from custom_interfaces.msg import ObjectMetadata

class RoboticArmNode(Node):
    def __init__(self):
        super().__init__('robotic_arm_node')
        self.subscription = self.create_subscription(ObjectMetadata, 'object_metadata', self.evaluate_grasp, 10)

    def evaluate_grasp(self, msg):
        if msg.accessible:
            self.get_logger().info(f"Evaluating grasp for object: {msg.object_type}")
            # Evaluate grasp based on size, friction, and weight
            if msg.size_x < 1.0 and msg.size_y < 1.0:
                if msg.friction > 0.2 and msg.weight < 5.0:  
                    self.get_logger().info(f"Grasp feasible for {msg.object_type} (Material: {msg.material}, Weight: {msg.weight}kg, Friction: {msg.friction})")
                else:
                    self.get_logger().info(f"Grasp not feasible due to weight or friction.")
            else:
                self.get_logger().info(f"Grasp not feasible due to object size.")
        else:
            self.get_logger().info(f"Object {msg.object_type} is not accessible for grasping.")

        self.get_logger().info(f"Processing camera image with resolution: {msg.camera_image.width}x{msg.camera_image.height}")

def main(args=None):
    rclpy.init(args=args)
    node = RoboticArmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
