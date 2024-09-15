import rclpy
from rclpy.node import Node
from custom_interfaces.msg import ObjectMetadata
from geometry_msgs.msg import Point, Quaternion
from sensor_msgs.msg import Image
import random

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.publisher_ = self.create_publisher(ObjectMetadata, 'object_metadata', 10)
        self.timer = self.create_timer(1.0, self.publish_object_data)

    def publish_object_data(self):
        msg = ObjectMetadata()
        
        # Simulate object type
        msg.object_type = random.choice(['cylinder', 'box', 'sphere'])
        
        # Simulate position and orientation
        msg.position = Point()
        msg.position.x = random.uniform(0.0, 5.0)
        msg.position.y = random.uniform(0.0, 5.0)
        msg.position.z = random.uniform(0.0, 5.0)
        msg.orientation = Quaternion()
        msg.orientation.x = random.uniform(0.0, 1.0)
        msg.orientation.y = random.uniform(0.0, 1.0)
        msg.orientation.z = random.uniform(0.0, 1.0)
        msg.orientation.w = random.uniform(0.0, 1.0)
        
        # Simulate object size
        msg.size_x = random.uniform(0.1, 1.0)
        msg.size_y = random.uniform(0.1, 1.0)
        msg.size_z = random.uniform(0.1, 1.0)
        
        # Simulate material, friction, and weight
        msg.material = random.choice(['metal', 'plastic', 'wood'])
        msg.friction = random.uniform(0.1, 0.9)
        msg.weight = random.uniform(0.1, 10.0)  # Weight in kg
        
        # Simulate accessibility
        msg.accessible = random.choice([True, False])

        # Simulate camera image (dummy data)
        msg.camera_image = Image()
        msg.camera_image.height = 480
        msg.camera_image.width = 640
        msg.camera_image.encoding = 'rgb8'
        msg.camera_image.is_bigendian = 0
        msg.camera_image.step = 640 * 3
        msg.camera_image.data = [random.randint(0, 255) for _ in range(640 * 480 * 3)]  # Random image data

        # Publish the object metadata
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published object: {msg.object_type} (Material: {msg.material}, Friction: {msg.friction}, Weight: {msg.weight}kg)")

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
