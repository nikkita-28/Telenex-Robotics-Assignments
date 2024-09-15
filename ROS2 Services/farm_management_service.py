import rclpy
from rclpy.node import Node
from custom_interfaces.srv import HarvestSchedule  

class FarmManagementService(Node):
    def __init__(self):
        super().__init__('farm_management_service')
        self.srv = self.create_service(HarvestSchedule, 'harvest_schedule', self.handle_schedule_request)

    def handle_schedule_request(self, request, response):
        self.get_logger().info(f'Received data from robot {request.robot_id}')
        
        # Simple decision logic based on status and crop yield
        if request.status == "harvesting" and request.crop_yield > 100.0:
            response.schedule = "nextfield"
        elif request.status == "maintenance":
            response.schedule = "maintenance"
        else:
            response.schedule = "idle"
        
        self.get_logger().info(f'Scheduling response: {response.schedule}')
        return response

def main(args=None):
    rclpy.init(args=args)
    farm_management_service = FarmManagementService()
    rclpy.spin(farm_management_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
