import rclpy
from rclpy.node import Node
from custom_interfaces.srv import CleaningTaskManagement

class CleaningTaskManagementService(Node):

    def __init__(self):
        super().__init__('cleaning_task_management_service')
        self.srv = self.create_service(CleaningTaskManagement, 'manage_cleaning_task', self.manage_cleaning_task_callback)
        self.get_logger().info('Cleaning Task Management Service is ready.')

    def manage_cleaning_task_callback(self, request, response):
        robot_id = request.robot_id
        status = request.status
        area_covered = request.area_covered

        self.get_logger().info(f'Received request from {robot_id}: Status={status}, Area Covered={area_covered} m²')

        # Determine new cleaning status based on current status and area covered
        if status == 'cleaning' and area_covered < 50.0:
            response.new_status = 'clean'
        elif status == 'cleaning' and area_covered >= 50.0:
            response.new_status = 'not_clean'
        elif status == 'charging':
            response.new_status = 'idle'
        elif status == 'idle':
            response.new_status = 'resume_cleaning'
        else:
            response.new_status = 'unknown_status'

        self.get_logger().info(f'Responding with new status: {response.new_status}')
        return response

def main(args=None):
    rclpy.init(args=args)
    cleaning_task_management_service = CleaningTaskManagementService()
    rclpy.spin(cleaning_task_management_service)
    cleaning_task_management_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
