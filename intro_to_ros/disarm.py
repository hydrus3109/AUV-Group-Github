import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool

class ROVController2(Node):
    def __init__(self):
        super().__init__('rov_controller2')
        # This service might need a few seconds to become available
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/mavros/cmd/arming service not available, waiting again...')

    def send_disarm_command(self, arm: bool):
        """ Sends a command to arm or disarm the ROV """
        req = CommandBool.Request()
        req.value = arm  # True to arm, False to disarm
        self.future = self.arm_client.call_async(req)
        self.future.add_done_callback(self.arm_response_callback)

    def arm_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Disarming successful' if future.result().success else 'Disarming unsuccessful')
            else:
                self.get_logger().error('Arming/Disarming failed with response: %r' % response.result)
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))

def main(args=None):
    rclpy.init(args=args)
    rov_controller = ROVController2()
    rov_controller.send_disarm_command(False)  # Change to False to disarm
    rclpy.spin(rov_controller)
    rov_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()