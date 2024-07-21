#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode
import numpy as np

class ModeController(Node):
    def __init__(self):
        super().__init__('rov_controller')
        # This service might need a few seconds to become available
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/mavros/set_mode service not available, waiting again...')

    def send_mode_command(self, mode):
        """ Sends a command to arm or disarm the ROV """
        req = SetMode.Request()
        req.custom_mode = mode  # True to arm, False to disarm
        self.future = self.mode_client.call_async(req)
        self.future.add_done_callback(self.mode_response_callback)

    def mode_response_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info('Mode change successful')
            else:
                self.get_logger().error('Mode change failed')
        except Exception as e:
            self.get_logger().error('Service call failed: %r' % (e,))

def main(args=None):
    rclpy.init(args=args)
    rov_controller = ModeController()
    rov_controller.send_mode_command('MANUAL') #MANUAL DISARMED
    rclpy.spin_once(rov_controller)
    rov_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()