#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool
import time

class ROVArmer(Node):
    def __init__(self):
        super().__init__('rov_armer')
        """
        Connects to arming service and waits for connection
        """
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/mavros/cmd/arming service not available, waiting again...')

    def send_command(self, arm: bool):
        """ Sends a command to arm or disarm the ROV based on the bool given"""
        req = CommandBool.Request()
        req.value = arm  # True to arm, False to disarm
        self.future = self.arm_client.call_async(req)
        self.future.add_done_callback(self.arm_response_callback)

    def arm_response_callback(self, future):
        """
        gets response from arming and confirms whether or not robot is armed
        """
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Arming/Disarming successful' if future.result().success else 'Arming/Disarming unsuccessful')
            else:
                self.get_logger().error('Arming/Disarming failed with response: %r' % response.result)
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = ROVArmer()
    try:
        node.send_command(True)
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.send_command(False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
