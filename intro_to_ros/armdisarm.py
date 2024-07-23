#!/usr/bin/env python3

#~/ardupilot/Tools/autotest/sim_vehicle.py --vehicle=ArduSub --aircraft="bwsibot" -L RATBeach --out=udp:YOUR_COMPUTER_IP:14550
#ros2 launch mavros apm.launch fcu_url:=udp://192.168.2.2:14550@14555 gcs_url:=udp://:14550@YOUR_COMPUTER_IP:14550 tgt_system:=1 tgt_component:=1 system_id:=255 component_id:=240

#cd ~/auvc_ws
#colcon build --packages-select intro_to_ros --symlink-install
#source ~/auvc_ws/install/setup.zs  h

#ros2 topic list
#ros2 topic type /your/topic
#ro2 topic echo /your/topic :)))))
#ros2  interface show your_msg_library/msg/YourMessageType

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

from time import sleep

class ArmingDisarmingNode(Node):
    done = False

    def __init__(self):
        super().__init__("arming_disarming_node")

        self.arming_client = self.create_client(SetBool, "bluerov2/arming")

        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("BlueROV2 arming service not available, waiting...")

    def arm(self):
        # Arm
        arm_future = self._arm()
        rclpy.spin_until_future_complete(self, arm_future)
        self.get_logger().info("Armed!")
        self.done = True

    def disarm(self):
        # Disarm
        disarm_future = self._disarm()
        rclpy.spin_until_future_complete(self, disarm_future)
        self.get_logger().info("Disarmed!")

    def _arm(self):
        self.get_logger().info(f"Arming...")
        future = self.arming_client.call_async(SetBool.Request(data=True))
        return future

    def _disarm(self):
        self.get_logger().info(f"Disarming...")
        future = self.arming_client.call_async(SetBool.Request(data=False))
        return future

    def destroy_node(self):
        disarm_future = self._disarm()
        rclpy.spin_until_future_complete(self, disarm_future)
        self.get_logger().info("Disarmed before shutting down the node")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    armingNode = ArmingDisarmingNode()
    
    try:
        while rclpy.ok():
            armingNode.arm()
            rclpy.spin_once(armingNode)
            if armingNode.done:
                break
        
        sleep(120)
    except KeyboardInterrupt:
        pass
    finally:
        armingNode.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()