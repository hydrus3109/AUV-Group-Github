import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import FluidPressure
from mavros_msgs.msg import OverrideRCIn, RCOut
from pymavlink import mavutil
import numpy as np


def set_rc_channel_pwm(mav, channel_id, pwm=1500):
    """Set RC channel pwm value
    Args:
        channel_id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if channel_id < 1 or channel_id > 18:
        print("Channel does not exist.")
        return

    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    mav.mav.rc_channels_override_send(
        mav.target_system,  # target_system
        mav.target_component,  # target_component
        *rc_channel_values
    )


def set_vertical_power(mav, power=0):
    """Set vertical power
    Args:
        power (int, optional): Vertical power value -100-100
    """
    if power < -100 or power > 100:
        print("Power value out of range. Clipping...")
        power = np.clip(power, -100, 100)

    power = int(power)

    set_rc_channel_pwm(mav, 3, 1500 + power * 5)


def press_to_depth(pressure):
    """Convert pressure to depth
    Args:
        pressure (float): Pressure in hPa
    Returns:
        float: Depth in water in meters
    """
    rho = 1029  # density of fresh water in kg/m^3
    g = 9.81  # gravity in m/s^2
    pressure_at_sea_level = 1013.25  # pressure at sea level in hPa
    # multiply by 100 to convert hPa to Pa
    return (pressure - pressure_at_sea_level) * 100 / (rho * g)


class GPTNode(Node):
    def __init__(self):
        super().__init__('gpt_node')

        # Initialize connection to MAVLink
        self.mav = mavutil.mavlink_connection("udpin:0.0.0.0:14550")

        # ROS 2 Publishers and Subscribers
        self.depth_publisher = self.create_publisher(Float64, '/current_depth', 10)
        self.pressure_subscriber = self.create_subscription(FluidPressure, '/pressure', self.pressure_callback, 10)
        self.rc_out_publisher = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.rc_out_subscriber = self.create_subscription(RCOut, '/mavros/rc/out', self.rc_out_callback, 10)

        self.get_logger().info("MavrosNode has been started")

        # Arm the vehicle and set mode
        self.arm_and_set_mode()

        # Ask user for desired depth
        self.desired_depth = float(input("Enter target depth: "))

    def arm_and_set_mode(self):
        self.mav.wait_heartbeat()
        self.mav.arducopter_arm()
        self.mav.motors_armed_wait()

        self.mav.mav.set_mode_send(
            self.mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            19,  # Manual mode
        )

    def pressure_callback(self, msg):
        press_abs = msg.fluid_pressure  # in hPa

        # Calculate depth
        current_depth = press_to_depth(press_abs)
        self.get_logger().info(f"Current Depth: {current_depth}")

        # Publish current depth
        depth_msg = Float64()
        depth_msg.data = current_depth
        self.depth_publisher.publish(depth_msg)

        # Calculate error
        error = self.desired_depth - current_depth
        self.get_logger().info(f"Depth Error: {error}")


    def rc_out_callback(self, msg):
        self.get_logger().info(f"RC Out: {msg.channels}")
        power = -50
        power = np.clip(power, -100, 100)

        power = int(power)

        set_rc_channel_pwm(self.mav, 3, 1500 + power * 5)



def main(args=None):
    rclpy.init(args=args)
    gpt_node = GPTNode()
    rclpy.spin(gpt_node)
    gpt_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()