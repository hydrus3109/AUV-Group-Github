import rclpy
from mavros_msgs.msg import OverrideRCIn

GIMBALL_CHANNEL = 0

def send_camera_commands():
    print("sim_rc")
    rclpy.init_node('blue_rov_camera_node', anonymous=True)
    rc_pub = rclpy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
    command = OverrideRCIn()
    command.channels[GIMBALL_CHANNEL] = 1500 
    rate = rclpy.Rate(10)  # Specify the rate (in Hz) at which to send commands

    print('Node started!')

    while not rclpy.is_shutdown():
        # Send camera tilt up command
        print('Tilting up...')