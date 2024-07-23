#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Pose2D
import matplotlib.patches as patches
import math
import matplotlib.pyplot as plt

import numpy as np
class physicsSubscriber(Node):
    def __init__(self):
        super().__init__("physicssubscriber")
                
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.subscription = self.create_subscription(
            Pose2D,
            '/physics/pose2d',
            self.callback,
            qos_profile
        )
        self.subscription
        self.get_logger().info("starting subscriber node")

    def callback(self, msg):
        self.get_logger().info("info receieved")
        self.sim_auv_motion(msg.x, msg.y, msg.theta)
        self.get_logger().info("task completed")
    def calc_auv2_accel(self, T, alpha, theta, mass=100):
        """given a np array of forces in newtons, an alpha offset of the motor in radians, and a theta offset of the vehicle in radians, and the mass of the vehicle  in kgs, reutrns the a 2d array of the vehicle's accleration in m/s^2"""
        if mass <= 0:
            raise ZeroDivisionError("can't divide by 0/negative mass")
        Thorz = T * math.cos(alpha)
        Tvert = T * math.sin(alpha)
        tothorizontal = Thorz[0] + Thorz[1] - Thorz[2] - Thorz[3]
        totvertical = Tvert[0] - Tvert[1] - Tvert[2] + Tvert[3]
        accel = np.array([tothorizontal, totvertical])
        rotate = np.array(
            [[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]]
        )
        return np.dot(rotate, accel) / mass



    def calc_auv2_ang_accel(self, T, alpha, L, l, inertia=100):
        Thorz = T * np.sin(alpha)
        Tvert = T * np.cos(alpha)
        torques = l * Thorz + L * Tvert
        return (torques[0] - torques[1] + torques[2] - torques[3])/inertia


    def calc_auv_motion(self, T, alpha, L, l, lmass, inertia, dt=0.1, t_final=10, x0=0, y0=0, theta=0):
        count = 0
        curang = theta
        timestamps = np.array([0])
        x = np.array([x0])
        y = np.array([y0])
        theta = np.array([theta])
        v = np.array([0, 0])
        omega = np.array([0])
        a = np.array([0, 0])
        while count <= t_final:
            count += dt
            timestamps = np.append(timestamps, count)
            accel = self.calc_auv2_accel(T, alpha, curang, lmass)
            a = np.append(a, accel)
            angaccel = self.calc_auv2_ang_accel(T, alpha, L, l, inertia)
            newv = v[-1] + accel * dt
            newomega = omega[-1] + angaccel * dt
            v = np.append(v, newv)
            omega = np.append(omega, newomega)
            newtheta = theta[-1] + omega[-2]* dt + 0.5 * angaccel * dt * dt
            theta = np.append(theta, newtheta)
            curang = newtheta
            newcoords = np.array([x[-1], y[-1]]) + v[-2]* dt + 0.5 * accel * dt * dt
            x = np.append(x, newcoords[0])
            y = np.append(y, newcoords[1])
        return timestamps, x, y, theta, v, omega, a
    def sim_auv_motion(self, x0,y0,theta0):
        T = np.array([2.0,1.0,2.0,1.0])
        alpha = math.pi/4
        t_final = 10
        dt = 0.1
        l = 0.2
        L = 0.3
        mass = 1.0
        timestamps, x, y, theta, v, omega, a = self.calc_auv_motion(T, alpha, L, l, mass, 1, dt, t_final, x0, y0, theta0)
        fig = plt.figure(figsize=(10, 10))
        plt.plot()
        plt.gca().set_aspect("equal", adjustable="box")

        for i in range(int(t_final / dt)):
            plt.gca().add_patch(
                patches.Rectangle(
                    (x[i]-L, y[i]-l),
                    2 * l,
                    2 * L,
                    angle=theta[i],
                    edgecolor="red",
                    facecolor="none",
                    rotation_point = "center",
                    lw=1,
                )
            )
            
        plt.savefig("/home/aidan/auvc_ws/src/intro_to_ros/intro_to_ros/plot.png")
        
def main(args=None):
    rclpy.init(args=args)
    node = physicsSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=="__main__":
    main()