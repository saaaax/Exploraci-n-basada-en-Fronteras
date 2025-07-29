#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import matplotlib.pyplot as plt
import numpy as np

import math

def quaternion_to_yaw(q):
    # q: geometry_msgs.msg.Quaternion
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class PosePlotter(Node):
    def __init__(self):
        super().__init__('pose_plotter')
        self.amcl_poses = []  # (x, y, yaw)
        self.gt_poses = []    # (x, y, yaw)
        #self.ekf_poses = []   # (x, y, yaw)
        
        # Suscriptores
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/ekf_pose',
            self.amcl_callback,
            10
        )
        self.gt_sub = self.create_subscription(
            PoseStamped,  # Tipo correcto para /ground_truth_pose (asumiendo PoseStamped)
            '/ground_truth_pose',
            self.gt_callback,
            10
        )
        # self.ekf_sub = self.create_subscription(
        #     PoseWithCovarianceStamped,
        #     '/ekf_pose',
        #     self.ekf_callback,
        #     10
        # )
        
        # Configuración del gráfico
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.set_title('AMCL vs Ground Truth Pose')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.grid(True)
        self.ax.legend()

    def amcl_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.amcl_poses.append((x, y, yaw))
        self.update_plot()

    # def ekf_callback(self, msg):
    #     x = msg.pose.pose.position.x
    #     y = msg.pose.pose.position.y
    #     yaw = quaternion_to_yaw(msg.pose.pose.orientation)
    #     self.ekf_poses.append((x, y, yaw))
    #     self.update_plot()

    def gt_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.orientation)
        self.gt_poses.append((x, y, yaw))
        self.update_plot()

    def update_plot(self):
        if len(self.amcl_poses) > 0 and len(self.gt_poses) > 0:
            self.ax.clear()
            # Extraer coordenadas y ángulos
            amcl_x, amcl_y, amcl_yaw = zip(*self.amcl_poses)
            gt_x, gt_y, gt_yaw = zip(*self.gt_poses)
            # ekf_x, ekf_y, ekf_yaw = zip(*self.ekf_poses)
            # Trayectorias
            self.ax.plot(amcl_x, amcl_y, 'b-', label='AMCL Pose')
            self.ax.plot(gt_x, gt_y, 'r-', label='Ground Truth Pose')
            # self.ax.plot(ekf_x, ekf_y, 'g-', label='EKF Pose')
            # Flechas de dirección (solo cada N puntos para no saturar)
            N = 10
            self.ax.quiver(amcl_x[::N], amcl_y[::N], 
                           np.cos(amcl_yaw[::N]), np.sin(amcl_yaw[::N]), 
                           color='b', scale=20, width=0.005)
            self.ax.quiver(gt_x[::N], gt_y[::N], 
                           np.cos(gt_yaw[::N]), np.sin(gt_yaw[::N]), 
                           color='r', scale=20, width=0.005)
            # self.ax.quiver(ekf_x[::N], ekf_y[::N], 
            #                np.cos(ekf_yaw[::N]), np.sin(ekf_yaw[::N]), 
            #                color='g', scale=20, width=0.005)
            self.ax.legend()
            self.ax.grid(True)
            all_x = amcl_x + gt_x #+ ekf_x
            all_y = amcl_y + gt_y #+ ekf_y
            if all_x and all_y:
                self.ax.set_xlim(min(all_x) - 1, max(all_x) + 1)
                self.ax.set_ylim(min(all_y) - 1, max(all_y) + 1)
            plt.draw()
            plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    pose_plotter = PosePlotter()
    
    try:
        rclpy.spin(pose_plotter)
    except KeyboardInterrupt:
        pass
    
    # Guardar y mostrar el gráfico al finalizar
    plt.savefig('amcl_vs_ground_truth.png')
    print("Gráfico guardado. Cierra la ventana para finalizar.")
    plt.ioff()
    plt.show()  # Bloquea hasta que el usuario cierre la ventana
    
    pose_plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
