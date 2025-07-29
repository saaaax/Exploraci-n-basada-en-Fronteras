#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from sklearn.cluster import DBSCAN
import numpy as np

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)

        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.map_info = None

    def map_callback(self, msg):
        self.map_info = msg.info
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape((height, width))

        frontiers = self.detect_frontiers(data)
        if not frontiers:
            self.get_logger().info('No frontiers found.')
            return

        centroids = self.cluster_frontiers(frontiers)
        if centroids:
            target = centroids[0]  # elegir el primero (mejor: más cercano al robot)
            self.publish_goal(target[0], target[1])

    def detect_frontiers(self, map_data):
        frontiers = []
        height, width = map_data.shape

        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if map_data[y, x] == 0:
                    neighborhood = map_data[y-1:y+2, x-1:x+2]
                    if -1 in neighborhood:
                        frontiers.append((x, y))
        return frontiers

    def cluster_frontiers(self, frontiers):
        if len(frontiers) < 5:
            return frontiers

        clustering = DBSCAN(eps=3, min_samples=3).fit(frontiers)
        centroids = []

        for label in set(clustering.labels_):
            if label == -1:
                continue
            points = [frontiers[i] for i in range(len(frontiers)) if clustering.labels_[i] == label]
            cx = int(np.mean([p[0] for p in points]))
            cy = int(np.mean([p[1] for p in points]))
            centroids.append((cx, cy))

        return centroids

    def grid_to_world(self, x, y):
        res = self.map_info.resolution
        origin = self.map_info.origin.position
        wx = origin.x + (x + 0.5) * res
        wy = origin.y + (y + 0.5) * res
        return wx, wy

    def publish_goal(self, x, y):
        wx, wy = self.grid_to_world(x, y)
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = wx
        goal.pose.position.y = wy
        goal.pose.orientation.w = 1.0

        self.goal_publisher.publish(goal)
        self.get_logger().info(f'Published goal to: ({wx:.2f}, {wy:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()