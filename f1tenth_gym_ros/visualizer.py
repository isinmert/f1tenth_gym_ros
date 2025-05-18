import rclpy 
from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory

import os
import csv
import yaml

from typing import List, Tuple

RACECAR_WIDTH = 0.2032
RACECAR_HEIGHT = 0.1
RACECAR_WHEELBASE = 0.3302
RACECAR_GROUND_OFFSET = 0.04

class Visualizer(Node):
    def __init__(self):
        super().__init__("visualizer")

        self.box_publisher = self.create_publisher(Marker, "/racecar_box", 10)
        self.timer = self.create_timer(0.1, self._publish_box_callback)

        self.centerline_timer = self.create_timer(
            1.0, self._publish_centerline_callback)

        self.centerline_publisher = self.create_publisher(Marker,
                                                          "/centerline", 10)

        self.racecar_pose_subscriber = self.create_subscription(
            Odometry,  "/ego_racecar/odom", self._odom_callback, 10
            )
        

        config = os.path.join(
            get_package_share_directory('f1tenth_gym_ros'),
            'config',
            'sim.yaml'
            )
        config_dict = yaml.safe_load(open(config, 'r'))
        try:
            map_name = config_dict['bridge']['ros__parameters']['map_name']
        except:
            map_name = None
        

        if map_name:
            centerline_csv_path = os.path.join(
                get_package_share_directory('f1tenth_gym_ros'), 
                'maps', 
                map_name + '.csv'
            )
        else: 
            centerline_csv_path = None

        self.centerline_points : List[Tuple[float, float]] = []
        try:
            with open(centerline_csv_path, newline='') as f:
                reader = csv.reader(f)
                for row in reader:
                    px = float(row[0])
                    py = float(row[1])
                    self.centerline_points.append((px, py))
        except:
            self.get_logger().warning(
                f"Can't read centerline file at {centerline_csv_path}")

        for point in self.centerline_points:
            px = point[0]
            py = point[1]
            self.get_logger().info(f"px:{px}, py:{py}")
               

        self.last_odomerty_msg = Odometry()

    def _publish_centerline_callback(self) -> None:
        
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "polyline"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # add centerline points
        marker.points = [Point(x=x, y=y, z=0.0) for x, y in self.centerline_points]

        marker.scale.x = 0.2 
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.centerline_publisher.publish(marker)
        return
    
    def _odom_callback(self, msg:Odometry) -> None:
        self.last_odomerty_msg = msg

    def _publish_box_callback(self) -> None:
        marker_msg = Marker()

        marker_msg.action = Marker.ADD
        marker_msg.type = Marker.CUBE
        marker_msg.header.frame_id = "map"
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.ns = "cube"
        marker_msg.id = 0
        marker_msg.type = Marker.CUBE
        marker_msg.action = Marker.ADD

        # position
        marker_msg.pose.position = self.last_odomerty_msg.pose.pose.position
        marker_msg.pose.position.z += RACECAR_GROUND_OFFSET

        # orientation
        marker_msg.pose.orientation \
            = self.last_odomerty_msg.pose.pose.orientation

        # size 
        marker_msg.scale.x = RACECAR_WHEELBASE
        marker_msg.scale.y = RACECAR_WIDTH
        marker_msg.scale.z = RACECAR_HEIGHT

        # color
        marker_msg.color.a = 0.5
        marker_msg.color.b = 1.0
        marker_msg.color.g = 0.0
        marker_msg.color.r = 0.0

        # publish the marker
        self.box_publisher.publish(marker_msg)

        self.box_publisher
        return

def main(args=None):
    rclpy.init(args=args)
    node = Visualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    