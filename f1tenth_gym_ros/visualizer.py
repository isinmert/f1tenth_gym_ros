import rclpy 
from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

class Visualizer(Node):
    def __init__(self):
        super().__init__("visualizer")

        self.box_publisher = self.create_publisher(Marker, "/racecar_box", 10)
        self.timer = self.create_timer(0.1, self._publish_box_callback)

        self.racecar_pose_subscriber = self.create_subscription(
            Odometry,  "/ego_racecar/odom", self._odom_callback, 10
            )

        self.last_odomerty_msg = Odometry()
    
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

        # orientation
        marker_msg.pose.orientation \
            = self.last_odomerty_msg.pose.pose.orientation

        # size 
        marker_msg.scale.x = 0.3302
        marker_msg.scale.y = 0.2032
        marker_msg.scale.z = 0.1

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
    