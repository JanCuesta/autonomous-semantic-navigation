import numpy as np
import rclpy
import math


from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point

from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.time import Time
from rclpy.duration import Duration

from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    ReliabilityPolicy,
    qos_profile_sensor_data
)


class CoverageNode(Node):

    def __init__(self):
        super().__init__('coverage_node')

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            map_qos
        )

        self.camera_subscription = self.create_subscription(
            PointCloud2,
            '/depth_pcl',
            self.depth_callback,
            qos_profile_sensor_data
        )

        self.coverage_publisher = self.create_publisher(
            OccupancyGrid,
            '/semantic/coverage_map',
            map_qos
        )

        self.map_grid = None
        self.coverage_grid = None
        self.map_info = None
        self.map_frame = None
        self.test_painted = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def map_callback(self, msg):

        # Convert the navigation map to a 2D matrix
        self.map_grid = np.array(
            msg.data, dtype=np.int8
        ).reshape(msg.info.height, msg.info.width)

        # Initialize our coverage map only once
        if self.coverage_grid is None:

            # Free cells (0) -> not observed (0)
            # Walls/unknown -> not eligible (-1)
            self.coverage_grid = np.where(
                self.map_grid == 0, 0, -1
            ).astype(np.int8)

        # Create a ROS message for our coverage map
        coverage_msg = OccupancyGrid()

        # Use the same coordinate system and dimensions
        coverage_msg.header = msg.header
        coverage_msg.info = msg.info

        # Convert our 2D matrix back to a 1D array
        coverage_msg.data = self.coverage_grid.flatten().tolist()

        # Publish it
        self.coverage_publisher.publish(coverage_msg)

        self.get_logger().info("Coverage map published!")

        self.map_info = msg.info
        self.map_frame = msg.header.frame_id

    def depth_callback(self, msg):

        if self.map_grid is None:
            return

        if self.test_painted or self.coverage_grid is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.0)
            )

        except TransformException:
            return

        # Only process points when TF is available
        points = point_cloud2.read_points_numpy(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True
        )

        self.get_logger().info(
            f"TF available: {len(points)} points ready"
        )

        # Select one valid camera measurement
        point = points[len(points) // 2]

        # Create a ROS point in the camera frame
        camera_point = PointStamped()
        camera_point.header = msg.header

        camera_point.point.x = float(point[0])
        camera_point.point.y = float(point[1])
        camera_point.point.z = float(point[2])

        # Transform camera coordinates into map coordinates
        map_point = do_transform_point(camera_point, transform)

        self.get_logger().info(
            f"Camera: ({point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f}) "
            f"-> Map: ({map_point.point.x:.2f}, "
            f"{map_point.point.y:.2f}, {map_point.point.z:.2f})"
        )

        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        resolution = self.map_info.resolution

        camera_x = transform.transform.translation.x
        camera_y = transform.transform.translation.y

        paint_x = (camera_x + map_point.point.x) / 2.0
        paint_y = (camera_y + map_point.point.y) / 2.0

        col = math.floor((paint_x - origin_x) / resolution)
        row = math.floor((paint_y - origin_y) / resolution)

        height, width = self.map_grid.shape

        if not (0 <= row < height and 0 <= col < width):
            return

        # Only paint a free cell
        if self.map_grid[row, col] != 0:
            self.get_logger().info("Selected cell is not free")
            return

        # Mark as observed
        self.coverage_grid[row, col] = 100

        # Publish the updated coverage map
        coverage_msg = OccupancyGrid()

        coverage_msg.header.frame_id = self.map_frame
        coverage_msg.header.stamp = msg.header.stamp
        coverage_msg.info = self.map_info
        coverage_msg.data = self.coverage_grid.flatten().tolist()

        self.coverage_publisher.publish(coverage_msg)

        self.test_painted = True

        self.get_logger().info(
            f"Painted coverage cell: row={row}, col={col}"
        )
                
def main(args=None):
    rclpy.init(args=args)
    node = CoverageNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__== '__main__':
    main()


