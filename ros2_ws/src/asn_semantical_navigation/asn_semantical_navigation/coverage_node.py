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
        self.last_processed_stamp = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.declare_parameter('processing_hz', 2.0)

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
        self.map_info = msg.info
        self.map_frame = msg.header.frame_id

        # Convert our 2D matrix back to a 1D array
        coverage_msg.data = self.coverage_grid.flatten().tolist()

        # Publish it
        self.coverage_publisher.publish(coverage_msg)

        self.get_logger().info("Coverage map published!")

    def depth_callback(self, msg):

        # Check that the navigation and coverage maps exist
        if self.map_grid is None or self.coverage_grid is None:
            return

        # Process at most once every 0.5 seconds of simulation time
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_processed_stamp is not None:
            elapsed = stamp - self.last_processed_stamp
            
            processing_hz = self.get_parameter('processing_hz').value
            processing_interval = 1.0 / processing_hz

            if 0 <= elapsed < processing_interval:
                return

        # Get transformation from camera frame to map frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.0)
            )

        except TransformException:
            return

        points = point_cloud2.read_points_numpy(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=False
        )

        # Identify measurements with valid XYZ coordinates
        valid_mask = np.isfinite(points).all(axis=1)

        valid_count = np.count_nonzero(valid_mask)
        total_count = len(points)

        self.get_logger().info(
            f"Valid depth: {valid_count}/{total_count} "
            f"({100 * valid_count / total_count:.1f}%)"
        )

        # Keep only valid measurements
        points = points[valid_mask]

        # Maximum camera measurement range
        max_depth_m = 3.0

        # Calculate 3D distance from camera to each measured point
        distances = np.linalg.norm(points, axis=1)

        # Keep only points within 3 meters
        range_mask = distances <= max_depth_m

        points = points[range_mask]

        self.get_logger().info(
            f"Points within {max_depth_m}m: {len(points)}"
        )

        if len(points) == 0:
            return
        
        # Select 1000 random camera measurements
        num_samples = min(1000, len(points))

        indices = np.random.choice(
            len(points),
            size=num_samples,
            replace=False
        )

        sampled_points = points[indices]

        # Camera position in map coordinates
        camera_x = transform.transform.translation.x
        camera_y = transform.transform.translation.y

        # Reuse the same ROS message for each point
        camera_point = PointStamped()
        camera_point.header = msg.header

        total_painted = 0

        floor_hits = 0

        # Process each selected camera measurement
        for point in sampled_points:

            # Skip invalid coordinates
            if not np.isfinite(point).all():
                continue

            # Set point coordinates in camera frame
            camera_point.point.x = float(point[0])
            camera_point.point.y = float(point[1])
            camera_point.point.z = float(point[2])

            # Transform point into map coordinates
            map_point = do_transform_point(camera_point, transform)
            # Approximate floor detection (assuming floor z = 0 in map frame)
            if abs(map_point.point.z) < 0.10:
                floor_hits += 1

            # Actual 3D distance from camera to measured point
            ray_distance = float(np.linalg.norm(point))

            painted = self.paint_ray(
                camera_x,
                camera_y,
                map_point.point.x,
                map_point.point.y,
                ray_distance
            )

            total_painted += painted

        # Record when we processed this cloud
        self.last_processed_stamp = stamp

        # Publish the coverage map after processing all rays
        if total_painted > 0:

            coverage_msg = OccupancyGrid()
            coverage_msg.header.frame_id = self.map_frame
            coverage_msg.header.stamp = msg.header.stamp
            coverage_msg.info = self.map_info

            coverage_msg.data = self.coverage_grid.flatten().tolist()

            self.coverage_publisher.publish(coverage_msg)

            self.get_logger().info(
                f"Processed {len(sampled_points)} rays, "
                f"floor hits: {floor_hits}, "
                f"painted {total_painted} new cells"
            )


    def world_to_cell(self, x, y):
        resolution = self.map_info.resolution
        origin = self.map_info.origin.position

        col = math.floor((x - origin.x) / resolution)
        row = math.floor((y - origin.y) / resolution)

        return row, col

    def paint_ray(self, start_x, start_y, end_x, end_y, ray_distance):

        # Bresenham ray-tracing algorithm
        row0, col0 = self.world_to_cell(start_x, start_y)
        row1, col1 = self.world_to_cell(end_x, end_y)

        dx = abs(col1 - col0)
        dy = -abs(row1 - row0)

        sx = 1 if col0 < col1 else -1
        sy = 1 if row0 < row1 else -1

        error = dx + dy

        row, col = row0, col0
        updated = 0

        height, width = self.map_grid.shape

        resolution = self.map_info.resolution
        origin = self.map_info.origin.position

        # Total horizontal length of the projected ray
        horizontal_distance = math.hypot(
            end_x - start_x,
            end_y - start_y
        )

        for _ in range(max(height, width) + 1):

            # Stop at map boundaries
            if not (0 <= row < height and 0 <= col < width):
                break

            # Stop at obstacles or unknown cells
            if self.map_grid[row, col] != 0:
                break

            # Position of current cell center in map coordinates
            cell_x = origin.x + (col + 0.5) * resolution
            cell_y = origin.y + (row + 0.5) * resolution

            # Estimate progress along the 3D camera ray
            if horizontal_distance > 1e-6:
                progress = math.hypot(
                    cell_x - start_x,
                    cell_y - start_y
                ) / horizontal_distance

                progress = min(1.0, progress)
            else:
                progress = 1.0

            # Approximate 3D distance to this cell along the ray
            distance = progress * ray_distance

            # Distance-based confidence
            confidence = round(100 * math.exp(-distance / 1.5))
            confidence = max(1, min(100, confidence))

            # Keep the highest confidence observed so far
            if confidence > self.coverage_grid[row, col]:
                self.coverage_grid[row, col] = confidence
                updated += 1

            # Stop when the endpoint is reached
            if row == row1 and col == col1:
                break

            # Bresenham: move to the next cell
            e2 = 2 * error

            if e2 >= dy:
                error += dy
                col += sx

            if e2 <= dx:
                error += dx
                row += sy

        return updated   
                
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


