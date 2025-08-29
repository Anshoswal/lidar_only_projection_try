# import rclpy
# import numpy as np
# from rclpy.node import Node
# from sklearn.cluster import DBSCAN
# from sklearn.linear_model import RANSACRegressor
# from sensor_msgs.msg import PointCloud


# class ConeDetector(Node):

#     def __init__(self):
#         super().__init__('cone_detector')

#         # Subscriber to PointCloud
#         self.subscriber = self.create_subscription(PointCloud, '/carmaker/pointcloud', self.process_cloud_callback, 10)

#         self.frame_id = 0
#         self.get_logger().info("The node has been initialised.")

#     def remove_ground_ransac(self, pcd):
#         xy = pcd[:, :2]
#         z = pcd[:, 2]

#         if len(xy) < 10:
#             return pcd

#         ransac = RANSACRegressor(residual_threshold=0.03)
#         ransac.fit(xy, z)

#         inlier = ransac.inlier_mask_
#         return pcd[~inlier]

#     def process_cloud_callback(self, msg: PointCloud):
#         if not msg.points:
#             return

#         # Convert geometry_msgs/Point32[] to numpy
#         pcd = np.array([[p.x, p.y, p.z] for p in msg.points])

#         if msg.channels:
#             channel_names = [ch.name for ch in msg.channels]
#             self.get_logger().info(f"Available channels: {channel_names}")
#         else:
#             self.get_logger().info("No channels available in this PointCloud.")

#         # Default intensity = zeros
#         intensities = np.zeros(len(msg.points))

#         # Search for 'intensity' channel explicitly
#         for channel in msg.channels:
#             if channel.name.lower() == "intensity":
#                 intensities = np.array(channel.values)
#                 self.get_logger().info("Using 'intensity' channel for point data.")
#                 break

#         # Stack intensity (or placeholder zeros) with XYZ
#         pcd = np.column_stack((pcd, intensities))

#         # Ground removal
#         pcd = self.remove_ground_ransac(pcd)
#         if pcd.shape[0] == 0:
#             return

#         # Clustering in XY
#         clusters = DBSCAN(min_samples=2, eps=0.5).fit(pcd[:, :2])
#         labels = clusters.labels_
#         unique_labels = set(labels)

#         self.get_logger().info(
#             f"Frame {self.frame_id}: Found {len(unique_labels) - (1 if -1 in unique_labels else 0)} cones/clusters"
#         )

#         # Report nearest point per cluster
#         for label in unique_labels:
#             if label == -1:
#                 continue

#             cluster_points = pcd[labels == label]

#             distances = np.linalg.norm(cluster_points[:, :3], axis=1)
#             min_index = np.argmin(distances)
#             nearest_point = cluster_points[min_index]
#             min_distance = distances[min_index]

#             self.get_logger().info(
#                 f"  Cluster {label}: Nearest point at distance {min_distance:.3f}m -> "
#                 f"(x={nearest_point[0]:.3f}, y={nearest_point[1]:.3f}, z={nearest_point[2]:.3f}, intensity={nearest_point[3]:.3f})"
#             )

#         self.frame_id += 1


# def main(args=None):
#     rclpy.init(args=args)
#     node = ConeDetector()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# import rclpy
# import numpy as np
# from rclpy.node import Node
# from sklearn.cluster import DBSCAN
# from sklearn.linear_model import RANSACRegressor
# from sensor_msgs.msg import PointCloud


# class ConeDetector(Node):

#     def __init__(self):
#         super().__init__('cone_detector')

#         # Subscriber to PointCloud
#         self.subscriber = self.create_subscription(PointCloud, '/carmaker/pointcloud', self.process_cloud_callback, 10)

#         self.frame_id = 0
#         self.get_logger().info("The node has been initialised.")

#     def remove_ground_ransac(self, pcd):
#         xy = pcd[:, :2]
#         z = pcd[:, 2]

#         if len(xy) < 10:
#             return pcd

#         ransac = RANSACRegressor(residual_threshold=0.03)
#         ransac.fit(xy, z)

#         inlier = ransac.inlier_mask_
#         return pcd[~inlier]

#     def process_cloud_callback(self, msg: PointCloud):
#         if not msg.points:
#             return

#         # Convert geometry_msgs/Point32[] to numpy
#         pcd = np.array([[p.x, p.y, p.z] for p in msg.points])

#         if msg.channels:
#             channel_names = [ch.name for ch in msg.channels]
#         else:
#             self.get_logger().info("No channels available in this PointCloud.")
            
#         # Default intensity = zeros
#         intensities = np.zeros(len(msg.points))

#         # Search for 'intensity' channel explicitly
#         for channel in msg.channels:
#             if channel.name.lower() == "intensity":
#                 intensities = np.array(channel.values)
#                 break
        
#         # Stack intensity (or placeholder zeros) with XYZ
#         pcd = np.column_stack((pcd, intensities))

#         # Ground removal
#         pcd = self.remove_ground_ransac(pcd)
#         if pcd.shape[0] == 0:
#             return

#         # Clustering in XY
#         clusters = DBSCAN(min_samples=2, eps=0.5).fit(pcd[:, :2])
#         labels = clusters.labels_
#         unique_labels = set(labels)

#         self.get_logger().info(
#             f"Frame {self.frame_id}: Found {len(unique_labels) - (1 if -1 in unique_labels else 0)} cones/clusters"
#         )

#         # Report nearest point per cluster and project other points
#         for label in unique_labels:
#             if label == -1:
#                 continue

#             cluster_points = pcd[labels == label]
#             cluster_points_xyz = cluster_points[:, :3] # We only need XYZ for geometry

#             # Find the nearest point (this will define our plane)
#             distances = np.linalg.norm(cluster_points_xyz, axis=1)
#             min_index = np.argmin(distances)
#             nearest_point = cluster_points[min_index]
#             min_distance = distances[min_index]

#             self.get_logger().info(
#                 f"  Cluster {label}: Nearest point at distance {min_distance:.3f}m -> "
#                 f"(x={nearest_point[0]:.3f}, y={nearest_point[1]:.3f}, z={nearest_point[2]:.3f}, intensity={nearest_point[3]:.3f})"
#             )
            
#             # ####################################################################
#             # ## START: NEW PROJECTION LOGIC                                    ##
#             # ####################################################################

#             # Let the nearest point be p_nearest = (a, b, c)
#             a, b = nearest_point[0], nearest_point[1]
#             p_nearest_xyz = nearest_point[:3]

#             # The normal vector to the plane ax + by = k is n = (a, b, 0)
#             plane_normal = np.array([a, b, 0])

#             # Avoid division by zero if the nearest point is at the origin (0,0,z)
#             norm_squared = np.dot(plane_normal, plane_normal)
#             if np.isclose(norm_squared, 0):
#                 self.get_logger().warning(
#                     f"    -> Cluster {label}: Cannot define a plane as the nearest point is at the origin. Skipping projection."
#                 )
#                 continue
            
#             self.get_logger().info(f"    -> Projecting cluster points onto plane defined by normal <{a:.2f}, {b:.2f}, 0.00>")

#             # Vectorized calculation for all points in the cluster
#             # This is much faster than a for-loop in Python
            
#             # Vector from a point on the plane (p_nearest) to each point in the cluster
#             vec_to_points = cluster_points_xyz - p_nearest_xyz
            
#             # Calculate the dot product for all points at once
#             dot_products = np.dot(vec_to_points, plane_normal)
            
#             # Calculate the projection scale factor for each point
#             scales = dot_products / norm_squared
            
#             # Project all points by subtracting the scaled normal vector
#             projected_points_xyz = cluster_points_xyz - scales[:, np.newaxis] * plane_normal

#             # Log the results for verification
#             for i, (orig, proj) in enumerate(zip(cluster_points_xyz, projected_points_xyz)):
#                 self.get_logger().info(
#                     f"      Point {i}: Original (x={orig[0]:.3f}, y={orig[1]:.3f}, z={orig[2]:.3f}) "
#                     f"-> Projected (x'={proj[0]:.3f}, y'={proj[1]:.3f}, z'={proj[2]:.3f})"
#                 )

#             # ####################################################################
#             # ## END: NEW PROJECTION LOGIC                                      ##
#             # ####################################################################

#         self.frame_id += 1


# def main(args=None):
#     rclpy.init(args=args)
#     node = ConeDetector()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
import numpy as np
from rclpy.node import Node
from sklearn.cluster import DBSCAN
from sklearn.linear_model import RANSACRegressor
from sensor_msgs.msg import PointCloud

# We need these message definitions to build the PointCloud2 message manually
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

class ConeDetector(Node):

    def __init__(self):
        super().__init__('cone_detector')

        # Subscriber to PointCloud
        self.subscriber = self.create_subscription(PointCloud, '/carmaker/pointcloud', self.process_cloud_callback, 10)

        # Publisher for the projected points to be visualized in RViz2
        self.publisher_ = self.create_publisher(PointCloud2, '/projected_points', 10)

        self.frame_id = 0
        self.get_logger().info("The node has been initialised.")

    def remove_ground_ransac(self, pcd):
        xy = pcd[:, :2]
        z = pcd[:, 2]

        if len(xy) < 10:
            return pcd

        ransac = RANSACRegressor(residual_threshold=0.03)
        ransac.fit(xy, z)

        inlier = ransac.inlier_mask_
        return pcd[~inlier]

    def process_cloud_callback(self, msg: PointCloud):
        if not msg.points:
            return

        pcd = np.array([[p.x, p.y, p.z] for p in msg.points])
        
        intensities = np.zeros(len(msg.points))
        for channel in msg.channels:
            if channel.name.lower() == "intensity":
                intensities = np.array(channel.values)
                break
        
        pcd = np.column_stack((pcd, intensities))
        pcd = self.remove_ground_ransac(pcd)
        if pcd.shape[0] == 0:
            return

        clusters = DBSCAN(min_samples=2, eps=0.5).fit(pcd[:, :2])
        labels = clusters.labels_
        unique_labels = set(labels)

        self.get_logger().info(
            f"Frame {self.frame_id}: Found {len(unique_labels) - (1 if -1 in unique_labels else 0)} cones/clusters"
        )
        
        all_projected_points = []

        for label in unique_labels:
            if label == -1:
                continue

            cluster_points = pcd[labels == label]
            cluster_points_xyz = cluster_points[:, :3]

            distances = np.linalg.norm(cluster_points_xyz, axis=1)
            min_index = np.argmin(distances)
            nearest_point = cluster_points[min_index]
            p_nearest_xyz = nearest_point[:3]
            
            a, b = nearest_point[0], nearest_point[1]
            plane_normal = np.array([a, b, 0])
            norm_squared = np.dot(plane_normal, plane_normal)
            if np.isclose(norm_squared, 0):
                continue
            
            vec_to_points = cluster_points_xyz - p_nearest_xyz
            scales = np.dot(vec_to_points, plane_normal) / norm_squared
            projected_points_xyz = cluster_points_xyz - scales[:, np.newaxis] * plane_normal
            
            all_projected_points.append(projected_points_xyz)


        # ####################################################################
        # ## START: MANUAL PointCloud2 PUBLISHING LOGIC                     ##
        # ####################################################################
        if all_projected_points:
            # Consolidate all points into a single (N, 3) NumPy array
            final_projected_cloud = np.vstack(all_projected_points).astype(np.float32)

            # Create the PointCloud2 message
            cloud_msg = PointCloud2()
            
            # 1. Fill the header
            cloud_msg.header = Header()
            cloud_msg.header.stamp = self.get_clock().now().to_msg()
            cloud_msg.header.frame_id = msg.header.frame_id # Use the same frame as the input

            # 2. Fill the metadata
            cloud_msg.height = 1
            cloud_msg.width = final_projected_cloud.shape[0]
            cloud_msg.is_dense = True # No invalid points
            cloud_msg.is_bigendian = False # Assuming a little-endian system (most common)

            # 3. Define the fields (x, y, z)
            cloud_msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
            ]

            # 4. Calculate the size of one point and one row
            # A 32-bit float is 4 bytes. We have 3 floats (x, y, z).
            cloud_msg.point_step = 4 * 3 # 12 bytes per point
            cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width

            # 5. Convert the NumPy data to a raw byte array and assign it
            cloud_msg.data = final_projected_cloud.tobytes()

            # Publish the message
            self.publisher_.publish(cloud_msg)
            self.get_logger().info(f"Published {len(final_projected_cloud)} projected points to /projected_points")

        # ####################################################################
        # ## END: MANUAL PUBLISHING LOGIC                                   ##
        # ####################################################################

        self.frame_id += 1

# (main function remains the same)
def main(args=None):
    rclpy.init(args=args)
    node = ConeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()