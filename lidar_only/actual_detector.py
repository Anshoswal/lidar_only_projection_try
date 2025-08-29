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

# import rclpy
# import numpy as np
# from rclpy.node import Node
# from sklearn.cluster import DBSCAN
# from sklearn.linear_model import RANSACRegressor
# from sensor_msgs.msg import PointCloud

# # We need these message definitions to build the PointCloud2 message manually
# from sensor_msgs.msg import PointCloud2, PointField
# from std_msgs.msg import Header

# class ConeDetector(Node):

#     def __init__(self):
#         super().__init__('cone_detector')

#         # Subscriber to PointCloud
#         self.subscriber = self.create_subscription(PointCloud, '/carmaker/pointcloud', self.process_cloud_callback, 10)

#         # Publisher for the projected points to be visualized in RViz2
#         self.publisher_ = self.create_publisher(PointCloud2, '/projected_points', 10)

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

#         pcd = np.array([[p.x, p.y, p.z] for p in msg.points])
        
#         intensities = np.zeros(len(msg.points))
#         for channel in msg.channels:
#             if channel.name.lower() == "intensity":
#                 intensities = np.array(channel.values)
#                 break
        
#         pcd = np.column_stack((pcd, intensities))
#         pcd = self.remove_ground_ransac(pcd)
#         if pcd.shape[0] == 0:
#             return

#         clusters = DBSCAN(min_samples=2, eps=0.5).fit(pcd[:, :2])
#         labels = clusters.labels_
#         unique_labels = set(labels)

#         self.get_logger().info(
#             f"Frame {self.frame_id}: Found {len(unique_labels) - (1 if -1 in unique_labels else 0)} cones/clusters"
#         )
        
#         all_projected_points = []

#         for label in unique_labels:
#             if label == -1:
#                 continue

#             cluster_points = pcd[labels == label]
#             cluster_points_xyz = cluster_points[:, :3]

#             distances = np.linalg.norm(cluster_points_xyz, axis=1)
#             min_index = np.argmin(distances)
#             nearest_point = cluster_points[min_index]
#             p_nearest_xyz = nearest_point[:3]
            
#             a, b = nearest_point[0], nearest_point[1]
#             plane_normal = np.array([a, b, 0])
#             norm_squared = np.dot(plane_normal, plane_normal)
#             if np.isclose(norm_squared, 0):
#                 continue
            
#             vec_to_points = cluster_points_xyz - p_nearest_xyz
#             scales = np.dot(vec_to_points, plane_normal) / norm_squared
#             projected_points_xyz = cluster_points_xyz - scales[:, np.newaxis] * plane_normal
            
#             all_projected_points.append(projected_points_xyz)


#         # ####################################################################
#         # ## START: MANUAL PointCloud2 PUBLISHING LOGIC                     ##
#         # ####################################################################
#         if all_projected_points:
#             # Consolidate all points into a single (N, 3) NumPy array
#             final_projected_cloud = np.vstack(all_projected_points).astype(np.float32)

#             # Create the PointCloud2 message
#             cloud_msg = PointCloud2()
            
#             # 1. Fill the header
#             cloud_msg.header = Header()
#             cloud_msg.header.stamp = self.get_clock().now().to_msg()
#             cloud_msg.header.frame_id = msg.header.frame_id # Use the same frame as the input

#             # 2. Fill the metadata
#             cloud_msg.height = 1
#             cloud_msg.width = final_projected_cloud.shape[0]
#             cloud_msg.is_dense = True # No invalid points
#             cloud_msg.is_bigendian = False # Assuming a little-endian system (most common)

#             # 3. Define the fields (x, y, z)
#             cloud_msg.fields = [
#                 PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
#                 PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
#                 PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
#             ]

#             # 4. Calculate the size of one point and one row
#             # A 32-bit float is 4 bytes. We have 3 floats (x, y, z).
#             cloud_msg.point_step = 4 * 3 # 12 bytes per point
#             cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width

#             # 5. Convert the NumPy data to a raw byte array and assign it
#             cloud_msg.data = final_projected_cloud.tobytes()

#             # Publish the message
#             self.publisher_.publish(cloud_msg)
#             self.get_logger().info(f"Published {len(final_projected_cloud)} projected points to /projected_points")

#         # ####################################################################
#         # ## END: MANUAL PUBLISHING LOGIC                                   ##
#         # ####################################################################

#         self.frame_id += 1

# # (main function remains the same)
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

# # We need these message definitions to build the PointCloud2 message manually
# from sensor_msgs.msg import PointCloud2, PointField
# from std_msgs.msg import Header

# class ConeDetector(Node):

#     def __init__(self):
#         super().__init__('cone_detector')

#         # Subscriber to PointCloud
#         self.subscriber = self.create_subscription(PointCloud, '/carmaker/pointcloud', self.process_cloud_callback, 10)

#         # Publisher for the projected points to be visualized in RViz2
#         self.publisher_ = self.create_publisher(PointCloud2, '/projected_points', 10)
        
#         # <<< NEW >>> Publisher for the calculated centroids
#         self.centroid_publisher_ = self.create_publisher(PointCloud2, '/cluster_centroids', 10)

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

#         pcd = np.array([[p.x, p.y, p.z] for p in msg.points])
        
#         intensities = np.zeros(len(msg.points))
#         for channel in msg.channels:
#             if channel.name.lower() == "intensity":
#                 intensities = np.array(channel.values)
#                 break
        
#         pcd = np.column_stack((pcd, intensities))
#         pcd = self.remove_ground_ransac(pcd)
#         if pcd.shape[0] == 0:
#             return

#         clusters = DBSCAN(min_samples=2, eps=0.5).fit(pcd[:, :2])
#         labels = clusters.labels_
#         unique_labels = set(labels)

#         self.get_logger().info(
#             f"Frame {self.frame_id}: Found {len(unique_labels) - (1 if -1 in unique_labels else 0)} cones/clusters"
#         )
        
#         all_projected_points = []
#         all_centroids = [] # <<< NEW >>> List to store centroids of each cluster

#         for label in unique_labels:
#             if label == -1:
#                 continue

#             cluster_points = pcd[labels == label]
#             cluster_points_xyz = cluster_points[:, :3]

#             distances = np.linalg.norm(cluster_points_xyz, axis=1)
#             min_index = np.argmin(distances)
#             nearest_point = cluster_points[min_index]
#             p_nearest_xyz = nearest_point[:3]
            
#             a, b = nearest_point[0], nearest_point[1]
#             plane_normal = np.array([a, b, 0])
#             norm_squared = np.dot(plane_normal, plane_normal)
#             if np.isclose(norm_squared, 0):
#                 continue
            
#             vec_to_points = cluster_points_xyz - p_nearest_xyz
#             scales = np.dot(vec_to_points, plane_normal) / norm_squared
#             projected_points_xyz = cluster_points_xyz - scales[:, np.newaxis] * plane_normal
            
#             # <<< NEW >>> Calculate the centroid of the projected cluster
#             centroid = np.mean(projected_points_xyz, axis=0)
#             all_centroids.append(centroid)
            
#             all_projected_points.append(projected_points_xyz)


#         # ####################################################################
#         # ## START: PUBLISHING LOGIC FOR PROJECTED POINTS                   ##
#         # ####################################################################
#         if all_projected_points:
#             final_projected_cloud = np.vstack(all_projected_points).astype(np.float32)
#             cloud_msg = PointCloud2()
#             cloud_msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=msg.header.frame_id)
#             cloud_msg.height = 1
#             cloud_msg.width = final_projected_cloud.shape[0]
#             cloud_msg.is_dense = True
#             cloud_msg.is_bigendian = False
#             cloud_msg.fields = [
#                 PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
#                 PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
#                 PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
#             ]
#             cloud_msg.point_step = 12
#             cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
#             cloud_msg.data = final_projected_cloud.tobytes()
#             self.publisher_.publish(cloud_msg)
#             self.get_logger().info(f"Published {len(final_projected_cloud)} projected points to /projected_points")

#         # ####################################################################
#         # ## <<< NEW >>> START: PUBLISHING LOGIC FOR CENTROIDS              ##
#         # ####################################################################
#         if all_centroids:
#             # Consolidate all centroids into a single (N, 3) NumPy array
#             final_centroids = np.vstack(all_centroids).astype(np.float32)

#             # Create the PointCloud2 message for centroids
#             centroid_msg = PointCloud2()
            
#             centroid_msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=msg.header.frame_id)
#             centroid_msg.height = 1
#             centroid_msg.width = final_centroids.shape[0]
#             centroid_msg.is_dense = True
#             centroid_msg.is_bigendian = False
#             centroid_msg.fields = [
#                 PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
#                 PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
#                 PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
#             ]
#             centroid_msg.point_step = 12
#             centroid_msg.row_step = centroid_msg.point_step * centroid_msg.width
#             centroid_msg.data = final_centroids.tobytes()

#             # Publish the centroid message
#             self.centroid_publisher_.publish(centroid_msg)
#             self.get_logger().info(f"Published {len(final_centroids)} centroids to /cluster_centroids")
#         # ####################################################################
#         # ## END: CENTROID PUBLISHING LOGIC                                 ##
#         # ####################################################################

#         self.frame_id += 1

# # (main function remains the same)
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
# import csv
# import os
# from rclpy.node import Node
# from sklearn.cluster import DBSCAN
# from sklearn.linear_model import RANSACRegressor
# from sensor_msgs.msg import PointCloud

# # We need these message definitions to build the PointCloud2 message manually
# from sensor_msgs.msg import PointCloud2, PointField
# from std_msgs.msg import Header

# class ConeDetector(Node):

#     def __init__(self):
#         super().__init__('cone_detector')

#         # Subscriber to PointCloud
#         self.subscriber = self.create_subscription(PointCloud, '/carmaker/pointcloud', self.process_cloud_callback, 10)

#         # Publisher for the projected points to be visualized in RViz2
#         self.publisher_ = self.create_publisher(PointCloud2, '/projected_points', 10)
        
#         # Publisher for the calculated centroids
#         self.centroid_publisher_ = self.create_publisher(PointCloud2, '/cluster_centroids', 10)

#         self.frame_id = 0

#         # <<< NEW >>> CSV file setup
#         self.csv_filename = "centroids_log.csv"
#         # Create a new CSV file every time the node starts
#         with open(self.csv_filename, 'w', newline='') as f:
#             writer = csv.writer(f)
#             # Write the header row
#             writer.writerow(["frame_id", "timestamp_sec", "centroid_x", "centroid_y", "centroid_z"])
        
#         self.get_logger().info(f"The node has been initialised. Logging centroids to {self.csv_filename}")


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

#         pcd = np.array([[p.x, p.y, p.z] for p in msg.points])
        
#         intensities = np.zeros(len(msg.points))
#         for channel in msg.channels:
#             if channel.name.lower() == "intensity":
#                 intensities = np.array(channel.values)
#                 break
        
#         pcd = np.column_stack((pcd, intensities))
#         pcd = self.remove_ground_ransac(pcd)
#         if pcd.shape[0] == 0:
#             return

#         clusters = DBSCAN(min_samples=2, eps=0.5).fit(pcd[:, :2])
#         labels = clusters.labels_
#         unique_labels = set(labels)

#         self.get_logger().info(
#             f"Frame {self.frame_id}: Found {len(unique_labels) - (1 if -1 in unique_labels else 0)} cones/clusters"
#         )
        
#         all_projected_points = []
#         all_centroids = [] 

#         for label in unique_labels:
#             if label == -1:
#                 continue

#             cluster_points = pcd[labels == label]
#             cluster_points_xyz = cluster_points[:, :3]

#             distances = np.linalg.norm(cluster_points_xyz, axis=1)
#             min_index = np.argmin(distances)
#             nearest_point = cluster_points[min_index]
#             p_nearest_xyz = nearest_point[:3]
            
#             a, b = nearest_point[0], nearest_point[1]
#             plane_normal = np.array([a, b, 0])
#             norm_squared = np.dot(plane_normal, plane_normal)
#             if np.isclose(norm_squared, 0):
#                 continue
            
#             vec_to_points = cluster_points_xyz - p_nearest_xyz
#             scales = np.dot(vec_to_points, plane_normal) / norm_squared
#             projected_points_xyz = cluster_points_xyz - scales[:, np.newaxis] * plane_normal
            
#             centroid = np.mean(projected_points_xyz, axis=0)
#             all_centroids.append(centroid)
            
#             all_projected_points.append(projected_points_xyz)

#         # (Publishing logic for projected points and centroids remains the same)
#         # ...

#         # ####################################################################
#         # ## <<< NEW >>> START: CSV WRITING LOGIC                           ##
#         # ####################################################################
#         if all_centroids:
#             # Get the timestamp from the incoming message header for accuracy
#             timestamp = msg.header.stamp
#             timestamp_in_seconds = timestamp.sec + timestamp.nanosec / 1e9

#             # Open the CSV file in 'append' mode to add new rows
#             with open(self.csv_filename, 'a', newline='') as f:
#                 writer = csv.writer(f)
#                 for centroid in all_centroids:
#                     # Prepare the row data
#                     row = [
#                         self.frame_id,
#                         timestamp_in_seconds,
#                         centroid[0],  # centroid_x
#                         centroid[1],  # centroid_y
#                         centroid[2]   # centroid_z
#                     ]
#                     writer.writerow(row)
#             self.get_logger().info(f"Stored {len(all_centroids)} centroids in {self.csv_filename}")
#         # ####################################################################
#         # ## END: CSV WRITING LOGIC                                         ##
#         # ####################################################################


#         # Publishing logic for projected points
#         if all_projected_points:
#             final_projected_cloud = np.vstack(all_projected_points).astype(np.float32)
#             cloud_msg = PointCloud2()
#             cloud_msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=msg.header.frame_id)
#             cloud_msg.height = 1
#             cloud_msg.width = final_projected_cloud.shape[0]
#             cloud_msg.is_dense = True
#             cloud_msg.is_bigendian = False
#             cloud_msg.fields = [
#                 PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
#                 PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
#                 PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
#             ]
#             cloud_msg.point_step = 12
#             cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
#             cloud_msg.data = final_projected_cloud.tobytes()
#             self.publisher_.publish(cloud_msg)

#         # Publishing logic for centroids
#         if all_centroids:
#             final_centroids = np.vstack(all_centroids).astype(np.float32)
#             centroid_msg = PointCloud2()
#             centroid_msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=msg.header.frame_id)
#             centroid_msg.height = 1
#             centroid_msg.width = final_centroids.shape[0]
#             centroid_msg.is_dense = True
#             centroid_msg.is_bigendian = False
#             centroid_msg.fields = [
#                 PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
#                 PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
#                 PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
#             ]
#             centroid_msg.point_step = 12
#             centroid_msg.row_step = centroid_msg.point_step * centroid_msg.width
#             centroid_msg.data = final_centroids.tobytes()
#             self.centroid_publisher_.publish(centroid_msg)

#         self.frame_id += 1


# def main(args=None):
#     rclpy.init(args=args)
#     node = ConeDetector()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass

# import rclpy
# import numpy as np
# import csv
# import os
# from rclpy.node import Node
# from sklearn.cluster import DBSCAN
# from sklearn.linear_model import RANSACRegressor
# from sensor_msgs.msg import PointCloud

# # We need these message definitions to build the PointCloud2 message manually
# from sensor_msgs.msg import PointCloud2, PointField
# from std_msgs.msg import Header

# class ConeDetector(Node):

#     def __init__(self):
#         super().__init__('cone_detector')

#         # Subscribers and Publishers
#         self.subscriber = self.create_subscription(PointCloud, '/carmaker/pointcloud', self.process_cloud_callback, 10)
#         self.publisher_ = self.create_publisher(PointCloud2, '/projected_points', 10)
#         self.centroid_publisher_ = self.create_publisher(PointCloud2, '/cluster_centroids', 10)
#         # <<< NEW >>> Publisher for the transformed points
#         self.transformed_publisher_ = self.create_publisher(PointCloud2, '/transformed_points', 10)

#         self.frame_id = 0

#         # <<< NEW >>> Setup for CSV logging
#         self.csv_filename = "centroids_log.csv"
#         with open(self.csv_filename, 'w', newline='') as f:
#             writer = csv.writer(f)
#             writer.writerow(["frame_id", "timestamp_sec", "centroid_x", "centroid_y", "centroid_z"])
        
#         self.transformed_csv_filename = "transformed_points_log.csv"
#         with open(self.transformed_csv_filename, 'w', newline='') as f:
#             writer = csv.writer(f)
#             writer.writerow(["frame_id", "timestamp_sec", "cluster_label", "t_x", "t_y", "t_z"])
        
#         self.get_logger().info("Node initialised. Logging to CSV files.")

#     def remove_ground_ransac(self, pcd):
#         # (This function remains unchanged)
#         xy = pcd[:, :2]
#         z = pcd[:, 2]
#         if len(xy) < 10: return pcd
#         ransac = RANSACRegressor(residual_threshold=0.03)
#         ransac.fit(xy, z)
#         inlier = ransac.inlier_mask_
#         return pcd[~inlier]

#     def process_cloud_callback(self, msg: PointCloud):
#         if not msg.points:
#             return
        
#         # (Initial point cloud processing remains unchanged)
#         pcd = np.array([[p.x, p.y, p.z] for p in msg.points])
#         intensities = np.zeros(len(msg.points))
#         for channel in msg.channels:
#             if channel.name.lower() == "intensity":
#                 intensities = np.array(channel.values)
#                 break
#         pcd = np.column_stack((pcd, intensities))
#         pcd = self.remove_ground_ransac(pcd)
#         if pcd.shape[0] == 0:
#             return

#         # (Clustering remains unchanged)
#         clusters = DBSCAN(min_samples=2, eps=0.5).fit(pcd[:, :2])
#         labels = clusters.labels_
#         unique_labels = set(labels)
#         self.get_logger().info(
#             f"Frame {self.frame_id}: Found {len(unique_labels) - (1 if -1 in unique_labels else 0)} clusters"
#         )
        
#         # Lists to aggregate data from all clusters in this frame
#         all_projected_points = []
#         all_centroids = []
#         all_transformed_points = [] # <<< NEW >>>
#         csv_rows_transformed = []   # <<< NEW >>>

#         timestamp = msg.header.stamp
#         timestamp_in_seconds = timestamp.sec + timestamp.nanosec / 1e9

#         for label in unique_labels:
#             if label == -1:
#                 continue

#             # (Projection logic is the same)
#             cluster_points = pcd[labels == label]
#             cluster_points_xyz = cluster_points[:, :3]
#             distances = np.linalg.norm(cluster_points_xyz, axis=1)
#             min_index = np.argmin(distances)
#             nearest_point = cluster_points[min_index]
#             p_nearest_xyz = nearest_point[:3]
#             a, b = nearest_point[0], nearest_point[1]
#             plane_normal = np.array([a, b, 0])
#             norm_squared = np.dot(plane_normal, plane_normal)
#             if np.isclose(norm_squared, 0): continue
#             vec_to_points = cluster_points_xyz - p_nearest_xyz
#             scales = np.dot(vec_to_points, plane_normal) / norm_squared
#             projected_points_xyz = cluster_points_xyz - scales[:, np.newaxis] * plane_normal
            
#             # Centroid calculation is the same
#             centroid = np.mean(projected_points_xyz, axis=0)

#             # <<< NEW >>> Perform the coordinate transformation (translation)
#             transformed_points = projected_points_xyz - centroid
            
#             # Aggregate data for publishing and logging
#             all_projected_points.append(projected_points_xyz)
#             all_centroids.append(centroid)
#             all_transformed_points.append(transformed_points)

#             # <<< NEW >>> Prepare rows for the transformed points CSV
#             for point in transformed_points:
#                 csv_rows_transformed.append([
#                     self.frame_id, timestamp_in_seconds, label, 
#                     point[0], point[1], point[2]
#                 ])

#         # ####################################################################
#         # ## CSV WRITING LOGIC                                            ##
#         # ####################################################################
#         if all_centroids:
#             with open(self.csv_filename, 'a', newline='') as f:
#                 writer = csv.writer(f)
#                 for centroid in all_centroids:
#                     writer.writerow([self.frame_id, timestamp_in_seconds, centroid[0], centroid[1], centroid[2]])

#         # <<< NEW >>> Write all transformed points for this frame to CSV
#         if csv_rows_transformed:
#             with open(self.transformed_csv_filename, 'a', newline='') as f:
#                 writer = csv.writer(f)
#                 writer.writerows(csv_rows_transformed)
#             self.get_logger().info(f"Stored {len(csv_rows_transformed)} transformed points in CSV.")

#         # ####################################################################
#         # ## PointCloud2 PUBLISHING LOGIC                                 ##
#         # ####################################################################
#         # (Publishing for projected points and centroids is unchanged)
#         # ...

#         # <<< NEW >>> Publishing logic for transformed points
#         if all_transformed_points:
#             final_transformed_cloud = np.vstack(all_transformed_points).astype(np.float32)
#             trans_msg = PointCloud2()
#             # Use a different frame_id for visualization if needed, or use the original
#             trans_msg.header = Header(stamp=msg.header.stamp, frame_id=msg.header.frame_id)
#             trans_msg.height = 1
#             trans_msg.width = final_transformed_cloud.shape[0]
#             trans_msg.is_dense = True
#             trans_msg.is_bigendian = False
#             trans_msg.fields = [
#                 PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
#                 PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
#                 PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
#             ]
#             trans_msg.point_step = 12
#             trans_msg.row_step = trans_msg.point_step * trans_msg.width
#             trans_msg.data = final_transformed_cloud.tobytes()
#             self.transformed_publisher_.publish(trans_msg)

#         self.frame_id += 1

# # (main function remains the same)
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
import csv
import os
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

        # Subscribers and Publishers
        self.subscriber = self.create_subscription(PointCloud, '/carmaker/pointcloud', self.process_cloud_callback, 10)
        self.publisher_ = self.create_publisher(PointCloud2, '/projected_points', 10)
        self.centroid_publisher_ = self.create_publisher(PointCloud2, '/cluster_centroids', 10)
        self.transformed_publisher_ = self.create_publisher(PointCloud2, '/transformed_points', 10)

        self.frame_id = 0

        # <<< MODIFIED >>> Setup for CSV logging with intensity
        self.csv_filename = "centroids_log.csv"
        with open(self.csv_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            # Add avg_intensity column
            writer.writerow(["frame_id", "timestamp_sec", "centroid_x", "centroid_y", "centroid_z", "avg_intensity"])
        
        self.transformed_csv_filename = "transformed_points_log.csv"
        with open(self.transformed_csv_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            # Add intensity column
            writer.writerow(["frame_id", "timestamp_sec", "cluster_label", "t_x", "t_y", "t_z", "intensity"])
        
        self.get_logger().info("Node initialised. Logging to CSV files.")

    def remove_ground_ransac(self, pcd):
        # (This function remains unchanged)
        xy = pcd[:, :2]
        z = pcd[:, 2]
        if len(xy) < 10: return pcd
        ransac = RANSACRegressor(residual_threshold=0.03)
        ransac.fit(xy, z)
        inlier = ransac.inlier_mask_
        return pcd[~inlier]

    def process_cloud_callback(self, msg: PointCloud):
        if not msg.points:
            return
        
        # Point cloud processing to include intensity (pcd is now N x 4)
        pcd_xyz = np.array([[p.x, p.y, p.z] for p in msg.points])
        intensities = np.zeros((len(msg.points), 1)) # Ensure it's a column vector
        for channel in msg.channels:
            if channel.name.lower() == "intensity":
                intensities = np.array(channel.values).reshape(-1, 1)
                break
        pcd = np.hstack((pcd_xyz, intensities))
        pcd = self.remove_ground_ransac(pcd)
        if pcd.shape[0] == 0:
            return

        # Clustering
        clusters = DBSCAN(min_samples=2, eps=0.5).fit(pcd[:, :2])
        labels = clusters.labels_
        unique_labels = set(labels)
        self.get_logger().info(
            f"Frame {self.frame_id}: Found {len(unique_labels) - (1 if -1 in unique_labels else 0)} clusters"
        )
        
        # Lists to aggregate data
        all_projected_points = []
        all_centroids = []
        all_transformed_points = []
        csv_rows_centroids = []     # <<< NEW >>>
        csv_rows_transformed = []

        timestamp = msg.header.stamp
        timestamp_in_seconds = timestamp.sec + timestamp.nanosec / 1e9

        for label in unique_labels:
            if label == -1:
                continue

            # Extract full cluster data (x, y, z, intensity)
            cluster_points = pcd[labels == label]
            # Separate coordinates from intensity
            cluster_points_xyz = cluster_points[:, :3]
            cluster_intensities = cluster_points[:, 3] # <<< NEW >>>

            # (Projection logic is the same, uses only xyz)
            distances = np.linalg.norm(cluster_points_xyz, axis=1)
            min_index = np.argmin(distances)
            nearest_point = cluster_points_xyz[min_index]
            a, b = nearest_point[0], nearest_point[1]
            plane_normal = np.array([a, b, 0])
            norm_squared = np.dot(plane_normal, plane_normal)
            if np.isclose(norm_squared, 0): continue
            vec_to_points = cluster_points_xyz - nearest_point
            scales = np.dot(vec_to_points, plane_normal) / norm_squared
            projected_points_xyz = cluster_points_xyz - scales[:, np.newaxis] * plane_normal
            
            # Centroid calculation
            centroid = np.mean(projected_points_xyz, axis=0)
            
            # <<< NEW >>> Calculate average intensity for the cluster
            avg_intensity = np.mean(cluster_intensities)

            # Coordinate transformation
            transformed_points = projected_points_xyz - centroid
            
            # Aggregate data
            all_projected_points.append(projected_points_xyz)
            all_centroids.append(centroid)
            all_transformed_points.append(transformed_points)
            
            # <<< MODIFIED >>> Prepare rows for both CSV files
            csv_rows_centroids.append([
                self.frame_id, timestamp_in_seconds,
                centroid[0], centroid[1], centroid[2],
                avg_intensity
            ])
            
            for i, point in enumerate(transformed_points):
                csv_rows_transformed.append([
                    self.frame_id, timestamp_in_seconds, label, 
                    point[0], point[1], point[2],
                    cluster_intensities[i] # Add the original intensity of the point
                ])

        # ####################################################################
        # ## CSV WRITING LOGIC                                            ##
        # ####################################################################
        if csv_rows_centroids:
            with open(self.csv_filename, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerows(csv_rows_centroids)

        if csv_rows_transformed:
            with open(self.transformed_csv_filename, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerows(csv_rows_transformed)
            self.get_logger().info(f"Stored {len(csv_rows_transformed)} points and {len(csv_rows_centroids)} centroids to CSV.")

        # ####################################################################
        # ## PointCloud2 PUBLISHING LOGIC (Unchanged)                     ##
        # ####################################################################
        # (Publishing logic remains the same as it only visualizes coordinates)
#         # Publishing logic for projected points
        if all_projected_points:
            final_projected_cloud = np.vstack(all_projected_points).astype(np.float32)
            cloud_msg = PointCloud2()
            cloud_msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=msg.header.frame_id)
            cloud_msg.height = 1
            cloud_msg.width = final_projected_cloud.shape[0]
            cloud_msg.is_dense = True
            cloud_msg.is_bigendian = False
            cloud_msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
            ]
            cloud_msg.point_step = 12
            cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
            cloud_msg.data = final_projected_cloud.tobytes()
            self.publisher_.publish(cloud_msg)

        # Publishing logic for centroids
        if all_centroids:
            final_centroids = np.vstack(all_centroids).astype(np.float32)
            centroid_msg = PointCloud2()
            centroid_msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=msg.header.frame_id)
            centroid_msg.height = 1
            centroid_msg.width = final_centroids.shape[0]
            centroid_msg.is_dense = True
            centroid_msg.is_bigendian = False
            centroid_msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
            ]
            centroid_msg.point_step = 12
            centroid_msg.row_step = centroid_msg.point_step * centroid_msg.width
            centroid_msg.data = final_centroids.tobytes()
            self.centroid_publisher_.publish(centroid_msg)


        self.frame_id += 1

# (main function and the rest of the publishing logic remain the same)
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