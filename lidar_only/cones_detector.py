# # # import rclpy
# # # from rclpy.node import Node
# # # from rclpy.duration import Duration
# # # from sklearn.cluster import DBSCAN
# # # from sklearn.linear_model import RANSACRegressor
# # # import numpy as np
# # # from sensor_msgs.msg import PointCloud
# # # from visualization_msgs.msg import Marker, MarkerArray
# # # from geometry_msgs.msg import Point
# # # import csv


# # # class ConesDetector(Node):
# # #     def __init__(self):
# # #         super().__init__('cones_detector')
# # #         self.subscriber = self.create_subscription(
# # #             PointCloud, '/carmaker/pointcloud', self.cluster, 10
# # #         )
# # #         self.publisher = self.create_publisher(MarkerArray, '/visualise', 10)

# # #         # Counters
# # #         self.frame_count = 0
# # #         self.marker_id = 0

# # #         # CSV setup for grid visualization
# # #         self.grid_csv_file = open("triangle_grids.csv", "w", newline="")
# # #         self.grid_csv_writer = csv.writer(self.grid_csv_file)
# # #         self.GRID_SIZE = 30 # Defines the resolution of the grid (30x30)

# # #     def __del__(self):
# # #         """Ensure CSV file is closed on exit"""
# # #         if hasattr(self, "grid_csv_file") and not self.grid_csv_file.closed:
# # #             self.grid_csv_file.close()

# # #     def remove_ground_ransac(self, pc_data):
# # #         xyz_data = pc_data[:, :3]
# # #         if len(xyz_data) < 10: return pc_data
# # #         try:
# # #             if xyz_data.shape[0] < 3: return np.array([]) 
# # #             ransac = RANSACRegressor(residual_threshold=0.05)
# # #             ransac.fit(xyz_data[:, :2], xyz_data[:, 2])
# # #             return pc_data[~ransac.inlier_mask_]
# # #         except ValueError:
# # #             return np.array([])


# # #     def cluster(self, msg):
# # #         self.frame_count += 1
# # #         if not msg.points: return

# # #         pc_data_xyz = np.array([[p.x, p.y, p.z] for p in msg.points])
# # #         if msg.channels and msg.channels[0].name.lower() == 'intensity':
# # #             intensities = np.array(msg.channels[0].values).reshape(-1, 1)
# # #             pc_data = np.hstack((pc_data_xyz, intensities))
# # #         else:
# # #             pc_data = np.hstack((pc_data_xyz, np.zeros((pc_data_xyz.shape[0], 1))))
        
# # #         pc_data = self.remove_ground_ransac(pc_data)
# # #         if pc_data.shape[0] < 5: return

# # #         labels = DBSCAN(eps=0.5, min_samples=5).fit_predict(pc_data[:, :2])
# # #         unique_labels = set(labels)
# # #         if -1 in unique_labels: unique_labels.remove(-1)

# # #         markerarr = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
# # #         self.publisher.publish(markerarr)
# # #         markerarr = MarkerArray()
# # #         self.marker_id = 0

# # #         if not unique_labels: return
# # #         print(f"\n--- Frame: {self.frame_count}, Found {len(unique_labels)} clusters ---")
        
# # #         for label in unique_labels:
# # #             cluster_points = pc_data[labels == label]
# # #             if cluster_points.shape[0] < 5: continue
            
# # #             sizes = np.ptp(cluster_points[:, :3], axis=0)
# # #             if sizes[0] > 1.0 or sizes[1] > 1.0:
# # #                 print(f"    -> Discarding cluster {label} due to large size.")
# # #                 continue

# # #             min_z = np.min(cluster_points[:, 2])
# # #             base_points_mask = cluster_points[:, 2] <= min_z + 0.05
# # #             base_points = cluster_points[base_points_mask]
            
# # #             if base_points.shape[0] == 0: continue

# # #             squared_distances = base_points[:, 0]**2 + base_points[:, 1]**2
# # #             closest_point = base_points[np.argmin(squared_distances), :3]
            
# # #             plane_normal = np.array([closest_point[0], closest_point[1], 0.0])
# # #             norm = np.linalg.norm(plane_normal)
# # #             if norm < 1e-6: continue
# # #             plane_normal /= norm

# # #             vectors_to_project = cluster_points[:, :3] - closest_point
# # #             distances_from_plane = np.dot(vectors_to_project, plane_normal)
# # #             projected_points_xyz = cluster_points[:, :3] - np.outer(distances_from_plane, plane_normal)
            
# # #             self.create_and_save_grid(projected_points_xyz, cluster_points, closest_point, plane_normal, label)

# # #             # --- Visualization ---
# # #             proj_marker = Marker()
# # #             proj_marker.header.frame_id = "Lidar_F"
# # #             proj_marker.header.stamp = self.get_clock().now().to_msg()
# # #             proj_marker.ns = "projections"
# # #             proj_marker.id = self.marker_id
# # #             proj_marker.type = Marker.POINTS
# # #             proj_marker.action = Marker.ADD
# # #             proj_marker.pose.orientation.w = 1.0
# # #             proj_marker.scale.x = 0.03
# # #             proj_marker.scale.y = 0.03
# # #             proj_marker.color.r, proj_marker.color.g, proj_marker.color.b = 0.0, 0.5, 1.0
# # #             proj_marker.color.a = 1.0
# # #             proj_marker.lifetime = Duration(seconds=0.2).to_msg()
# # #             proj_marker.points = [Point(x=p[0], y=p[1], z=p[2]) for p in projected_points_xyz]
# # #             markerarr.markers.append(proj_marker)
# # #             self.marker_id += 1
        
# # #         self.grid_csv_file.flush()
# # #         if markerarr.markers:
# # #             self.publisher.publish(markerarr)

# # #     # --- MODIFIED: Function updated to trim empty rows and columns ---
# # #     def create_and_save_grid(self, projected_points, original_points, plane_origin, plane_normal, cluster_label):
# # #         """Converts projected points into a trimmed 2D grid of intensities and saves to CSV."""
        
# # #         z_coords = projected_points[:, 2]
# # #         tangent_vector = np.array([-plane_normal[1], plane_normal[0], 0])
# # #         h_coords = np.dot(projected_points - plane_origin, tangent_vector)
# # #         intensities = original_points[:, 3]

# # #         z_min, z_max = np.min(z_coords), np.max(z_coords)
# # #         h_min, h_max = np.min(h_coords), np.max(h_coords)
# # #         range_z = z_max - z_min
# # #         range_h = h_max - h_min
        
# # #         if range_z < 1e-6 or range_h < 1e-6: return

# # #         max_range = max(range_z, range_h)
# # #         scale = (self.GRID_SIZE - 1) / max_range

# # #         intensity_sum_grid = np.zeros((self.GRID_SIZE, self.GRID_SIZE), dtype=float)
# # #         point_count_grid = np.zeros((self.GRID_SIZE, self.GRID_SIZE), dtype=int)
        
# # #         for i in range(len(h_coords)):
# # #             col = int((h_coords[i] - h_min) * scale)
# # #             row = int((z_coords[i] - z_min) * scale)
# # #             grid_row_idx = self.GRID_SIZE - 1 - row
# # #             intensity_sum_grid[grid_row_idx, col] += intensities[i]
# # #             point_count_grid[grid_row_idx, col] += 1

# # #         final_grid = np.full((self.GRID_SIZE, self.GRID_SIZE), '0', dtype=object)
        
# # #         non_empty_cells = point_count_grid > 0
# # #         average_intensities = np.divide(intensity_sum_grid, point_count_grid, where=non_empty_cells)
# # #         final_grid[non_empty_cells] = np.char.mod('%.1f', average_intensities[non_empty_cells])

# # #         # --- NEW: Trim empty rows and columns from the grid ---
# # #         # Find rows and columns that contain at least one non-'0' value
# # #         row_mask = (final_grid != '0').any(axis=1)
# # #         col_mask = (final_grid != '0').any(axis=0)

# # #         # Apply the masks to get the trimmed grid
# # #         if np.any(row_mask) and np.any(col_mask):
# # #             trimmed_grid = final_grid[row_mask][:, col_mask]
# # #         else:
# # #             # Handle case where grid is completely empty
# # #             trimmed_grid = np.array([['0']])

# # #         # Write the trimmed grid to the CSV file
# # #         self.grid_csv_writer.writerow([])
# # #         self.grid_csv_writer.writerow([f"Frame {self.frame_count}", f"Cluster {cluster_label}"])
# # #         for grid_row in trimmed_grid:
# # #             self.grid_csv_writer.writerow(grid_row)


# # # def main(args=None):
# # #     rclpy.init(args=args)
# # #     node = ConesDetector()
# # #     try:
# # #         rclpy.spin(node)
# # #     except KeyboardInterrupt:
# # #         pass
# # #     finally:
# # #         node.destroy_node()
# # #         rclpy.shutdown()

# # # if __name__ == '__main__':
# # #     main()

# # # # import rclpy
# # # # from rclpy.node import Node
# # # # from rclpy.duration import Duration
# # # # from sklearn.cluster import DBSCAN
# # # # from sklearn.linear_model import RANSACRegressor
# # # # import numpy as np
# # # # from sensor_msgs.msg import PointCloud2
# # # # from sensor_msgs_py import point_cloud2
# # # # from visualization_msgs.msg import Marker, MarkerArray
# # # # from geometry_msgs.msg import Point
# # # # import csv
# # # # import threading


# # # # class ConesDetector(Node):
# # # #     def __init__(self):
# # # #         super().__init__('cones_detector')
# # # #         self.subscriber = self.create_subscription(
# # # #             PointCloud2, '/carmaker/pointcloud', self.cluster, 10
# # # #         )
# # # #         self.publisher = self.create_publisher(MarkerArray, '/visualise', 10)

# # # #         # Counters
# # # #         self.frame_count = 0
# # # #         self.marker_id = 0

# # # #         # CSV setup
# # # #         self.grid_csv_file = open("triangle_grids.csv", "w", newline="")
# # # #         self.grid_csv_writer = csv.writer(self.grid_csv_file)
# # # #         self.csv_lock = threading.Lock()  # prevent race conditions
# # # #         self.GRID_SIZE = 30  # Defines resolution of grid (30x30)

# # # #     def __del__(self):
# # # #         """Ensure CSV file is closed on exit"""
# # # #         if hasattr(self, "grid_csv_file") and not self.grid_csv_file.closed:
# # # #             self.grid_csv_file.close()

# # # #     def remove_ground_ransac(self, pc_data):
# # # #         xyz_data = pc_data[:, :3]
# # # #         if len(xyz_data) < 10:
# # # #             return pc_data
# # # #         try:
# # # #             if xyz_data.shape[0] < 3:
# # # #                 return np.array([])
# # # #             ransac = RANSACRegressor(residual_threshold=0.05)
# # # #             ransac.fit(xyz_data[:, :2], xyz_data[:, 2])
# # # #             return pc_data[~ransac.inlier_mask_]
# # # #         except ValueError:
# # # #             return np.array([])

# # # #     def cluster(self, msg: PointCloud2):
# # # #         self.frame_count += 1

# # # #         # --- Extract (x,y,z,intensity) from PointCloud2 ---
# # # #         pc_list = list(point_cloud2.read_points(
# # # #             msg,
# # # #             field_names=("x", "y", "z", "intensity"),
# # # #             skip_nans=True
# # # #         ))

# # # #         if len(pc_list) == 0:
# # # #             return

# # # #         pc_data = np.array(pc_list, dtype=np.float32)
# # # #         # shape: (N,4) -> columns: x,y,z,intensity

# # # #         pc_data = self.remove_ground_ransac(pc_data)
# # # #         if pc_data.shape[0] < 5:
# # # #             return

# # # #         labels = DBSCAN(eps=0.5, min_samples=5).fit_predict(pc_data[:, :2])
# # # #         unique_labels = set(labels)
# # # #         if -1 in unique_labels:
# # # #             unique_labels.remove(-1)

# # # #         markerarr = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
# # # #         self.publisher.publish(markerarr)
# # # #         markerarr = MarkerArray()
# # # #         self.marker_id = 0

# # # #         if not unique_labels:
# # # #             return
# # # #         self.get_logger().info(f"Frame {self.frame_count}: Found {len(unique_labels)} clusters")

# # # #         for label in unique_labels:
# # # #             cluster_points = pc_data[labels == label]
# # # #             if cluster_points.shape[0] < 5:
# # # #                 continue

# # # #             sizes = np.ptp(cluster_points[:, :3], axis=0)
# # # #             if sizes[0] > 1.0 or sizes[1] > 1.0:
# # # #                 continue

# # # #             min_z = np.min(cluster_points[:, 2])
# # # #             base_points_mask = cluster_points[:, 2] <= min_z + 0.05
# # # #             base_points = cluster_points[base_points_mask]
# # # #             if base_points.shape[0] == 0:
# # # #                 continue

# # # #             squared_distances = base_points[:, 0]**2 + base_points[:, 1]**2
# # # #             closest_point = base_points[np.argmin(squared_distances), :3]

# # # #             plane_normal = np.array([closest_point[0], closest_point[1], 0.0])
# # # #             norm = np.linalg.norm(plane_normal)
# # # #             if norm < 1e-6:
# # # #                 continue
# # # #             plane_normal /= norm

# # # #             vectors_to_project = cluster_points[:, :3] - closest_point
# # # #             distances_from_plane = np.dot(vectors_to_project, plane_normal)
# # # #             projected_points_xyz = cluster_points[:, :3] - np.outer(distances_from_plane, plane_normal)

# # # #             self.create_and_save_grid(projected_points_xyz, cluster_points, closest_point, plane_normal, label)

# # # #             # --- Visualization ---
# # # #             proj_marker = Marker()
# # # #             proj_marker.header.frame_id = msg.header.frame_id or "Lidar_F"
# # # #             proj_marker.header.stamp = self.get_clock().now().to_msg()
# # # #             proj_marker.ns = "projections"
# # # #             proj_marker.id = self.marker_id
# # # #             proj_marker.type = Marker.POINTS
# # # #             proj_marker.action = Marker.ADD
# # # #             proj_marker.pose.orientation.w = 1.0
# # # #             proj_marker.scale.x = 0.03
# # # #             proj_marker.scale.y = 0.03
# # # #             proj_marker.color.r, proj_marker.color.g, proj_marker.color.b = 0.0, 0.5, 1.0
# # # #             proj_marker.color.a = 1.0
# # # #             proj_marker.lifetime = Duration(seconds=0, nanoseconds=int(0.2 * 1e9)).to_msg()
# # # #             proj_marker.points = [Point(x=p[0], y=p[1], z=p[2]) for p in projected_points_xyz]
# # # #             markerarr.markers.append(proj_marker)
# # # #             self.marker_id += 1

# # # #         self.grid_csv_file.flush()
# # # #         if markerarr.markers:
# # # #             self.publisher.publish(markerarr)

# # # #     def create_and_save_grid(self, projected_points, original_points, plane_origin, plane_normal, cluster_label):
# # # #         """Converts projected points into a trimmed 2D grid of intensities and saves to CSV."""
# # # #         z_coords = projected_points[:, 2]
# # # #         tangent_vector = np.array([-plane_normal[1], plane_normal[0], 0])
# # # #         h_coords = np.dot(projected_points - plane_origin, tangent_vector)
# # # #         intensities = original_points[:, 3]

# # # #         z_min, z_max = np.min(z_coords), np.max(z_coords)
# # # #         h_min, h_max = np.min(h_coords), np.max(h_coords)
# # # #         range_z = z_max - z_min
# # # #         range_h = h_max - h_min
# # # #         if range_z < 1e-6 or range_h < 1e-6:
# # # #             return

# # # #         max_range = max(range_z, range_h)
# # # #         scale = (self.GRID_SIZE - 1) / max_range

# # # #         intensity_sum_grid = np.zeros((self.GRID_SIZE, self.GRID_SIZE), dtype=float)
# # # #         point_count_grid = np.zeros((self.GRID_SIZE, self.GRID_SIZE), dtype=int)

# # # #         for i in range(len(h_coords)):
# # # #             col = int((h_coords[i] - h_min) * scale)
# # # #             row = int((z_coords[i] - z_min) * scale)
# # # #             grid_row_idx = self.GRID_SIZE - 1 - row
# # # #             intensity_sum_grid[grid_row_idx, col] += intensities[i]
# # # #             point_count_grid[grid_row_idx, col] += 1

# # # #         final_grid = np.full((self.GRID_SIZE, self.GRID_SIZE), '0', dtype=object)
# # # #         non_empty_cells = point_count_grid > 0
# # # #         average_intensities = np.divide(intensity_sum_grid, point_count_grid, where=non_empty_cells)
# # # #         final_grid[non_empty_cells] = np.char.mod('%.1f', average_intensities[non_empty_cells])

# # # #         # Trim empty rows/cols
# # # #         row_mask = (final_grid != '0').any(axis=1)
# # # #         col_mask = (final_grid != '0').any(axis=0)
# # # #         trimmed_grid = final_grid[row_mask][:, col_mask] if np.any(row_mask) and np.any(col_mask) else np.array([['0']])

# # # #         with self.csv_lock:
# # # #             self.grid_csv_writer.writerow([])
# # # #             self.grid_csv_writer.writerow([f"Frame {self.frame_count}", f"Cluster {cluster_label}"])
# # # #             for grid_row in trimmed_grid:
# # # #                 self.grid_csv_writer.writerow(grid_row)


# # # # def main(args=None):
# # # #     rclpy.init(args=args)
# # # #     node = ConesDetector()
# # # #     try:
# # # #         rclpy.spin(node)
# # # #     except KeyboardInterrupt:
# # # #         pass
# # # #     finally:
# # # #         node.destroy_node()
# # # #         rclpy.shutdown()


# # # # if __name__ == '__main__':
# # # #     main()

# # # # import rclpy
# # # # from rclpy.node import Node
# # # # from rclpy.duration import Duration
# # # # from sklearn.cluster import DBSCAN
# # # # from sklearn.linear_model import RANSACRegressor
# # # # import numpy as np
# # # # from sensor_msgs.msg import PointCloud2
# # # # from sensor_msgs_py import point_cloud2
# # # # from visualization_msgs.msg import Marker, MarkerArray
# # # # from geometry_msgs.msg import Point
# # # # import csv
# # # # import threading


# # # # class ConesDetector(Node):
# # # #     def __init__(self):
# # # #         super().__init__('cones_detector')
# # # #         self.subscriber = self.create_subscription(
# # # #             PointCloud2, '/carmaker/pointcloud', self.cluster, 10
# # # #         )
# # # #         self.publisher = self.create_publisher(MarkerArray, '/visualise', 10)

# # # #         # Counters
# # # #         self.frame_count = 0
# # # #         self.marker_id = 0

# # # #         # CSV setup
# # # #         self.grid_csv_file = open("triangle_grids.csv", "w", newline="")
# # # #         self.grid_csv_writer = csv.writer(self.grid_csv_file)
# # # #         self.csv_lock = threading.Lock()
# # # #         self.GRID_SIZE = 30  # Defines resolution of grid (30x30)

# # # #     def __del__(self):
# # # #         """Ensure CSV file is closed on exit"""
# # # #         if hasattr(self, "grid_csv_file") and not self.grid_csv_file.closed:
# # # #             self.grid_csv_file.close()

# # # #     def remove_ground_ransac(self, pc_data):
# # # #         xyz_data = pc_data[:, :3]
# # # #         if len(xyz_data) < 10:
# # # #             return pc_data
# # # #         try:
# # # #             if xyz_data.shape[0] < 3:
# # # #                 return np.array([])
# # # #             ransac = RANSACRegressor(residual_threshold=0.05)
# # # #             ransac.fit(xyz_data[:, :2], xyz_data[:, 2])
# # # #             return pc_data[~ransac.inlier_mask_]
# # # #         except ValueError:
# # # #             return np.array([])

# # # #     def cluster(self, msg: PointCloud2):
# # # #         self.frame_count += 1

# # # #         # --- Extract (x,y,z,intensity) robustly ---
# # # #         field_names = [f.name for f in msg.fields]
# # # #         if "intensity" in field_names:
# # # #             pc_list = list(point_cloud2.read_points(
# # # #                 msg, field_names=("x", "y", "z", "intensity"), skip_nans=True
# # # #             ))
# # # #         else:
# # # #             pc_list = list(point_cloud2.read_points(
# # # #                 msg, field_names=("x", "y", "z"), skip_nans=True
# # # #             ))
# # # #             # Add dummy intensity = 0
# # # #             pc_list = [p + (0.0,) for p in pc_list]

# # # #         if len(pc_list) == 0:
# # # #             self.get_logger().warn(f"Frame {self.frame_count}: No points read from PointCloud2")
# # # #             return

# # # #         self.get_logger().info(
# # # #             f"Frame {self.frame_count}: received {len(pc_list)} points (fields={field_names})"
# # # #         )

# # # #         pc_data = np.array(pc_list, dtype=np.float32)  # shape (N,4): x,y,z,intensity

# # # #         # --- Remove ground ---
# # # #         pc_data = self.remove_ground_ransac(pc_data)
# # # #         if pc_data.shape[0] < 5:
# # # #             self.get_logger().info("Not enough points after ground removal")
# # # #             return

# # # #         # --- Clustering ---
# # # #         labels = DBSCAN(eps=0.5, min_samples=5).fit_predict(pc_data[:, :2])
# # # #         unique_labels = set(labels)
# # # #         if -1 in unique_labels:
# # # #             unique_labels.remove(-1)

# # # #         # Delete all old markers
# # # #         markerarr = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
# # # #         self.publisher.publish(markerarr)
# # # #         markerarr = MarkerArray()
# # # #         self.marker_id = 0

# # # #         if not unique_labels:
# # # #             self.get_logger().info("No clusters found")
# # # #             return
# # # #         self.get_logger().info(f"Frame {self.frame_count}: Found {len(unique_labels)} clusters")

# # # #         # --- Process clusters ---
# # # #         for label in unique_labels:
# # # #             cluster_points = pc_data[labels == label]
# # # #             if cluster_points.shape[0] < 5:
# # # #                 continue

# # # #             # Rough size filter
# # # #             sizes = np.ptp(cluster_points[:, :3], axis=0)
# # # #             if sizes[0] > 1.5 or sizes[1] > 1.5:  # widened tolerance
# # # #                 continue

# # # #             # Base points (lowest z)
# # # #             min_z = np.min(cluster_points[:, 2])
# # # #             base_points_mask = cluster_points[:, 2] <= min_z + 0.05
# # # #             base_points = cluster_points[base_points_mask]
# # # #             if base_points.shape[0] == 0:
# # # #                 continue

# # # #             # Closest base point
# # # #             squared_distances = base_points[:, 0]**2 + base_points[:, 1]**2
# # # #             closest_point = base_points[np.argmin(squared_distances), :3]

# # # #             # Projection plane
# # # #             plane_normal = np.array([closest_point[0], closest_point[1], 0.0])
# # # #             norm = np.linalg.norm(plane_normal)
# # # #             if norm < 1e-6:
# # # #                 continue
# # # #             plane_normal /= norm

# # # #             vectors_to_project = cluster_points[:, :3] - closest_point
# # # #             distances_from_plane = np.dot(vectors_to_project, plane_normal)
# # # #             projected_points_xyz = cluster_points[:, :3] - np.outer(distances_from_plane, plane_normal)

# # # #             # Save grid
# # # #             self.create_and_save_grid(projected_points_xyz, cluster_points, closest_point, plane_normal, label)

# # # #             # --- Visualization ---
# # # #             proj_marker = Marker()
# # # #             proj_marker.header.frame_id = msg.header.frame_id or "Lidar_F"
# # # #             proj_marker.header.stamp = self.get_clock().now().to_msg()
# # # #             proj_marker.ns = "projections"
# # # #             proj_marker.id = self.marker_id
# # # #             proj_marker.type = Marker.POINTS
# # # #             proj_marker.action = Marker.ADD
# # # #             proj_marker.pose.orientation.w = 1.0
# # # #             proj_marker.scale.x = 0.15   # bigger markers
# # # #             proj_marker.scale.y = 0.15
# # # #             proj_marker.color.r, proj_marker.color.g, proj_marker.color.b = 0.0, 0.5, 1.0
# # # #             proj_marker.color.a = 1.0
# # # #             proj_marker.lifetime = Duration(seconds=0, nanoseconds=int(0.2 * 1e9)).to_msg()
# # # #             proj_marker.points = [Point(x=p[0], y=p[1], z=p[2]) for p in projected_points_xyz]
# # # #             markerarr.markers.append(proj_marker)
# # # #             self.marker_id += 1

# # # #         self.grid_csv_file.flush()
# # # #         if markerarr.markers:
# # # #             self.publisher.publish(markerarr)

# # # #     def create_and_save_grid(self, projected_points, original_points, plane_origin, plane_normal, cluster_label):
# # # #         """Converts projected points into a trimmed 2D grid of intensities and saves to CSV."""
# # # #         z_coords = projected_points[:, 2]
# # # #         tangent_vector = np.array([-plane_normal[1], plane_normal[0], 0])
# # # #         h_coords = np.dot(projected_points - plane_origin, tangent_vector)
# # # #         intensities = original_points[:, 3]

# # # #         z_min, z_max = np.min(z_coords), np.max(z_coords)
# # # #         h_min, h_max = np.min(h_coords), np.max(h_coords)
# # # #         range_z = z_max - z_min
# # # #         range_h = h_max - h_min
# # # #         if range_z < 1e-6 or range_h < 1e-6:
# # # #             return

# # # #         max_range = max(range_z, range_h)
# # # #         scale = (self.GRID_SIZE - 1) / max_range

# # # #         intensity_sum_grid = np.zeros((self.GRID_SIZE, self.GRID_SIZE), dtype=float)
# # # #         point_count_grid = np.zeros((self.GRID_SIZE, self.GRID_SIZE), dtype=int)

# # # #         for i in range(len(h_coords)):
# # # #             col = int((h_coords[i] - h_min) * scale)
# # # #             row = int((z_coords[i] - z_min) * scale)
# # # #             grid_row_idx = self.GRID_SIZE - 1 - row
# # # #             intensity_sum_grid[grid_row_idx, col] += intensities[i]
# # # #             point_count_grid[grid_row_idx, col] += 1

# # # #         final_grid = np.full((self.GRID_SIZE, self.GRID_SIZE), '0', dtype=object)
# # # #         non_empty_cells = point_count_grid > 0
# # # #         average_intensities = np.divide(intensity_sum_grid, point_count_grid, where=non_empty_cells)
# # # #         final_grid[non_empty_cells] = np.char.mod('%.1f', average_intensities[non_empty_cells])

# # # #         # Trim empty rows/cols
# # # #         row_mask = (final_grid != '0').any(axis=1)
# # # #         col_mask = (final_grid != '0').any(axis=0)
# # # #         trimmed_grid = final_grid[row_mask][:, col_mask] if np.any(row_mask) and np.any(col_mask) else np.array([['0']])

# # # #         with self.csv_lock:
# # # #             self.grid_csv_writer.writerow([])
# # # #             self.grid_csv_writer.writerow([f"Frame {self.frame_count}", f"Cluster {cluster_label}"])
# # # #             for grid_row in trimmed_grid:
# # # #                 self.grid_csv_writer.writerow(grid_row)


# # # # def main(args=None):
# # # #     rclpy.init(args=args)
# # # #     node = ConesDetector()
# # # #     try:
# # # #         rclpy.spin(node)
# # # #     except KeyboardInterrupt:
# # # #         pass
# # # #     finally:
# # # #         node.destroy_node()
# # # #         rclpy.shutdown()


# # # # if __name__ == '__main__':
# # # #     main()

# # # import rclpy
# # # from rclpy.node import Node
# # # from sklearn.cluster import DBSCAN
# # # import numpy as np
# # # from sensor_msgs.msg import PointCloud
# # # import csv


# # # class PointCloudLogger(Node):
# # #     def __init__(self):
# # #         super().__init__('point_cloud_logger')
# # #         self.subscriber = self.create_subscription(
# # #             PointCloud, '/carmaker/pointcloud', self.log_clusters_callback, 10
# # #         )

# # #         # Counters
# # #         self.frame_count = 0

# # #         # CSV setup for raw point data
# # #         self.csv_file = open("cluster_points_raw.csv", "w", newline="")
# # #         self.csv_writer = csv.writer(self.csv_file)
# # #         self.csv_writer.writerow([
# # #             "frame", "cluster_id", "x", "y", "z", "intensity"
# # #         ])
# # #         self.get_logger().info("PointCloudLogger node started. Logging to cluster_points_raw.csv")

# # #     def __del__(self):
# # #         """Ensure CSV file is closed when the node is destroyed."""
# # #         if hasattr(self, "csv_file") and not self.csv_file.closed:
# # #             self.csv_file.close()
# # #             self.get_logger().info("CSV file closed.")

# # #     def log_clusters_callback(self, msg):
# # #         """
# # #         Receives a PointCloud message, clusters it, and logs every point
# # #         in every valid cluster to a CSV file.
# # #         """
# # #         self.frame_count += 1
# # #         if not msg.points:
# # #             return

# # #         # 1. Convert PointCloud to a NumPy array with intensity
# # #         pc_data_xyz = np.array([[p.x, p.y, p.z] for p in msg.points])
# # #         if msg.channels and msg.channels[0].name.lower() == 'intensity':
# # #             intensities = np.array(msg.channels[0].values).reshape(-1, 1)
# # #             pc_data = np.hstack((pc_data_xyz, intensities))
# # #         else:
# # #             # If no intensity channel, append a column of zeros
# # #             pc_data = np.hstack((pc_data_xyz, np.zeros((pc_data_xyz.shape[0], 1))))
        
# # #         if pc_data.shape[0] < 5:
# # #             return

# # #         # 2. Perform DBSCAN clustering on the (x, y) coordinates
# # #         # This is a fast operation
# # #         labels = DBSCAN(eps=0.5, min_samples=5).fit_predict(pc_data[:, :2])

# # #         # 3. Iterate through each point and log it if it's in a cluster
# # #         points_logged_in_frame = 0
# # #         for i in range(len(labels)):
# # #             cluster_id = labels[i]
            
# # #             # We only care about points that are part of a cluster (not noise)
# # #             if cluster_id != -1:
# # #                 point_data = pc_data[i]
                
# # #                 self.csv_writer.writerow([
# # #                     self.frame_count,
# # #                     cluster_id,
# # #                     f"{point_data[0]:.4f}", # x
# # #                     f"{point_data[1]:.4f}", # y
# # #                     f"{point_data[2]:.4f}", # z
# # #                     f"{point_data[3]:.4f}"  # intensity
# # #                 ])
# # #                 points_logged_in_frame += 1
        
# # #         # Optional: Flush the file buffer periodically to save data progressively
# # #         if self.frame_count % 10 == 0:
# # #             self.csv_file.flush()

# # #         self.get_logger().info(f"Frame {self.frame_count}: Logged {points_logged_in_frame} points from {len(set(labels)-{-1})} clusters.")


# # # def main(args=None):
# # #     rclpy.init(args=args)
# # #     node = PointCloudLogger()
# # #     try:
# # #         rclpy.spin(node)
# # #     except KeyboardInterrupt:
# # #         print("\nShutting down cleanly.")
# # #     finally:
# # #         # The __del__ method will handle closing the file
# # #         node.destroy_node()
# # #         rclpy.shutdown()

# # # if __name__ == '__main__':
# # #     main()

# # import rclpy
# # from rclpy.node import Node
# # from sklearn.cluster import DBSCAN
# # from sklearn.linear_model import RANSACRegressor # Added back for ground removal
# # import numpy as np
# # from sensor_msgs.msg import PointCloud
# # import csv


# # class GroundRemovedLogger(Node):
# #     def __init__(self):
# #         super().__init__('ground_removed_logger')
# #         self.subscriber = self.create_subscription(
# #             PointCloud, '/carmaker/pointcloud', self.log_clusters_callback, 10
# #         )

# #         # Counters
# #         self.frame_count = 0

# #         # CSV setup for raw point data
# #         self.csv_file = open("cluster_points_no_ground.csv", "w", newline="")
# #         self.csv_writer = csv.writer(self.csv_file)
# #         self.csv_writer.writerow([
# #             "frame", "cluster_id", "x", "y", "z", "intensity"
# #         ])
# #         self.get_logger().info("GroundRemovedLogger node started. Logging to cluster_points_no_ground.csv")

# #     def __del__(self):
# #         """Ensure CSV file is closed when the node is destroyed."""
# #         if hasattr(self, "csv_file") and not self.csv_file.closed:
# #             self.csv_file.close()
# #             self.get_logger().info("CSV file closed.")

# #     # --- NEW: RANSAC function added back into the class ---
# #     def remove_ground_ransac(self, pc_data):
# #         """
# #         Removes the ground plane from point cloud data using RANSAC.
# #         pc_data is expected to have shape (N, 4) with [x, y, z, intensity].
# #         """
# #         # RANSAC works on the XYZ coordinates
# #         xyz_data = pc_data[:, :3]
# #         if len(xyz_data) < 10:
# #             return pc_data
        
# #         try:
# #             # RANSAC needs at least 3 points to fit a plane
# #             if xyz_data.shape[0] < 3:
# #                 return np.array([]) 

# #             ransac = RANSACRegressor(residual_threshold=0.05)
# #             ransac.fit(xyz_data[:, :2], xyz_data[:, 2])
            
# #             # Return the original data (with intensity) for points that are NOT the ground
# #             return pc_data[~ransac.inlier_mask_]
        
# #         except ValueError as e:
# #             self.get_logger().warn(f"RANSAC failed: {e}. Returning no points.")
# #             return np.array([])

# #     def log_clusters_callback(self, msg):
# #         """
# #         Receives a PointCloud, removes the ground, clusters the remaining points,
# #         and logs every point in every valid cluster to a CSV file.
# #         """
# #         self.frame_count += 1
# #         if not msg.points:
# #             return

# #         # 1. Convert PointCloud to a NumPy array with intensity
# #         pc_data_xyz = np.array([[p.x, p.y, p.z] for p in msg.points])
# #         if msg.channels and msg.channels[0].name.lower() == 'intensity':
# #             intensities = np.array(msg.channels[0].values).reshape(-1, 1)
# #             pc_data = np.hstack((pc_data_xyz, intensities))
# #         else:
# #             pc_data = np.hstack((pc_data_xyz, np.zeros((pc_data_xyz.shape[0], 1))))
        
# #         # --- NEW: Call the RANSAC ground removal function ---
# #         pc_data_no_ground = self.remove_ground_ransac(pc_data)

# #         if pc_data_no_ground.shape[0] < 5:
# #             return

# #         # 2. Perform DBSCAN clustering on the remaining points
# #         labels = DBSCAN(eps=0.5, min_samples=5).fit_predict(pc_data_no_ground[:, :2])

# #         # 3. Iterate through each point and log it if it's in a cluster
# #         points_logged_in_frame = 0
# #         for i in range(len(labels)):
# #             cluster_id = labels[i]
            
# #             if cluster_id != -1: # Ignore noise
# #                 point_data = pc_data_no_ground[i]
                
# #                 self.csv_writer.writerow([
# #                     self.frame_count,
# #                     cluster_id,
# #                     f"{point_data[0]:.4f}", # x
# #                     f"{point_data[1]:.4f}", # y
# #                     f"{point_data[2]:.4f}", # z
# #                     f"{point_data[3]:.4f}"  # intensity
# #                 ])
# #                 points_logged_in_frame += 1
        
# #         if self.frame_count % 10 == 0:
# #             self.csv_file.flush()

# #         self.get_logger().info(f"Frame {self.frame_count}: Logged {points_logged_in_frame} non-ground points from {len(set(labels)-{-1})} clusters.")


# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = GroundRemovedLogger()
# #     try:
# #         rclpy.spin(node)
# #     except KeyboardInterrupt:
# #         print("\nShutting down cleanly.")
# #     finally:
# #         node.destroy_node()
# #         rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()

# import rclpy
# from rclpy.node import Node
# from rclpy.duration import Duration
# from sklearn.cluster import DBSCAN
# from sklearn.linear_model import RANSACRegressor
# import numpy as np
# from sensor_msgs.msg import PointCloud
# from visualization_msgs.msg import Marker, MarkerArray # Added for visualization
# import csv


# class ConeDetectorAndLogger(Node):
#     def __init__(self):
#         super().__init__('cone_detector_and_logger')
#         # Subscriber for point cloud data
#         self.subscriber = self.create_subscription(
#             PointCloud, '/carmaker/pointcloud', self.process_point_cloud, 10
#         )
#         # --- NEW: Publisher for visualization markers ---
#         self.publisher = self.create_publisher(MarkerArray, '/cone_markers', 10)

#         # Counters
#         self.frame_count = 0
#         self.marker_id = 0

#         # CSV setup
#         self.csv_file = open("cluster_points_no_ground.csv", "w", newline="")
#         self.csv_writer = csv.writer(self.csv_file)
#         self.csv_writer.writerow([
#             "frame", "cluster_id", "x", "y", "z", "intensity"
#         ])
#         self.get_logger().info("Node started. Logging to CSV and publishing markers to /cone_markers")

#     def __del__(self):
#         if hasattr(self, "csv_file") and not self.csv_file.closed:
#             self.csv_file.close()

#     def remove_ground_ransac(self, pc_data):
#         xyz_data = pc_data[:, :3]
#         if len(xyz_data) < 10: return pc_data
#         try:
#             if xyz_data.shape[0] < 3: return np.array([]) 
#             ransac = RANSACRegressor(residual_threshold=0.05)
#             ransac.fit(xyz_data[:, :2], xyz_data[:, 2])
#             return pc_data[~ransac.inlier_mask_]
#         except ValueError as e:
#             self.get_logger().warn(f"RANSAC failed: {e}. Returning no points.")
#             return np.array([])

#     def process_point_cloud(self, msg):
#         self.frame_count += 1
#         if not msg.points: return

#         # --- NEW: Clear previous markers from RViz ---
#         marker_array = MarkerArray()
#         clear_marker = Marker(action=Marker.DELETEALL)
#         marker_array.markers.append(clear_marker)
#         self.publisher.publish(marker_array)

#         # 1. Convert PointCloud to NumPy array
#         pc_data_xyz = np.array([[p.x, p.y, p.z] for p in msg.points])
#         if msg.channels and msg.channels[0].name.lower() == 'intensity':
#             intensities = np.array(msg.channels[0].values).reshape(-1, 1)
#             pc_data = np.hstack((pc_data_xyz, intensities))
#         else:
#             pc_data = np.hstack((pc_data_xyz, np.zeros((pc_data_xyz.shape[0], 1))))
        
#         # 2. Remove ground plane
#         pc_data_no_ground = self.remove_ground_ransac(pc_data)
#         if pc_data_no_ground.shape[0] < 5: return

#         # 3. Perform clustering
#         labels = DBSCAN(eps=0.5, min_samples=5).fit_predict(pc_data_no_ground[:, :2])
#         unique_labels = set(labels)
#         if -1 in unique_labels: unique_labels.remove(-1)

#         marker_array = MarkerArray() # For new markers
#         self.marker_id = 0

#         # 4. Process each detected cluster
#         for label in unique_labels:
#             cluster_points = pc_data_no_ground[labels == label]
            
#             # --- Log every point in the cluster to CSV ---
#             for point_data in cluster_points:
#                 self.csv_writer.writerow([
#                     self.frame_count, label,
#                     f"{point_data[0]:.4f}", f"{point_data[1]:.4f}",
#                     f"{point_data[2]:.4f}", f"{point_data[3]:.4f}"
#                 ])
            
#             # --- NEW: Create a visualization marker for the cluster ---
#             centroid = cluster_points[:, :3].mean(axis=0)
#             sizes = np.ptp(cluster_points[:, :3], axis=0)

#             marker = Marker()
#             marker.header.frame_id = "Lidar_F"
#             marker.header.stamp = self.get_clock().now().to_msg()
#             marker.ns = "cones"
#             marker.id = self.marker_id
#             marker.type = Marker.CYLINDER
#             marker.action = Marker.ADD
            
#             marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = \
#                 centroid[0], centroid[1], centroid[2]
#             marker.pose.orientation.w = 1.0
            
#             marker.scale.x = max(sizes[0], 0.1) # Diameter
#             marker.scale.y = max(sizes[1], 0.1) # Diameter
#             marker.scale.z = max(sizes[2], 0.2) # Height
            
#             marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.5, 0.0, 0.8 # Orange
#             marker.lifetime = Duration(seconds=0.2).to_msg()
            
#             marker_array.markers.append(marker)
#             self.marker_id += 1

#         # Publish the new markers
#         if marker_array.markers:
#             self.publisher.publish(marker_array)

#         self.get_logger().info(f"Frame {self.frame_count}: Found {len(unique_labels)} clusters.")


# def main(args=None):
#     rclpy.init(args=args)
#     node = ConeDetectorAndLogger()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         print("\nShutting down cleanly.")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# import rclpy
# from rclpy.node import Node
# from rclpy.duration import Duration
# from sklearn.cluster import DBSCAN
# from sklearn.linear_model import RANSACRegressor
# import numpy as np
# from sensor_msgs.msg import PointCloud
# from visualization_msgs.msg import Marker, MarkerArray
# import csv


# class ConeDetectorAndLogger(Node):
#     def __init__(self):
#         super().__init__('cone_detector_and_logger')
#         # Subscriber for point cloud data
#         self.subscriber = self.create_subscription(
#             PointCloud, '/carmaker/pointcloud', self.process_point_cloud, 10
#         )
#         # Publisher for visualization markers
#         self.publisher = self.create_publisher(MarkerArray, '/cone_markers', 10)

#         # Counters
#         self.frame_count = 0
#         self.marker_id = 0

#         # CSV setup
#         self.csv_file = open("cluster_points_no_ground.csv", "w", newline="")
#         self.csv_writer = csv.writer(self.csv_file)
#         self.csv_writer.writerow([
#             "frame", "cluster_id", "x", "y", "z", "intensity"
#         ])
#         self.get_logger().info("Node started. Logging to CSV and publishing markers to /cone_markers")

#     def __del__(self):
#         if hasattr(self, "csv_file") and not self.csv_file.closed:
#             self.csv_file.close()

#     def remove_ground_ransac(self, pc_data):
#         xyz_data = pc_data[:, :3]
#         if len(xyz_data) < 10: return pc_data
#         try:
#             if xyz_data.shape[0] < 3: return np.array([]) 
#             ransac = RANSACRegressor(residual_threshold=0.05)
#             ransac.fit(xyz_data[:, :2], xyz_data[:, 2])
#             return pc_data[~ransac.inlier_mask_]
#         except ValueError as e:
#             self.get_logger().warn(f"RANSAC failed: {e}. Returning no points.")
#             return np.array([])

#     def process_point_cloud(self, msg):
#         self.frame_count += 1
#         if not msg.points: return

#         # Clear previous markers from RViz
#         marker_array = MarkerArray()
#         clear_marker = Marker(action=Marker.DELETEALL)
#         marker_array.markers.append(clear_marker)
#         self.publisher.publish(marker_array)

#         # 1. Convert PointCloud to NumPy array
#         pc_data_xyz = np.array([[p.x, p.y, p.z] for p in msg.points])
#         if msg.channels and msg.channels[0].name.lower() == 'intensity':
#             intensities = np.array(msg.channels[0].values).reshape(-1, 1)
#             pc_data = np.hstack((pc_data_xyz, intensities))
#         else:
#             pc_data = np.hstack((pc_data_xyz, np.zeros((pc_data_xyz.shape[0], 1))))
        
#         # 2. Remove ground plane
#         pc_data_no_ground = self.remove_ground_ransac(pc_data)
#         if pc_data_no_ground.shape[0] < 5: return

#         # 3. Perform clustering
#         labels = DBSCAN(eps=0.5, min_samples=5).fit_predict(pc_data_no_ground[:, :2])
#         unique_labels = set(labels)
#         if -1 in unique_labels: unique_labels.remove(-1)

#         marker_array = MarkerArray() # For new markers
#         self.marker_id = 0

#         # 4. Process each detected cluster
#         for label in unique_labels:
#             cluster_points = pc_data_no_ground[labels == label]
            
#             # --- Log every point in the cluster to CSV ---
#             for point_data in cluster_points:
#                 # --- MODIFIED: Convert intensity to integer before writing ---
#                 intensity_int = int(point_data[3])
                
#                 self.csv_writer.writerow([
#                     self.frame_count, label,
#                     f"{point_data[0]:.4f}", f"{point_data[1]:.4f}",
#                     f"{point_data[2]:.4f}", intensity_int
#                 ])
            
#             # --- Create a visualization marker for the cluster ---
#             centroid = cluster_points[:, :3].mean(axis=0)
#             sizes = np.ptp(cluster_points[:, :3], axis=0)

#             marker = Marker()
#             marker.header.frame_id = "Lidar_F"
#             marker.header.stamp = self.get_clock().now().to_msg()
#             marker.ns = "cones"
#             marker.id = self.marker_id
#             marker.type = Marker.CYLINDER
#             marker.action = Marker.ADD
            
#             marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = \
#                 centroid[0], centroid[1], centroid[2]
#             marker.pose.orientation.w = 1.0
            
#             marker.scale.x = max(sizes[0], 0.1) # Diameter
#             marker.scale.y = max(sizes[1], 0.1) # Diameter
#             marker.scale.z = max(sizes[2], 0.2) # Height
            
#             marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.5, 0.0, 0.8 # Orange
#             marker.lifetime = Duration(seconds=0.2).to_msg()
            
#             marker_array.markers.append(marker)
#             self.marker_id += 1

#         # Publish the new markers
#         if marker_array.markers:
#             self.publisher.publish(marker_array)

#         self.get_logger().info(f"Frame {self.frame_count}: Found {len(unique_labels)} clusters.")


# def main(args=None):
#     rclpy.init(args=args)
#     node = ConeDetectorAndLogger()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         print("\nShutting down cleanly.")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# import rclpy
# from rclpy.node import Node
# from rclpy.duration import Duration
# from sklearn.cluster import DBSCAN
# from sklearn.linear_model import RANSACRegressor
# import numpy as np
# from sensor_msgs.msg import PointCloud
# from visualization_msgs.msg import Marker, MarkerArray
# import csv


# class ConeDetectorAndLogger(Node):
#     def __init__(self):
#         super().__init__('cone_detector_and_logger')
#         # Subscriber for point cloud data
#         self.subscriber = self.create_subscription(
#             PointCloud, '/carmaker/pointcloud', self.process_point_cloud, 10
#         )
#         # Publisher for visualization markers
#         self.publisher = self.create_publisher(MarkerArray, '/cone_markers', 10)

#         # Counters
#         self.frame_count = 0
#         self.marker_id = 0

#         # CSV setup
#         self.csv_file = open("cluster_points_no_ground.csv", "w", newline="")
#         self.csv_writer = csv.writer(self.csv_file)
#         self.csv_writer.writerow([
#             "frame", "cluster_id", "x", "y", "z", "intensity"
#         ])
#         self.get_logger().info("Node started. Logging to CSV and publishing markers to /cone_markers")

#     def __del__(self):
#         if hasattr(self, "csv_file") and not self.csv_file.closed:
#             self.csv_file.close()

#     def remove_ground_ransac(self, pc_data):
#         xyz_data = pc_data[:, :3]
#         if len(xyz_data) < 10: return pc_data
#         try:
#             if xyz_data.shape[0] < 3: return np.array([]) 
#             ransac = RANSACRegressor(residual_threshold=0.05)
#             ransac.fit(xyz_data[:, :2], xyz_data[:, 2])
#             return pc_data[~ransac.inlier_mask_]
#         except ValueError as e:
#             self.get_logger().warn(f"RANSAC failed: {e}. Returning no points.")
#             return np.array([])

#     def process_point_cloud(self, msg):
#         self.frame_count += 1
#         if not msg.points: return

#         # Clear previous markers from RViz
#         marker_array = MarkerArray()
# #         clear_marker = Marker(action=Marker.DELETEALL)
# #         marker_array.markers.append(clear_marker)
# #         self.publisher.publish(marker_array)

# #         # 1. Convert PointCloud to NumPy array
# #         pc_data_xyz = np.array([[p.x, p.y, p.z] for p in msg.points])
# #         if msg.channels and msg.channels[0].name.lower() == 'intensity':
# #             intensities = np.array(msg.channels[0].values).reshape(-1, 1)
# #             pc_data = np.hstack((pc_data_xyz, intensities))
# #         else:
# #             pc_data = np.hstack((pc_data_xyz, np.zeros((pc_data_xyz.shape[0], 1))))
        
# #         # 2. Remove ground plane
# #         pc_data_no_ground = self.remove_ground_ransac(pc_data)
# #         if pc_data_no_ground.shape[0] < 5: return

# #         # 3. Perform clustering
# #         labels = DBSCAN(eps=0.5, min_samples=5).fit_predict(pc_data_no_ground[:, :2])
# #         unique_labels = set(labels)
# #         if -1 in unique_labels: unique_labels.remove(-1)

# #         marker_array = MarkerArray() # For new markers
# #         self.marker_id = 0

# #         # 4. Process each detected cluster
# #         for label in unique_labels:
# #             cluster_points = pc_data_no_ground[labels == label]
            
# #             # --- NEW: Intensity-based filtering logic ---
# #             # Calculate the average intensity of the points in the cluster
# #             avg_intensity = cluster_points[:, 3].mean()
            
# #             # Set a threshold to distinguish cones from other objects
# #             INTENSITY_THRESHOLD = 10000 
            
# #             # If the average intensity is too high, it's not a cone, so we skip it
# #             if avg_intensity > INTENSITY_THRESHOLD:
# #                 self.get_logger().warn(f"Discarding cluster {label} due to high avg intensity: {avg_intensity:.0f}")
# #                 continue
# #             # --- End of new logic ---

# #             # --- Log every point in the cluster to CSV ---
# #             for point_data in cluster_points:
# #                 intensity_int = int(point_data[3])
                
# #                 self.csv_writer.writerow([
# #                     self.frame_count, label,
# #                     f"{point_data[0]:.4f}", f"{point_data[1]:.4f}",
# #                     f"{point_data[2]:.4f}", intensity_int
# #                 ])
            
# #             # --- Create a visualization marker for the cluster ---
# #             centroid = cluster_points[:, :3].mean(axis=0)
# #             sizes = np.ptp(cluster_points[:, :3], axis=0)

# #             marker = Marker()
# #             marker.header.frame_id = "Lidar_F"
# #             marker.header.stamp = self.get_clock().now().to_msg()
# #             marker.ns = "cones"
# #             marker.id = self.marker_id
# #             marker.type = Marker.CYLINDER
# #             marker.action = Marker.ADD
            
# #             marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = \
# #                 centroid[0], centroid[1], centroid[2]
# #             marker.pose.orientation.w = 1.0
            
# #             marker.scale.x = max(sizes[0], 0.1) # Diameter
# #             marker.scale.y = max(sizes[1], 0.1) # Diameter
# #             marker.scale.z = max(sizes[2], 0.2) # Height
            
# #             marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.5, 0.0, 0.8 # Orange
# #             marker.lifetime = Duration(seconds=0.2).to_msg()
            
# #             marker_array.markers.append(marker)
# #             self.marker_id += 1

# #         # Publish the new markers
# #         if marker_array.markers:
# #             self.publisher.publish(marker_array)

# #         self.get_logger().info(f"Frame {self.frame_count}: Found {len(unique_labels)} clusters.")


# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = ConeDetectorAndLogger()
# #     try:
# #         rclpy.spin(node)
# #     except KeyboardInterrupt:
# #         print("\nShutting down cleanly.")
# #     finally:
# #         node.destroy_node()
# #         rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()

# import rclpy
# from rclpy.node import Node
# from rclpy.duration import Duration
# from sklearn.cluster import DBSCAN
# from sklearn.linear_model import RANSACRegressor
# import numpy as np
# from sensor_msgs.msg import PointCloud
# from visualization_msgs.msg import Marker, MarkerArray
# import csv


# class ConeDetectorAndLogger(Node):
#     def __init__(self):
#         super().__init__('cone_detector_and_logger')
#         # Subscriber for point cloud data
#         self.subscriber = self.create_subscription(
#             PointCloud, '/carmaker/pointcloud', self.process_point_cloud, 10
#         )
#         # Publisher for visualization markers
#         self.publisher = self.create_publisher(MarkerArray, '/cone_markers', 10)

#         # Counters
#         self.frame_count = 0
#         self.marker_id = 0

#         # CSV setup
#         self.csv_file = open("cluster_points_no_ground.csv", "w", newline="")
#         self.csv_writer = csv.writer(self.csv_file)
#         self.csv_writer.writerow([
#             "frame", "cluster_id", "x", "y", "z", "intensity_rescaled"
#         ])
#         self.get_logger().info("Node started. Logging to CSV and publishing markers to /cone_markers")

#     def __del__(self):
#         if hasattr(self, "csv_file") and not self.csv_file.closed:
#             self.csv_file.close()

#     def remove_ground_ransac(self, pc_data):
#         xyz_data = pc_data[:, :3]
#         if len(xyz_data) < 10: return pc_data
#         try:
#             if xyz_data.shape[0] < 3: return np.array([]) 
#             ransac = RANSACRegressor(residual_threshold=0.05)
#             ransac.fit(xyz_data[:, :2], xyz_data[:, 2])
#             return pc_data[~ransac.inlier_mask_]
#         except ValueError as e:
#             self.get_logger().warn(f"RANSAC failed: {e}. Returning no points.")
#             return np.array([])

#     def process_point_cloud(self, msg):
#         self.frame_count += 1
#         if not msg.points: return

#         # Clear previous markers from RViz
#         marker_array = MarkerArray()
#         clear_marker = Marker(action=Marker.DELETEALL)
#         marker_array.markers.append(clear_marker)
#         self.publisher.publish(marker_array)

#         # 1. Convert PointCloud to NumPy array
#         pc_data_xyz = np.array([[p.x, p.y, p.z] for p in msg.points])
#         if msg.channels and msg.channels[0].name.lower() == 'intensity':
#             intensities = np.array(msg.channels[0].values).reshape(-1, 1)
#             pc_data = np.hstack((pc_data_xyz, intensities))
#         else:
#             pc_data = np.hstack((pc_data_xyz, np.zeros((pc_data_xyz.shape[0], 1))))
        
#         # 2. Remove ground plane
#         pc_data_no_ground = self.remove_ground_ransac(pc_data)
#         if pc_data_no_ground.shape[0] < 5: return

#         # 3. Perform clustering
#         labels = DBSCAN(eps=0.5, min_samples=5).fit_predict(pc_data_no_ground[:, :2])
#         unique_labels = set(labels)
#         if -1 in unique_labels: unique_labels.remove(-1)

#         marker_array = MarkerArray() # For new markers
#         self.marker_id = 0

#         # 4. Process each detected cluster
#         for label in unique_labels:
#             cluster_points = pc_data_no_ground[labels == label]
            
#             # --- NEW: Rescale intensity values for this cluster to a 0-255 range ---
#             original_intensities = cluster_points[:, 3]
#             min_intensity = np.min(original_intensities)
#             max_intensity = np.max(original_intensities)
#             intensity_range = max_intensity - min_intensity

#             rescaled_intensities = np.zeros_like(original_intensities, dtype=int)
#             # Avoid division by zero if all points have the same intensity
#             if intensity_range > 0:
#                 # Apply min-max normalization and scale to 0-255
#                 rescaled_intensities = ((original_intensities - min_intensity) / intensity_range * 255).astype(int)
#             # --- End of new logic ---

#             # --- Log every point with its new rescaled intensity ---
#             for i, point_data in enumerate(cluster_points):
#                 self.csv_writer.writerow([
#                     self.frame_count, label,
#                     f"{point_data[0]:.4f}", f"{point_data[1]:.4f}",
#                     f"{point_data[2]:.4f}", rescaled_intensities[i]
#                 ])
            
#             # --- Create a visualization marker for the cluster ---
#             centroid = cluster_points[:, :3].mean(axis=0)
#             sizes = np.ptp(cluster_points[:, :3], axis=0)

#             marker = Marker()
#             marker.header.frame_id = "Lidar_F"
#             marker.header.stamp = self.get_clock().now().to_msg()
#             marker.ns = "cones"
#             marker.id = self.marker_id
#             marker.type = Marker.CYLINDER
#             marker.action = Marker.ADD
            
#             marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = \
#                 centroid[0], centroid[1], centroid[2]
#             marker.pose.orientation.w = 1.0
            
#             marker.scale.x = max(sizes[0], 0.1) # Diameter
#             marker.scale.y = max(sizes[1], 0.1) # Diameter
#             marker.scale.z = max(sizes[2], 0.2) # Height
            
#             marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.5, 0.0, 0.8 # Orange
#             marker.lifetime = Duration(seconds=0.2).to_msg()
            
#             marker_array.markers.append(marker)
#             self.marker_id += 1

#         # Publish the new markers
#         if marker_array.markers:
#             self.publisher.publish(marker_array)

#         self.get_logger().info(f"Frame {self.frame_count}: Found {len(unique_labels)} clusters.")


# def main(args=None):
#     rclpy.init(args=args)
#     node = ConeDetectorAndLogger()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         print("\nShutting down cleanly.")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sklearn.cluster import DBSCAN
from sklearn.linear_model import RANSACRegressor
import numpy as np
from sensor_msgs.msg import PointCloud
from visualization_msgs.msg import Marker, MarkerArray
import csv


class ConeDetectorAndLogger(Node):
    def __init__(self):
        super().__init__('cone_detector_and_logger')
        self.subscriber = self.create_subscription(
            PointCloud, '/carmaker/pointcloud', self.process_point_cloud, 10
        )
        self.publisher = self.create_publisher(MarkerArray, '/cone_markers', 10)

        # Counters
        self.frame_count = 0
        self.marker_id = 0

        # CSV setup for point-by-point data
        self.points_csv_file = open("cluster_points_no_ground.csv", "w", newline="")
        self.points_csv_writer = csv.writer(self.points_csv_file)
        self.points_csv_writer.writerow([
            "frame", "cluster_id", "x", "y", "z", "intensity_rescaled"
        ])
        
        # --- NEW: CSV setup for grid visualization ---
        self.grid_csv_file = open("triangle_grids.csv", "w", newline="")
        self.grid_csv_writer = csv.writer(self.grid_csv_file)
        self.GRID_SIZE = 30

        self.get_logger().info("Node started. Logging points and grids, publishing markers.")

    def __del__(self):
        if hasattr(self, "points_csv_file") and not self.points_csv_file.closed:
            self.points_csv_file.close()
        if hasattr(self, "grid_csv_file") and not self.grid_csv_file.closed:
            self.grid_csv_file.close()

    def remove_ground_ransac(self, pc_data):
        xyz_data = pc_data[:, :3]
        if len(xyz_data) < 10: return pc_data
        try:
            if xyz_data.shape[0] < 3: return np.array([]) 
            ransac = RANSACRegressor(residual_threshold=0.05)
            ransac.fit(xyz_data[:, :2], xyz_data[:, 2])
            return pc_data[~ransac.inlier_mask_]
        except ValueError as e:
            self.get_logger().warn(f"RANSAC failed: {e}. Returning no points.")
            return np.array([])

    def process_point_cloud(self, msg):
        self.frame_count += 1
        if not msg.points: return

        marker_array = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        self.publisher.publish(marker_array)

        pc_data_xyz = np.array([[p.x, p.y, p.z] for p in msg.points])
        if msg.channels and msg.channels[0].name.lower() == 'intensity':
            intensities = np.array(msg.channels[0].values).reshape(-1, 1)
            pc_data = np.hstack((pc_data_xyz, intensities))
        else:
            pc_data = np.hstack((pc_data_xyz, np.zeros((pc_data_xyz.shape[0], 1))))
        
        pc_data_no_ground = self.remove_ground_ransac(pc_data)
        if pc_data_no_ground.shape[0] < 5: return

        labels = DBSCAN(eps=0.5, min_samples=5).fit_predict(pc_data_no_ground[:, :2])
        unique_labels = set(labels)
        if -1 in unique_labels: unique_labels.remove(-1)

        marker_array = MarkerArray()
        self.marker_id = 0

        for label in unique_labels:
            cluster_points = pc_data_no_ground[labels == label]
            
            original_intensities = cluster_points[:, 3]
            min_intensity = np.min(original_intensities)
            max_intensity = np.max(original_intensities)
            intensity_range = max_intensity - min_intensity
            rescaled_intensities = np.zeros_like(original_intensities, dtype=int)
            if intensity_range > 0:
                rescaled_intensities = ((original_intensities - min_intensity) / intensity_range * 255).astype(int)

            for i, point_data in enumerate(cluster_points):
                self.points_csv_writer.writerow([
                    self.frame_count, label,
                    f"{point_data[0]:.4f}", f"{point_data[1]:.4f}",
                    f"{point_data[2]:.4f}", rescaled_intensities[i]
                ])

            closest_point = cluster_points[np.argmin(np.linalg.norm(cluster_points[:,:2], axis=1)), :3]
            plane_normal = np.array([closest_point[0], closest_point[1], 0.0])
            norm = np.linalg.norm(plane_normal)
            if norm < 1e-6: continue
            plane_normal /= norm
            vectors_to_project = cluster_points[:, :3] - closest_point
            distances_from_plane = np.dot(vectors_to_project, plane_normal)
            projected_points_xyz = cluster_points[:, :3] - np.outer(distances_from_plane, plane_normal)
            
            # --- Call the grid creation function ---
            self.create_and_save_grid(projected_points_xyz, rescaled_intensities, closest_point, plane_normal, label)

            centroid = cluster_points[:, :3].mean(axis=0)
            sizes = np.ptp(cluster_points[:, :3], axis=0)
            marker = Marker()
            marker.header.frame_id = "Lidar_F"
            # ... (rest of marker code is the same)
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "cones"
            marker.id = self.marker_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = centroid[0], centroid[1], centroid[2]
            marker.pose.orientation.w = 1.0
            marker.scale.x, marker.scale.y, marker.scale.z = max(sizes[0], 0.1), max(sizes[1], 0.1), max(sizes[2], 0.2)
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.5, 0.0, 0.8
            marker.lifetime = Duration(seconds=0.2).to_msg()
            marker_array.markers.append(marker)
            self.marker_id += 1

        if marker_array.markers:
            self.publisher.publish(marker_array)
        self.get_logger().info(f"Frame {self.frame_count}: Processed {len(unique_labels)} clusters.")

    def create_and_save_grid(self, projected_points, rescaled_intensities, plane_origin, plane_normal, cluster_label):
        """Creates a trimmed 2D grid of rescaled intensities and saves it to a CSV."""
        z_coords = projected_points[:, 2]
        tangent_vector = np.array([-plane_normal[1], plane_normal[0], 0])
        h_coords = np.dot(projected_points - plane_origin, tangent_vector)

        z_min, z_max = np.min(z_coords), np.max(z_coords)
        h_min, h_max = np.min(h_coords), np.max(h_coords)
        range_z, range_h = z_max - z_min, h_max - h_min
        if range_z < 1e-6 or range_h < 1e-6: return

        scale = (self.GRID_SIZE - 1) / max(range_z, range_h)

        intensity_sum_grid = np.zeros((self.GRID_SIZE, self.GRID_SIZE), dtype=float)
        point_count_grid = np.zeros((self.GRID_SIZE, self.GRID_SIZE), dtype=int)
        
        for i in range(len(h_coords)):
            col = int((h_coords[i] - h_min) * scale)
            row = int((z_coords[i] - z_min) * scale)
            grid_row_idx = self.GRID_SIZE - 1 - row
            intensity_sum_grid[grid_row_idx, col] += rescaled_intensities[i]
            point_count_grid[grid_row_idx, col] += 1

        final_grid = np.full((self.GRID_SIZE, self.GRID_SIZE), '0', dtype=object)
        non_empty_cells = point_count_grid > 0
        average_intensities = np.divide(intensity_sum_grid, point_count_grid, where=non_empty_cells)
        final_grid[non_empty_cells] = np.char.mod('%d', average_intensities[non_empty_cells])

        row_mask = (final_grid != '0').any(axis=1)
        col_mask = (final_grid != '0').any(axis=0)
        if np.any(row_mask) and np.any(col_mask):
            trimmed_grid = final_grid[row_mask][:, col_mask]
        else:
            trimmed_grid = np.array([['0']])

        self.grid_csv_writer.writerow([])
        self.grid_csv_writer.writerow([f"Frame {self.frame_count}", f"Cluster {cluster_label}"])
        for grid_row in trimmed_grid:
            self.grid_csv_writer.writerow(grid_row)


def main(args=None):
    rclpy.init(args=args)
    node = ConeDetectorAndLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down cleanly.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()