#!/usr/bin/env python3
import rosbag
import open3d as o3d
import numpy as np
import argparse
import sys
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
import struct
import copy

class InteractiveMapVisualizer:
    def __init__(self, keyframes_data, voxel_size=1.0):
        """
        Initialize the interactive visualizer
        keyframes_data: list of (points, pose_matrix) tuples
        """
        self.keyframes_data = keyframes_data
        self.voxel_size = voxel_size

        # Visualization states
        self.show_colorful_keyframes = False
        self.show_trajectory = True
        self.show_height_color = True

        # Build point clouds
        self.merged_pcd = None
        self.colored_keyframes_pcds = []
        self.trajectory_lines = None
        self.coordinate_frame = None

        # For point editing
        self.picked_points = []

        self._build_geometries()

    def _build_geometries(self):
        """Build all geometric objects"""
        print("Building geometries...")

        # Merge all keyframes
        all_points = []
        for points, _ in self.keyframes_data:
            all_points.extend(points)

        all_points = np.array(all_points)

        # Apply voxel downsampling
        if self.voxel_size > 0:
            print(f"Downsampling merged cloud with voxel size: {self.voxel_size}")
            all_points = self._voxel_downsample(all_points, self.voxel_size)

        # Create merged point cloud
        self.merged_pcd = o3d.geometry.PointCloud()
        self.merged_pcd.points = o3d.utility.Vector3dVector(all_points)
        self._color_by_height(self.merged_pcd)

        # Create individual keyframe point clouds with different colors
        print("Creating colored keyframes...")
        colors = self._generate_colors(len(self.keyframes_data))

        for i, (points, _) in enumerate(self.keyframes_data):
            if len(points) == 0:
                continue

            # Downsample individual keyframe
            if self.voxel_size > 0:
                points = self._voxel_downsample(points, self.voxel_size)

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)

            # Assign unique color to this keyframe
            color = colors[i]
            pcd.colors = o3d.utility.Vector3dVector(np.tile(color, (len(points), 1)))

            self.colored_keyframes_pcds.append(pcd)

        # Create trajectory
        self._build_trajectory()

        # Create coordinate frame
        self.coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10.0)

        print(f"Built {len(self.colored_keyframes_pcds)} keyframe point clouds")

    def _voxel_downsample(self, points, voxel_size):
        """Downsample points using voxel grid"""
        if len(points) == 0:
            return points

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        downsampled_pcd = pcd.voxel_down_sample(voxel_size)
        return np.asarray(downsampled_pcd.points)

    def _color_by_height(self, pcd):
        """Color point cloud by height (z-coordinate)"""
        points = np.asarray(pcd.points)
        colors = []
        z_values = points[:, 2]
        z_min, z_max = np.min(z_values), np.max(z_values)

        for z in z_values:
            if z_max > z_min:
                normalized_z = (z - z_min) / (z_max - z_min)
            else:
                normalized_z = 0.5

            # Blue (low) to red (high)
            color = [normalized_z, 0.5, 1.0 - normalized_z]
            colors.append(color)

        pcd.colors = o3d.utility.Vector3dVector(colors)

    def _generate_colors(self, n):
        """Generate n distinct colors"""
        colors = []
        for i in range(n):
            hue = (i * 360.0 / n) % 360
            colors.append(self._hsv_to_rgb(hue, 0.8, 0.9))
        return colors

    def _hsv_to_rgb(self, h, s, v):
        """Convert HSV to RGB"""
        h = h / 60.0
        c = v * s
        x = c * (1 - abs(h % 2 - 1))
        m = v - c

        if h < 1:
            rgb = [c, x, 0]
        elif h < 2:
            rgb = [x, c, 0]
        elif h < 3:
            rgb = [0, c, x]
        elif h < 4:
            rgb = [0, x, c]
        elif h < 5:
            rgb = [x, 0, c]
        else:
            rgb = [c, 0, x]

        return [rgb[0] + m, rgb[1] + m, rgb[2] + m]

    def _build_trajectory(self):
        """Build trajectory line set"""
        trajectory_points = []

        for _, pose_matrix in self.keyframes_data:
            trajectory_points.append(pose_matrix[:3, 3])

        if len(trajectory_points) < 2:
            return

        trajectory_points = np.array(trajectory_points)

        # Create line set
        lines = [[i, i + 1] for i in range(len(trajectory_points) - 1)]

        self.trajectory_lines = o3d.geometry.LineSet()
        self.trajectory_lines.points = o3d.utility.Vector3dVector(trajectory_points)
        self.trajectory_lines.lines = o3d.utility.Vector2iVector(lines)

        # Color the trajectory (yellow)
        colors = [[1, 1, 0] for _ in range(len(lines))]
        self.trajectory_lines.colors = o3d.utility.Vector3dVector(colors)

        # Add spheres at keyframe positions
        self.trajectory_spheres = []
        for point in trajectory_points:
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.5)
            sphere.translate(point)
            sphere.paint_uniform_color([1, 0, 0])  # Red spheres
            self.trajectory_spheres.append(sphere)

    def toggle_colorful_keyframes(self, vis):
        """Toggle between colorful keyframes and merged view"""
        self.show_colorful_keyframes = not self.show_colorful_keyframes

        if self.show_colorful_keyframes:
            # Remove merged cloud
            vis.remove_geometry(self.merged_pcd, reset_bounding_box=False)
            # Add individual keyframes
            for pcd in self.colored_keyframes_pcds:
                vis.add_geometry(pcd, reset_bounding_box=False)
            print("✓ Colorful keyframes mode enabled")
        else:
            # Remove individual keyframes
            for pcd in self.colored_keyframes_pcds:
                vis.remove_geometry(pcd, reset_bounding_box=False)
            # Add merged cloud
            vis.add_geometry(self.merged_pcd, reset_bounding_box=False)
            print("✓ Merged point cloud mode enabled")

        return False

    def toggle_trajectory(self, vis):
        """Toggle trajectory visibility"""
        self.show_trajectory = not self.show_trajectory

        if self.show_trajectory:
            if self.trajectory_lines is not None:
                vis.add_geometry(self.trajectory_lines, reset_bounding_box=False)
            for sphere in self.trajectory_spheres:
                vis.add_geometry(sphere, reset_bounding_box=False)
            print("✓ Trajectory enabled")
        else:
            if self.trajectory_lines is not None:
                vis.remove_geometry(self.trajectory_lines, reset_bounding_box=False)
            for sphere in self.trajectory_spheres:
                vis.remove_geometry(sphere, reset_bounding_box=False)
            print("✓ Trajectory disabled")

        return False

    def toggle_height_color(self, vis):
        """Toggle between height-based coloring and uniform color"""
        self.show_height_color = not self.show_height_color

        if not self.show_colorful_keyframes:
            if self.show_height_color:
                self._color_by_height(self.merged_pcd)
                print("✓ Height-based coloring enabled")
            else:
                # Uniform gray color
                points = np.asarray(self.merged_pcd.points)
                self.merged_pcd.colors = o3d.utility.Vector3dVector(
                    np.tile([0.7, 0.7, 0.7], (len(points), 1))
                )
                print("✓ Uniform color enabled")

            vis.update_geometry(self.merged_pcd)

        return False

    def crop_points(self, vis):
        """Crop points using manual selection with visualization"""
        print("\n" + "="*60)
        print("CROP MODE - Manual Selection")
        print("="*60)
        print("使用方法：")
        print("1. 观察点云，记住要保留的区域的坐标范围")
        print("2. 在终端输入裁剪范围")
        print("3. 程序会自动应用裁剪")
        print("="*60)

        # Switch to merged view if needed
        if self.show_colorful_keyframes:
            print("切换到合并点云模式...")
            self.show_colorful_keyframes = False
            for pcd in self.colored_keyframes_pcds:
                vis.remove_geometry(pcd, reset_bounding_box=False)
            vis.add_geometry(self.merged_pcd, reset_bounding_box=False)

        # Get current bounds
        points = np.asarray(self.merged_pcd.points)
        print(f"\n当前点云范围:")
        print(f"  X: [{points[:, 0].min():.2f}, {points[:, 0].max():.2f}]")
        print(f"  Y: [{points[:, 1].min():.2f}, {points[:, 1].max():.2f}]")
        print(f"  Z: [{points[:, 2].min():.2f}, {points[:, 2].max():.2f}]")

        try:
            print("\n输入要保留的范围 (按Enter使用当前最大范围):")
            x_min = input(f"  X最小值 [{points[:, 0].min():.2f}]: ") or str(points[:, 0].min())
            x_max = input(f"  X最大值 [{points[:, 0].max():.2f}]: ") or str(points[:, 0].max())
            y_min = input(f"  Y最小值 [{points[:, 1].min():.2f}]: ") or str(points[:, 1].min())
            y_max = input(f"  Y最大值 [{points[:, 1].max():.2f}]: ") or str(points[:, 1].max())
            z_min = input(f"  Z最小值 [{points[:, 2].min():.2f}]: ") or str(points[:, 2].min())
            z_max = input(f"  Z最大值 [{points[:, 2].max():.2f}]: ") or str(points[:, 2].max())

            x_min, x_max = float(x_min), float(x_max)
            y_min, y_max = float(y_min), float(y_max)
            z_min, z_max = float(z_min), float(z_max)

            # Apply crop
            mask = (
                    (points[:, 0] >= x_min) & (points[:, 0] <= x_max) &
                    (points[:, 1] >= y_min) & (points[:, 1] <= y_max) &
                    (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
            )

            if np.sum(mask) == 0:
                print("⚠️ 警告: 没有点在指定范围内!")
                return False

            # Update point cloud
            self.merged_pcd.points = o3d.utility.Vector3dVector(points[mask])
            colors = np.asarray(self.merged_pcd.colors)
            self.merged_pcd.colors = o3d.utility.Vector3dVector(colors[mask])

            vis.update_geometry(self.merged_pcd)

            print(f"✓ 裁剪完成! 保留了 {np.sum(mask)}/{len(points)} 个点")

        except ValueError:
            print("❌ 输入格式错误，裁剪取消")
        except Exception as e:
            print(f"❌ 裁剪失败: {e}")

        return False

    def delete_points(self, vis):
        """Delete points using sphere/box selection"""
        print("\n" + "="*60)
        print("DELETE MODE - Manual Selection")
        print("="*60)
        print("删除模式选择:")
        print("  1. 球形删除 (删除指定中心点半径内的所有点)")
        print("  2. 方形删除 (删除指定立方体范围内的所有点)")
        print("  3. 取消")
        print("="*60)

        # Switch to merged view if needed
        if self.show_colorful_keyframes:
            print("切换到合并点云模式...")
            self.show_colorful_keyframes = False
            for pcd in self.colored_keyframes_pcds:
                vis.remove_geometry(pcd, reset_bounding_box=False)
            vis.add_geometry(self.merged_pcd, reset_bounding_box=False)

        try:
            choice = input("选择删除模式 [1/2/3]: ")

            if choice == '1':
                self._delete_sphere(vis)
            elif choice == '2':
                self._delete_box(vis)
            else:
                print("取消删除")

        except Exception as e:
            print(f"❌ 删除失败: {e}")

        return False

    def _delete_sphere(self, vis):
        """Delete points within a sphere"""
        points = np.asarray(self.merged_pcd.points)

        print(f"\n当前点云中心约为:")
        center = points.mean(axis=0)
        print(f"  X: {center[0]:.2f}, Y: {center[1]:.2f}, Z: {center[2]:.2f}")

        print("\n输入要删除的球形区域:")
        cx = float(input(f"  中心X坐标 [{center[0]:.2f}]: ") or center[0])
        cy = float(input(f"  中心Y坐标 [{center[1]:.2f}]: ") or center[1])
        cz = float(input(f"  中心Z坐标 [{center[2]:.2f}]: ") or center[2])
        radius = float(input("  半径: "))

        # Calculate distances
        center_point = np.array([cx, cy, cz])
        distances = np.linalg.norm(points - center_point, axis=1)

        # Keep points outside the sphere
        mask = distances > radius

        if np.sum(~mask) == 0:
            print("⚠️ 警告: 没有点在指定球体内!")
            return

        # Update point cloud
        self.merged_pcd.points = o3d.utility.Vector3dVector(points[mask])
        colors = np.asarray(self.merged_pcd.colors)
        self.merged_pcd.colors = o3d.utility.Vector3dVector(colors[mask])

        vis.update_geometry(self.merged_pcd)

        print(f"✓ 删除完成! 删除了 {np.sum(~mask)}/{len(points)} 个点")

    def _delete_box(self, vis):
        """Delete points within a box"""
        points = np.asarray(self.merged_pcd.points)

        print(f"\n当前点云范围:")
        print(f"  X: [{points[:, 0].min():.2f}, {points[:, 0].max():.2f}]")
        print(f"  Y: [{points[:, 1].min():.2f}, {points[:, 1].max():.2f}]")
        print(f"  Z: [{points[:, 2].min():.2f}, {points[:, 2].max():.2f}]")

        print("\n输入要删除的方形区域:")
        x_min = float(input("  X最小值: "))
        x_max = float(input("  X最大值: "))
        y_min = float(input("  Y最小值: "))
        y_max = float(input("  Y最大值: "))
        z_min = float(input("  Z最小值: "))
        z_max = float(input("  Z最大值: "))

        # Keep points outside the box
        mask = ~(
                (points[:, 0] >= x_min) & (points[:, 0] <= x_max) &
                (points[:, 1] >= y_min) & (points[:, 1] <= y_max) &
                (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
        )

        if np.sum(~mask) == 0:
            print("⚠️ 警告: 没有点在指定方形区域内!")
            return

        # Update point cloud
        self.merged_pcd.points = o3d.utility.Vector3dVector(points[mask])
        colors = np.asarray(self.merged_pcd.colors)
        self.merged_pcd.colors = o3d.utility.Vector3dVector(colors[mask])

        vis.update_geometry(self.merged_pcd)

        print(f"✓ 删除完成! 删除了 {np.sum(~mask)}/{len(points)} 个点")

    def save_point_cloud(self, vis):
        """Save current point cloud to file"""
        filename = "edited_map.pcd"

        if self.show_colorful_keyframes:
            # Save all keyframes
            merged = o3d.geometry.PointCloud()
            for pcd in self.colored_keyframes_pcds:
                merged += pcd
            o3d.io.write_point_cloud(filename, merged)
        else:
            o3d.io.write_point_cloud(filename, self.merged_pcd)

        print(f"✓ Point cloud saved to {filename}")
        return False

    def print_help(self, vis):
        """Print help information"""
        print("\n" + "="*60)
        print("KEYBOARD SHORTCUTS:")
        print("="*60)
        print("  [K] - Toggle colorful keyframes view")
        print("  [T] - Toggle trajectory display")
        print("  [H] - Toggle height-based coloring")
        print("  [C] - Crop mode (keep selected region)")
        print("  [X] - Delete mode (remove selected region)")
        print("  [S] - Save point cloud to file")
        print("  [R] - Reset view")
        print("  [F] - Toggle fullscreen")
        print("  [P] - Print current camera parameters")
        print("  [?] - Show this help")
        print("  [Q/ESC] - Quit")
        print("="*60)
        print("MOUSE CONTROLS:")
        print("="*60)
        print("  Left button - Rotate view")
        print("  Right button - Zoom")
        print("  Middle button - Pan")
        print("  Scroll - Zoom in/out")
        print("="*60 + "\n")
        return False

    def reset_view(self, vis):
        """Reset camera view"""
        vis.reset_view_point(True)
        print("✓ View reset")
        return False

    def print_camera_params(self, vis):
        """Print current camera parameters"""
        param = vis.get_view_control().convert_to_pinhole_camera_parameters()
        print("\n" + "="*60)
        print("CAMERA PARAMETERS:")
        print("="*60)
        print("Extrinsic:")
        print(param.extrinsic)
        print("\nIntrinsic:")
        print(param.intrinsic.intrinsic_matrix)
        print("="*60 + "\n")
        return False

    def visualize(self):
        """Start interactive visualization"""
        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window(window_name="Interactive Bag Map Visualizer",
                          width=1400, height=900)

        # Add geometries
        vis.add_geometry(self.coordinate_frame)
        vis.add_geometry(self.merged_pcd)

        if self.show_trajectory and self.trajectory_lines is not None:
            vis.add_geometry(self.trajectory_lines)
            for sphere in self.trajectory_spheres:
                vis.add_geometry(sphere)

        # Register key callbacks
        vis.register_key_callback(ord('K'), self.toggle_colorful_keyframes)
        vis.register_key_callback(ord('T'), self.toggle_trajectory)
        vis.register_key_callback(ord('H'), self.toggle_height_color)
        vis.register_key_callback(ord('C'), self.crop_points)
        vis.register_key_callback(ord('X'), self.delete_points)
        vis.register_key_callback(ord('S'), self.save_point_cloud)
        vis.register_key_callback(ord('R'), self.reset_view)
        vis.register_key_callback(ord('P'), self.print_camera_params)
        vis.register_key_callback(ord('?'), self.print_help)

        # Set rendering options
        opt = vis.get_render_option()
        opt.point_size = 2.0
        opt.background_color = np.asarray([0.1, 0.1, 0.1])

        # Print initial help
        self.print_help(vis)

        # Run visualization
        vis.run()
        vis.destroy_window()


def read_points_from_pointcloud2(cloud_msg):
    """Convert ROS PointCloud2 message to numpy array"""
    points = []
    point_step = cloud_msg.point_step
    data = cloud_msg.data

    # Find field offsets
    x_offset = y_offset = z_offset = None

    for field in cloud_msg.fields:
        if field.name == 'x':
            x_offset = field.offset
        elif field.name == 'y':
            y_offset = field.offset
        elif field.name == 'z':
            z_offset = field.offset

    if None in (x_offset, y_offset, z_offset):
        print("Warning: Could not find x, y, z fields in point cloud")
        return np.array([])

    # Extract points
    for i in range(0, len(data), point_step):
        if i + 12 <= len(data):
            x = struct.unpack('f', data[i + x_offset:i + x_offset + 4])[0]
            y = struct.unpack('f', data[i + y_offset:i + y_offset + 4])[0]
            z = struct.unpack('f', data[i + z_offset:i + z_offset + 4])[0]

            if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                points.append([x, y, z])

    return np.array(points)


def pose_stamped_to_matrix(pose_stamped):
    """Convert PoseStamped message to 4x4 transformation matrix"""
    position = pose_stamped.pose.position
    orientation = pose_stamped.pose.orientation

    translation = np.array([position.x, position.y, position.z])

    x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
    norm = np.sqrt(x*x + y*y + z*z + w*w)
    if norm > 0:
        x, y, z, w = x/norm, y/norm, z/norm, w/norm

    rotation_matrix = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ])

    transformation = np.eye(4)
    transformation[:3, :3] = rotation_matrix
    transformation[:3, 3] = translation

    return transformation


def transform_points(points, transformation_matrix):
    """Transform points using transformation matrix"""
    if len(points) == 0:
        return points

    ones = np.ones((points.shape[0], 1))
    points_homogeneous = np.hstack([points, ones])
    transformed_points = (transformation_matrix @ points_homogeneous.T).T

    return transformed_points[:, :3]


def load_keyframes_from_bag(bag_path):
    """Load keyframe data from ROS bag file"""
    print(f"Loading map from bag file: {bag_path}")

    try:
        bag = rosbag.Bag(bag_path, 'r')
    except Exception as e:
        print(f"Error opening bag file: {e}")
        return None

    point_clouds = []
    poses = []

    print("Reading keyframe point clouds...")
    for topic, msg, t in bag.read_messages(topics=['/keyframe_pcd']):
        point_clouds.append(msg)

    print("Reading keyframe poses...")
    for topic, msg, t in bag.read_messages(topics=['/keyframe_pose']):
        poses.append(msg)

    bag.close()

    if len(point_clouds) != len(poses):
        print(f"Warning: Number of point clouds ({len(point_clouds)}) != number of poses ({len(poses)})")
        min_len = min(len(point_clouds), len(poses))
        point_clouds = point_clouds[:min_len]
        poses = poses[:min_len]

    print(f"Found {len(point_clouds)} keyframes")

    # Process each keyframe
    keyframes_data = []

    for i, (pcd_msg, pose_msg) in enumerate(zip(point_clouds, poses)):
        print(f"Processing keyframe {i+1}/{len(point_clouds)}")

        # Convert point cloud message to numpy array
        points = read_points_from_pointcloud2(pcd_msg)

        if len(points) == 0:
            print(f"Warning: Empty point cloud for keyframe {i}")
            continue

        # Get transformation matrix from pose
        transformation = pose_stamped_to_matrix(pose_msg)

        # Transform points to global coordinate system
        transformed_points = transform_points(points, transformation)

        keyframes_data.append((transformed_points, transformation))

    if len(keyframes_data) == 0:
        print("Error: No valid keyframes found in bag file")
        return None

    # Print statistics
    total_points = sum(len(points) for points, _ in keyframes_data)
    print(f"\nTotal points (before downsampling): {total_points}")

    all_points = np.vstack([points for points, _ in keyframes_data])
    print(f"Point cloud statistics:")
    print(f"  X range: [{np.min(all_points[:, 0]):.2f}, {np.max(all_points[:, 0]):.2f}]")
    print(f"  Y range: [{np.min(all_points[:, 1]):.2f}, {np.max(all_points[:, 1]):.2f}]")
    print(f"  Z range: [{np.min(all_points[:, 2]):.2f}, {np.max(all_points[:, 2]):.2f}]")

    return keyframes_data


def main():
    parser = argparse.ArgumentParser(
        description="Interactive visualization of map from ROS bag file using Open3D"
    )
    parser.add_argument("bag_path", help="Path to the ROS bag file")
    parser.add_argument("--voxel_size", type=float, default=1.0,
                        help="Voxel size for downsampling (default: 1.0, set to 0 to disable)")

    args = parser.parse_args()

    # Check if bag file exists
    import os
    if not os.path.exists(args.bag_path):
        print(f"Error: Bag file '{args.bag_path}' does not exist")
        sys.exit(1)

    # Load keyframes from bag file
    keyframes_data = load_keyframes_from_bag(args.bag_path)

    if keyframes_data is None:
        print("Failed to load map data")
        sys.exit(1)

    # Create and run interactive visualizer
    visualizer = InteractiveMapVisualizer(keyframes_data, args.voxel_size)
    visualizer.visualize()


if __name__ == "__main__":
    main()