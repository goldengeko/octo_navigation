#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import open3d as o3d
from sensor_msgs_py import point_cloud2
import math
from typing import Set, Tuple
import traceback


class PointCloudCleaner(Node):
    def __init__(self):
        super().__init__('point_cloud_cleaner')

        self.sub = self.create_subscription(
            PointCloud2,
            '/octomap_point_cloud_centers',
            self.cloud_callback,
            10
        )

        self.pub = self.create_publisher(
            PointCloud2,
            '/centerpoints_filtered',
            10
        )

        self.get_logger().info('✅ PointCloud cleaner node started.')

        # Parameters (tunable)
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('inflation_radius', 0.05)
        self.declare_parameter('max_voxels', 5000000)
        self.declare_parameter('hole_fill_enabled', True)
        self.declare_parameter('hole_fill_iterations', 2)
        self.declare_parameter('hole_fill_min_neighbors', 5)
        # Don't inflate by default (slow); enable fast top-down hole fill for floors
        self.declare_parameter('inflate_enabled', False)
        self.declare_parameter('top_down_fill', True)
        self.declare_parameter('top_down_iterations', 2)
        self.declare_parameter('publish_original_on_skip', True)

    def cloud_callback(self, msg: PointCloud2):
        try:
            # Convert ROS2 PointCloud2 -> numpy array
            points = np.array([
                [p[0], p[1], p[2]]
                for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            ])

            if points.size == 0 or len(points) == 0:
                self.get_logger().warning('Received empty point cloud.')
                return

            # Convert to Open3D point cloud
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)

            # --- CLEANING ---

            # 1️⃣ Remove isolated points (Statistical Outlier Removal)
            # Adjust nb_neighbors and std_ratio for aggressiveness
            cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)
            cleaned_pcd = pcd.select_by_index(ind)

            # 2️⃣ Voxel occupancy filling + inflation
            voxel_size = float(self.get_parameter('voxel_size').value)
            inflation_radius = float(self.get_parameter('inflation_radius').value)
            max_voxels = int(self.get_parameter('max_voxels').value)

            def fill_and_inflate(points: np.ndarray, voxel_size: float, inflation_radius: float,
                                 max_voxels: int, hole_fill_enabled: bool,
                                 hole_fill_iterations: int, hole_fill_min_neighbors: int,
                                 inflate_enabled: bool, top_down_fill: bool, top_down_iterations: int) -> np.ndarray:
                if points.size == 0:
                    return np.zeros((0, 3), dtype=np.float32)

                # Compute voxel indices relative to grid origin
                mins = points.min(axis=0)
                maxs = points.max(axis=0)
                grid_shape = np.ceil((maxs - mins) / voxel_size).astype(int) + 1

                # Map points to integer voxel coordinates
                indices = np.floor((points - mins) / voxel_size).astype(int)

                occupied: Set[Tuple[int, int, int]] = set(map(tuple, indices))

                # Determine dilation radius in voxels
                steps = max(0, int(math.ceil(inflation_radius / voxel_size)))

                # Estimate resulting voxel count and bail out if too large
                estimate = len(occupied) * ((2 * steps + 1) ** 3)
                if estimate > max_voxels:
                    self.get_logger().warn(
                        f'Voxel inflation would create ~{estimate} voxels > max_voxels({max_voxels}). '
                        'Skipping inflation to avoid memory blowup.'
                    )
                    steps = 0

                # Fast top-down (2D) hole-filling on floor surface if requested
                if top_down_fill:
                    # Build top-z map per (x,y) voxel
                    mins = points.min(axis=0)
                    maxs = points.max(axis=0)
                    sx = int(np.ceil((maxs[0] - mins[0]) / voxel_size)) + 1
                    sy = int(np.ceil((maxs[1] - mins[1]) / voxel_size)) + 1

                    top_z = np.full((sx, sy), np.nan, dtype=float)
                    idx_xy = np.floor((points[:, :2] - mins[:2]) / voxel_size).astype(int)
                    for (ix, iy), z in zip(map(tuple, idx_xy), points[:, 2]):
                        if 0 <= ix < sx and 0 <= iy < sy:
                            if np.isnan(top_z[ix, iy]) or z > top_z[ix, iy]:
                                top_z[ix, iy] = z

                    occ2d = ~np.isnan(top_z)

                    # Iterative fill: empty cells that have >= min_neighbors occupied neighbors become filled
                    neigh_offsets_2d = [(dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if not (dx == 0 and dy == 0)]
                    td_iters = max(0, int(top_down_iterations))
                    for _ in range(td_iters):
                        new_occ2d = occ2d.copy()
                        empty_idx = np.transpose(np.nonzero(~occ2d))
                        for x, y in empty_idx:
                            cnt = 0
                            max_neighbor_z = -np.inf
                            for dx, dy in neigh_offsets_2d:
                                nx, ny = x + dx, y + dy
                                if 0 <= nx < sx and 0 <= ny < sy and occ2d[nx, ny]:
                                    cnt += 1
                                    if not np.isnan(top_z[nx, ny]):
                                        if top_z[nx, ny] > max_neighbor_z:
                                            max_neighbor_z = top_z[nx, ny]
                                    if cnt >= hole_fill_min_neighbors:
                                        new_occ2d[x, y] = True
                                        # assign a z to the filled cell using highest neighbor z
                                        top_z[x, y] = max_neighbor_z if max_neighbor_z != -np.inf else (mins[2] + 0.5 * voxel_size)
                                        break
                        occ2d = new_occ2d

                    # Build centers using top_z for z coordinate
                    centers = []
                    xs, ys = np.nonzero(occ2d)
                    for ix, iy in zip(xs, ys):
                        cx = mins[0] + (ix + 0.5) * voxel_size
                        cy = mins[1] + (iy + 0.5) * voxel_size
                        cz = top_z[ix, iy]
                        if np.isnan(cz):
                            cz = mins[2] + 0.5 * voxel_size
                        centers.append((cx, cy, cz))

                    centers_arr = np.asarray(centers, dtype=np.float32)
                    #self.get_logger().info(f'top_down fill: produced {centers_arr.shape[0]} points (grid {sx}x{sy})')
                    return centers_arr

                # Optional hole-filling: iterate and fill empty voxels that have many occupied neighbors
                if hole_fill_enabled:
                    # Build boolean occupancy grid to allow fast neighbor counting
                    sx, sy, sz = grid_shape.tolist()
                    # Avoid allocating enormous arrays by checking rough size
                    rough_size = int(sx) * int(sy) * int(sz)
                    if rough_size <= max_voxels:
                        occ = np.zeros((sx, sy, sz), dtype=bool)
                        for ix, iy, iz in occupied:
                            if 0 <= ix < sx and 0 <= iy < sy and 0 <= iz < sz:
                                occ[ix, iy, iz] = True

                        # Neighbor offsets (26-connected)
                        neigh_offsets = [(dx, dy, dz)
                                         for dx in (-1, 0, 1)
                                         for dy in (-1, 0, 1)
                                         for dz in (-1, 0, 1)
                                         if not (dx == 0 and dy == 0 and dz == 0)]

                        for _ in range(max(0, int(hole_fill_iterations))):
                            new_occ = occ.copy()
                            # iterate only over empty voxels that are within bounding box of interest
                            empty_idx = np.transpose(np.nonzero(~occ))
                            for x, y, z in empty_idx:
                                cnt = 0
                                for dx, dy, dz in neigh_offsets:
                                    nx, ny, nz = x + dx, y + dy, z + dz
                                    if 0 <= nx < sx and 0 <= ny < sy and 0 <= nz < sz:
                                        if occ[nx, ny, nz]:
                                            cnt += 1
                                            # early exit
                                            if cnt >= hole_fill_min_neighbors:
                                                new_occ[x, y, z] = True
                                                break
                            occ = new_occ

                        # Convert occ back to occupied set
                        occupied = set()
                        filled_indices = np.transpose(np.nonzero(occ))
                        for ix, iy, iz in filled_indices:
                            occupied.add((int(ix), int(iy), int(iz)))
                    else:
                        # Grid too large to perform hole filling safely
                        self.get_logger().warn(
                            f"Skipping hole filling: grid would allocate ~{rough_size} voxels > max_voxels({max_voxels})"
                        )

                # Fast inflation (dilation) using a spherical kernel when requested
                if steps > 0:
                    sx, sy, sz = grid_shape.tolist()
                    rough_size = int(sx) * int(sy) * int(sz)
                    if rough_size <= max_voxels:
                        # Build boolean occupancy grid
                        occ = np.zeros((sx, sy, sz), dtype=bool)
                        for ix, iy, iz in occupied:
                            if 0 <= ix < sx and 0 <= iy < sy and 0 <= iz < sz:
                                occ[ix, iy, iz] = True

                        # Build spherical kernel offsets once
                        rng = np.arange(-steps, steps + 1)
                        dx, dy, dz = np.meshgrid(rng, rng, rng, indexing='ij')
                        dist2 = dx ** 2 + dy ** 2 + dz ** 2
                        mask = dist2 <= (steps ** 2 + 1e-9)
                        offsets = np.vstack((dx[mask].ravel(), dy[mask].ravel(), dz[mask].ravel())).T

                        # Pad occupancy grid to allow simple slicing without wrap-around
                        pad = steps
                        pad_occ = np.pad(occ, pad_width=pad, mode='constant', constant_values=False)
                        inflated = np.zeros_like(occ)

                        # Apply each offset as a slice (C-level operations, fast)
                        for ox, oy, oz in offsets:
                            sx0 = pad + int(ox)
                            sy0 = pad + int(oy)
                            sz0 = pad + int(oz)
                            inflated |= pad_occ[sx0:sx0 + sx, sy0:sy0 + sy, sz0:sz0 + sz]

                        # Convert inflated boolean grid back to occupied set
                        occupied = set(map(tuple, np.transpose(np.nonzero(inflated))))
                    else:
                        # Grid too large; avoid expensive inflation
                        self.get_logger().warn(
                            f"Skipping inflation: grid would allocate ~{rough_size} voxels > max_voxels({max_voxels})"
                        )

                # Convert voxel indices back to coordinates (voxel centers)
                centers = []
                for ix, iy, iz in occupied:
                    cx = mins[0] + (ix + 0.5) * voxel_size
                    cy = mins[1] + (iy + 0.5) * voxel_size
                    cz = mins[2] + (iz + 0.5) * voxel_size
                    centers.append((cx, cy, cz))

                if len(centers) == 0:
                    centers_arr = np.zeros((0, 3), dtype=np.float32)
                else:
                    centers_arr = np.asarray(centers, dtype=np.float32)

                self.get_logger().info(f'fill_and_inflate: produced {centers_arr.shape[0]} voxel centers (grid size {grid_shape.tolist()})')
                return centers_arr

            cleaned_points = np.asarray(cleaned_pcd.points)
            hole_fill_enabled = bool(self.get_parameter('hole_fill_enabled').value)
            hole_fill_iterations = int(self.get_parameter('hole_fill_iterations').value)
            hole_fill_min_neighbors = int(self.get_parameter('hole_fill_min_neighbors').value)
            inflate_enabled = bool(self.get_parameter('inflate_enabled').value)
            top_down_fill = bool(self.get_parameter('top_down_fill').value)
            top_down_iterations = int(self.get_parameter('top_down_iterations').value)
            publish_original_on_skip = bool(self.get_parameter('publish_original_on_skip').value)

            filtered_points = fill_and_inflate(
                cleaned_points,
                voxel_size,
                inflation_radius,
                max_voxels,
                hole_fill_enabled,
                hole_fill_iterations,
                hole_fill_min_neighbors,
                inflate_enabled,
                top_down_fill,
                top_down_iterations,
            )

            # If nothing was produced (e.g., filling skipped) and user wants original, fallback
            if filtered_points.size == 0 and publish_original_on_skip:
                if cleaned_points.size == 0:
                    filtered_points = np.zeros((0, 3), dtype=np.float32)
                else:
                    # use cleaned points as Nx3 float32
                    filtered_points = cleaned_points.astype(np.float32)

            # Create new PointCloud2 message
            # Ensure filtered_points is an (N,3) array; create list for ROS message
            if filtered_points.size == 0:
                filtered_list = []
            else:
                filtered_list = filtered_points.tolist()

            filtered_msg = point_cloud2.create_cloud_xyz32(msg.header, filtered_list)

            # Publish
            self.pub.publish(filtered_msg)
            #self.get_logger().info(f'Published cleaned cloud: raw={points.shape[0]}, cleaned={cleaned_points.shape[0]}, filtered={len(filtered_list)}')
        except Exception as e:
            self.get_logger().error(f'Exception in cloud_callback: {e}')
            tb = traceback.format_exc()
            self.get_logger().error(tb)
            return

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudCleaner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
