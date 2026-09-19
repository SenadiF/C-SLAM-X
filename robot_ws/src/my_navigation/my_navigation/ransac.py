import rclpy
from rclpy.node import Node
import numpy as np
import math

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class RansacIcpMapAligner(Node):



    def __init__(self):
        super().__init__('ransac_icp_map_aligner')

        self.declare_parameter('ransac_iterations', 200)
        self.declare_parameter('ransac_inlier_threshold', 0.15)  # meters
        self.declare_parameter('icp_max_iterations', 50)
        self.declare_parameter('icp_tolerance', 1e-4)
        self.declare_parameter('min_points_required', 20)
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('warm_start_inlier_ratio', 0.3)

        self.ransac_iterations = self.get_parameter('ransac_iterations').value
        self.ransac_inlier_threshold = self.get_parameter('ransac_inlier_threshold').value
        self.warm_start_inlier_ratio = self.get_parameter('warm_start_inlier_ratio').value
        self.icp_max_iterations = self.get_parameter('icp_max_iterations').value
        self.icp_tolerance = self.get_parameter('icp_tolerance').value
        self.min_points_required = self.get_parameter('min_points_required').value
        publish_rate = self.get_parameter('publish_rate').value

        self.map1_msg = None
        self.map2_msg = None

        self.last_transform = None      # (theta, tx, ty) most recent solved alignment
        self.last_alignment_error = None
        self.last_alignment_time_sec = None

        self.map1_sub = self.create_subscription(
            OccupancyGrid, '/robot1/map', self.map1_callback, 1)
        self.map2_sub = self.create_subscription(
            OccupancyGrid, '/robot2/map', self.map2_callback, 1)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(1.0 / publish_rate, self.try_align)

        self.get_logger().info('RANSAC+ICP map aligner started.')

    def map1_callback(self, msg):
        self.map1_msg = msg

    def map2_callback(self, msg):
        self.map2_msg = msg

    # ------------------------------------------------------------------
    # Convert an occupancy grid's occupied cells to a world-coordinate
    # 2D point cloud.
    # ------------------------------------------------------------------
    def grid_to_points(self, grid_msg):
        info = grid_msg.info
        width = info.width
        height = info.height
        resolution = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y

        data = np.array(grid_msg.data, dtype=np.int16).reshape((height, width))
        occupied_idx = np.argwhere(data >= 50)  # (row, col) = (y, x)

        if len(occupied_idx) == 0:
            return np.empty((0, 2))

        xs = origin_x + (occupied_idx[:, 1] + 0.5) * resolution
        ys = origin_y + (occupied_idx[:, 0] + 0.5) * resolution
        return np.stack([xs, ys], axis=1)

    # ------------------------------------------------------------------
    # RANSAC: robustly estimate coarse (theta, tx, ty) between the two
    # point sets.
    #
    # A source pair and a target pair can only correspond to the same
    # real-world features if the distance between them is preserved by
    # the (rotation + translation only, no scaling) transform we're
    # solving for. So instead of picking points independently from each
    # cloud - which pairs up unrelated points and fits noise - each
    # iteration picks a random pair from the source cloud and only
    # considers target pairs whose separation distance matches, which
    # makes every candidate transform geometrically plausible rather
    # than arbitrary.
    # ------------------------------------------------------------------
    def ransac_align(self, source_pts, target_pts):
        rng = np.random.default_rng()

        n_src = len(source_pts)
        n_tgt = len(target_pts)

        if n_src < 2 or n_tgt < 2:
            return None

        # Subsample both clouds for the pairwise-distance search - an
        # O(n^2) distance table over the full map would be too slow.
        max_pts_for_pairs = 80
        src_sub = source_pts
        if n_src > max_pts_for_pairs:
            src_sub = source_pts[rng.choice(n_src, max_pts_for_pairs, replace=False)]

        tgt_sub = target_pts
        if n_tgt > max_pts_for_pairs:
            tgt_sub = target_pts[rng.choice(n_tgt, max_pts_for_pairs, replace=False)]

        n_src_sub = len(src_sub)

        # Precompute target pairwise distances once, so each iteration
        # just looks up matching-distance pairs instead of recomputing.
        tgt_diffs = tgt_sub[:, None, :] - tgt_sub[None, :, :]
        tgt_dists = np.sqrt(np.sum(tgt_diffs ** 2, axis=2))
        distance_tolerance = self.ransac_inlier_threshold * 2.0

        best_inliers = 0
        best_transform = None

        for _ in range(self.ransac_iterations):
            i, j = rng.choice(n_src_sub, size=2, replace=False)
            p1, p2 = src_sub[i], src_sub[j]
            d_src = np.linalg.norm(p1 - p2)

            if d_src < 1e-3:
                continue

            candidates = np.argwhere(np.abs(tgt_dists - d_src) < distance_tolerance)
            candidates = candidates[candidates[:, 0] != candidates[:, 1]]
            if len(candidates) == 0:
                continue

            k, l = candidates[rng.integers(len(candidates))]
            q1, q2 = tgt_sub[k], tgt_sub[l]

            # Either point of the target pair could correspond to
            # either point of the source pair - try both orderings.
            for qa, qb in ((q1, q2), (q2, q1)):
                transform = self.estimate_rigid_transform(
                    np.array([p1, p2]), np.array([qa, qb])
                )
                if transform is None:
                    continue

                theta, tx, ty = transform
                transformed = self.apply_transform(source_pts, theta, tx, ty)
                inliers = self.count_inliers(transformed, target_pts)

                if inliers > best_inliers:
                    best_inliers = inliers
                    best_transform = transform

        if best_transform is None:
            return None

        self.get_logger().info(
            f'RANSAC best transform inliers: {best_inliers}/{n_src}'
        )
        return best_transform

    def estimate_rigid_transform(self, src_sample, tgt_sample):
        """
        Estimate the rigid transform (theta, tx, ty) that best maps
        src_sample onto tgt_sample using centroid alignment + SVD
        rotation estimation (standard Kabsch-style approach, applied
        here to small RANSAC samples).
        """
        if len(src_sample) != len(tgt_sample):
            return None

        src_centroid = src_sample.mean(axis=0)
        tgt_centroid = tgt_sample.mean(axis=0)

        src_centered = src_sample - src_centroid
        tgt_centered = tgt_sample - tgt_centroid

        H = src_centered.T @ tgt_centered
        try:
            U, _, Vt = np.linalg.svd(H)
        except np.linalg.LinAlgError:
            return None

        R = Vt.T @ U.T

        # Reflection correction
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T

        theta = math.atan2(R[1, 0], R[0, 0])
        t = tgt_centroid - R @ src_centroid

        return (theta, t[0], t[1])

    def apply_transform(self, points, theta, tx, ty):
        c, s = math.cos(theta), math.sin(theta)
        R = np.array([[c, -s], [s, c]])
        return (R @ points.T).T + np.array([tx, ty])

    def count_inliers(self, transformed_pts, target_pts):
        # For speed, subsample target points into a coarse grid lookup
        # rather than a full O(n*m) nearest-neighbour search.
        if len(target_pts) == 0 or len(transformed_pts) == 0:
            return 0

        inliers = 0
        # Subsample both sets for speed on large maps.
        max_check = 300
        sample = transformed_pts
        if len(sample) > max_check:
            idx = np.random.choice(len(sample), max_check, replace=False)
            sample = sample[idx]

        for p in sample:
            d = np.min(np.sum((target_pts - p) ** 2, axis=1))
            if math.sqrt(d) < self.ransac_inlier_threshold:
                inliers += 1

        return inliers

    # ------------------------------------------------------------------
    # ICP: refine the RANSAC estimate by iterative closest-point
    # minimization.
    # ------------------------------------------------------------------
    def icp_refine(self, source_pts, target_pts, initial_transform):
        theta, tx, ty = initial_transform
        prev_error = None

        # Subsample for performance on large maps.
        max_points = 500
        src = source_pts
        if len(src) > max_points:
            idx = np.random.choice(len(src), max_points, replace=False)
            src = src[idx]

        for iteration in range(self.icp_max_iterations):
            transformed = self.apply_transform(src, theta, tx, ty)

            # Find nearest target point for each transformed source point.
            correspondences = []
            total_error = 0.0
            for p in transformed:
                d2 = np.sum((target_pts - p) ** 2, axis=1)
                nearest_idx = np.argmin(d2)
                correspondences.append(target_pts[nearest_idx])
                total_error += d2[nearest_idx]

            correspondences = np.array(correspondences)
            mean_error = total_error / len(src)

            # Re-estimate the rigid transform from source -> matched targets.
            new_transform = self.estimate_rigid_transform(src, correspondences)
            if new_transform is None:
                break

            theta, tx, ty = new_transform

            if prev_error is not None and abs(prev_error - mean_error) < self.icp_tolerance:
                self.get_logger().info(
                    f'ICP converged after {iteration + 1} iterations, '
                    f'mean error={mean_error:.4f} m^2'
                )
                break

            prev_error = mean_error

        return (theta, tx, ty), prev_error

    # ------------------------------------------------------------------
    # Main alignment attempt, run periodically.
    # ------------------------------------------------------------------
    def try_align(self):
        if self.map1_msg is None or self.map2_msg is None:
            self.get_logger().info('Waiting for both robot maps...', throttle_duration_sec=5.0)
            return

        source_pts = self.grid_to_points(self.map2_msg)   # align robot2 onto robot1
        target_pts = self.grid_to_points(self.map1_msg)

        if len(source_pts) < self.min_points_required or len(target_pts) < self.min_points_required:
            self.get_logger().info(
                f'Not enough occupied points yet (robot1={len(target_pts)}, '
                f'robot2={len(source_pts)}) - need at least {self.min_points_required}.',
                throttle_duration_sec=5.0
            )
            return

        import time
        start_time = time.time()

        # Warm start: if the previous cycle's solved transform still
        # explains the current maps reasonably well, refine it directly
        # instead of re-running RANSAC's random search from scratch. This
        # is what keeps the alignment converging/stable across cycles
        # instead of jumping to a different, unrelated local optimum
        # every single time (each fresh RANSAC search can legitimately
        # find a different plausible-looking fit).
        coarse_transform = None

        if self.last_transform is not None:
            theta, tx, ty = self.last_transform
            transformed = self.apply_transform(source_pts, theta, tx, ty)
            warm_start_inliers = self.count_inliers(transformed, target_pts)
            checked = min(len(source_pts), 300)
            if warm_start_inliers / checked > self.warm_start_inlier_ratio:
                coarse_transform = self.last_transform

        if coarse_transform is None:
            coarse_transform = self.ransac_align(source_pts, target_pts)
            if coarse_transform is None:
                self.get_logger().warn('RANSAC failed to find a transform this cycle.')
                return

        refined_transform, final_error = self.icp_refine(
            source_pts, target_pts, coarse_transform
        )

        elapsed = time.time() - start_time

        self.last_transform = refined_transform
        self.last_alignment_error = final_error
        self.last_alignment_time_sec = elapsed

        theta, tx, ty = refined_transform

        self.get_logger().info(
            f'Alignment result: theta={math.degrees(theta):.2f} deg, '
            f'tx={tx:.3f} m, ty={ty:.3f} m, '
            f'error={final_error:.4f} m^2, time={elapsed:.3f} s'
        )

        self.broadcast_transform(theta, tx, ty)

    def broadcast_transform(self, theta, tx, ty):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'robot1/map'
        t.child_frame_id = 'robot2/map'

        t.transform.translation.x = tx
        t.transform.translation.y = ty
        t.transform.translation.z = 0.0

        t.transform.rotation.z = math.sin(theta / 2.0)
        t.transform.rotation.w = math.cos(theta / 2.0)

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = RansacIcpMapAligner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()