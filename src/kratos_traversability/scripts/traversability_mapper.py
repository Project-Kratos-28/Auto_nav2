#!/usr/bin/env python3
"""Terrain traversability from LiDAR / depth-camera points -> /traversability/grid for Nav2.

Keeps a 2.5D height map (highest point per 10 cm cell) in the odom frame, rolling with the
rover, and rates every cell:

  lethal (100)  a higher cell next to a lower one, with the drop both > max_step and steeper
                than max_slope_deg. Within short_radius every pair counts (rocks, steep slopes,
                crater walls) if the ground already falls that steeply to the next cell: the
                brink and the face, not the flat ground behind the brink (a 3 m wide bridge
                keeps its deck). Farther, up to pair_radius, only when the next cell towards
                the lower one is unseen and a beam passed just beyond it clearly below this
                cell's height: the edge of a drop whose wall is hidden but whose floor is in
                view. Ground hidden behind a crest (a ramp's top seen from below) has no such
                beam and stays unknown. Such an edge is first marked within edge_range of the
                rover (farther, range noise fakes it), then kept. Only the higher cell is marked, so a rock does not grow a ring of lethal
                ground around it, and flat ground before an edge stays drivable.
  hidden (60)   a hole the sensors cannot see into yet: beams passed over the cell so the ground
                there is at least max_drop lower than known ground within drop_radius, but the
                floor itself is out of sight (the near wall of a trench or crater hides it). The
                planner may still go there if nothing else works; when the rover gets close and
                sees the slope, the cell becomes lethal (a cliff) or cheap (a ramp).
  slope (0..40) grows from slope_cost_start_deg to max_slope_deg, so flat ground is preferred.
  unknown (-1)  never seen: TraversabilityLayer leaves the costmap cell as it is.

Ground height is relative: hills, ramps and shallow craters are fine as long as they are not
steep. Nothing here depends on the sensor's height above ground (lidar_z).

Sources are point clouds, each with its own topic and range limits (parameters
sources.<name>.topic etc.); points on the rover's own body (footprint box) are dropped.
Beams for the hidden-drop check are traced from the sensor origin for every ray_stride-th point.

Scans taken while the body tilts faster than max_tilt_rate_deg (rocking over a rock, into a
crater) are skipped: a LiDAR sweep or camera frame then spans several attitudes, and one pose
per scan puts the ground tens of centimetres off at a few metres (Isaac Sim: false lethal
patches around a crater after driving through it).

A cell's height is a weighted mean of the highest point per scan, weighted 1 / (1 + (r/4 m)^2)
by range: far returns are noisy (1 deg of chassis pitch is 20 cm at 12 m), so a close look
overrides them. The total weight is capped (max_weight), so the cell keeps following new data.
Static terrain is assumed. The map is lost behind the rover once it moves farther than size/2
from where it was kept.
"""
import array
import math

import numpy as np
import rclpy
import rclpy.duration
import rclpy.executors
from nav_msgs.msg import OccupancyGrid
from numpy.lib.stride_tricks import sliding_window_view
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def sliding_max(a, r, axis):
    """Max over a (2r+1)-wide window along axis; a padded with -inf, same shape out."""
    pad = [(0, 0), (0, 0)]
    pad[axis] = (r, r)
    padded = np.pad(a, pad, constant_values=-np.inf)
    return sliding_window_view(padded, 2 * r + 1, axis=axis).max(axis=-1)


class TraversabilityMapper(Node):
    def __init__(self):
        super().__init__('traversability_mapper')
        p = self.declare_parameter
        self.frame = p('frame', 'odom').value
        self.robot_frame = p('robot_frame', 'base_link').value
        self.res = p('resolution', 0.1).value
        self.n = int(round(p('size', 60.0).value / self.res))
        self.recenter = p('recenter_distance', 10.0).value
        self.window = int(round(p('window', 30.0).value / self.res / 2))
        self.max_slope = math.radians(p('max_slope_deg', 20.0).value)
        self.max_step = p('max_step', 0.15).value
        self.pair_radius = p('pair_radius', 1.5).value
        self.short_radius = p('short_radius', 0.5).value
        self.edge_range = p('edge_range', 4.0).value
        self.max_drop = p('max_drop', 0.3).value
        self.drop_radius = p('drop_radius', 2.5).value
        self.hidden_cost = int(p('hidden_cost', 60).value)
        self.slope_start = math.radians(p('slope_cost_start_deg', 8.0).value)
        self.slope_cost = p('slope_cost_max', 40.0).value
        self.max_point_height = p('max_point_height', 2.0).value   # above base_link: ignore canopies
        half_l, half_w = p('body_half_length', 0.75).value, p('body_half_width', 0.70).value
        self.body = (half_l, half_w)
        self.max_weight = p('max_weight', 5.0).value
        self.max_tilt_rate = math.radians(p('max_tilt_rate_deg', 10.0).value)

        self.sources = {}
        for name in p('sources', ['lidar']).value:
            src = {
                'topic': p(f'sources.{name}.topic', f'/{name}/points').value,
                'min_range': p(f'sources.{name}.min_range', 0.5).value,
                'max_range': p(f'sources.{name}.max_range', 15.0).value,
                'ray_stride': int(p(f'sources.{name}.ray_stride', 4).value),
                'ray_range': p(f'sources.{name}.ray_range', 12.0).value,
                'msg': None,
            }
            self.create_subscription(PointCloud2, src['topic'],
                                     lambda m, s=src: s.__setitem__('msg', m), qos_profile_sensor_data)
            self.sources[name] = src
            self.get_logger().info(f"source {name}: {src['topic']}")

        self.h = np.full((self.n, self.n), np.nan, np.float32)      # ground height per cell
        self.w = np.zeros((self.n, self.n), np.float32)             # weight behind h
        self.ub = np.full((self.n, self.n), np.inf, np.float32)     # lowest beam over the cell
        self.cost = np.full((self.n, self.n), -1, np.int8)
        self.edge = np.zeros((self.n, self.n), bool)                # hidden-wall edges confirmed close by
        self.skipped = 0
        self.origin = None                                          # (x, y) of cell (0, 0) corner

        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.pub = self.create_publisher(
            OccupancyGrid, p('grid_topic', '/traversability/grid').value,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(1.0 / p('rate', 4.0).value, self.tick)

        # Pairs (di, dj, distance) for the lethal check: 16 directions x a few radii.
        offsets = set()
        radii = list(np.arange(self.res, self.short_radius + 1e-6, self.res))
        radii += list(np.arange(self.short_radius + 0.2, self.pair_radius + 1e-6, 0.2))   # coarser far out
        for r in radii:
            for a in np.arange(0, 2 * math.pi, math.pi / 8):
                offsets.add((int(round(r * math.cos(a) / self.res)), int(round(r * math.sin(a) / self.res))))
        offsets.discard((0, 0))
        self.offsets = []
        for di, dj in sorted(offsets):
            m = max(abs(di), abs(dj))
            step = (int(round(di / m)), int(round(dj / m)))      # first cell towards the partner
            self.offsets.append((di, dj, self.res * math.hypot(di, dj), step))

    # --- grid bookkeeping --------------------------------------------------------------------
    def follow(self, rx, ry):
        size = self.n * self.res
        if self.origin is None:
            self.origin = (round((rx - size / 2) / self.res) * self.res, round((ry - size / 2) / self.res) * self.res)
            return
        cx, cy = self.origin[0] + size / 2, self.origin[1] + size / 2
        if math.hypot(rx - cx, ry - cy) < self.recenter:
            return
        sx, sy = int(round((rx - cx) / self.res)), int(round((ry - cy) / self.res))
        for name, fill in (('h', np.nan), ('w', 0.0), ('ub', np.inf), ('cost', -1), ('edge', False)):
            old = getattr(self, name)
            new = np.full_like(old, fill)
            src_y, dst_y = slice(max(sy, 0), self.n + min(sy, 0)), slice(max(-sy, 0), self.n + min(-sy, 0))
            src_x, dst_x = slice(max(sx, 0), self.n + min(sx, 0)), slice(max(-sx, 0), self.n + min(-sx, 0))
            new[dst_y, dst_x] = old[src_y, src_x]
            setattr(self, name, new)
        self.origin = (self.origin[0] + sx * self.res, self.origin[1] + sy * self.res)

    def cells(self, x, y):
        ix = np.floor((x - self.origin[0]) / self.res).astype(np.int64)
        iy = np.floor((y - self.origin[1]) / self.res).astype(np.int64)
        ok = (ix >= 0) & (iy >= 0) & (ix < self.n) & (iy < self.n)
        return iy * self.n + ix, ok

    # --- input -------------------------------------------------------------------------------
    def tilt_rate(self, stamp):
        """How fast the body's up axis turned over the 0.2 s before stamp (rad/s); 0 if unknown."""
        try:
            a = self.tf.lookup_transform(self.frame, self.robot_frame, stamp)
            b = self.tf.lookup_transform(self.frame, self.robot_frame, stamp - rclpy.duration.Duration(seconds=0.2))
        except TransformException:
            return 0.0
        up_a, up_b = quat_to_matrix(a.transform.rotation)[:, 2], quat_to_matrix(b.transform.rotation)[:, 2]
        return math.acos(max(-1.0, min(1.0, float(up_a @ up_b)))) / 0.2

    def integrate(self, src, robot):
        msg, src['msg'] = src['msg'], None
        try:
            # The sensor's pose when it took the scan: the chassis pitches while driving.
            t = self.tf.lookup_transform(self.frame, msg.header.frame_id, Time.from_msg(msg.header.stamp),
                                         timeout=rclpy.duration.Duration(seconds=0.1))
        except TransformException:
            try:
                t = self.tf.lookup_transform(self.frame, msg.header.frame_id, Time())
            except TransformException as e:
                self.get_logger().warn(f'no TF {msg.header.frame_id} -> {self.frame}: {e}', throttle_duration_sec=5.0)
                return
        if self.tilt_rate(Time.from_msg(msg.header.stamp)) > self.max_tilt_rate:
            self.skipped += 1
            return
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = pts[np.isfinite(pts).all(axis=1)]
        rng = np.linalg.norm(pts, axis=1)
        inside = (rng >= src['min_range']) & (rng <= src['max_range'])
        pts, rng = pts[inside], rng[inside]
        if len(pts) == 0:
            return
        o = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        pts = pts.astype(np.float64) @ quat_to_matrix(t.transform.rotation).T + o

        # Drop the rover's own body and anything far above it.
        rx, ry, rz, c, s = robot
        bx = c * (pts[:, 0] - rx) + s * (pts[:, 1] - ry)
        by = -s * (pts[:, 0] - rx) + c * (pts[:, 1] - ry)
        keep = ~((np.abs(bx) < self.body[0]) & (np.abs(by) < self.body[1]))
        keep &= pts[:, 2] < rz + self.max_point_height
        pts, rng = pts[keep], rng[keep]

        idx, ok = self.cells(pts[:, 0], pts[:, 1])
        idx, z, rng = idx[ok], pts[ok, 2].astype(np.float32), rng[ok].astype(np.float32)
        hf, wf = self.h.reshape(-1), self.w.reshape(-1)
        scan = np.full(hf.shape, -np.inf, np.float32)
        np.maximum.at(scan, idx, z)
        near = np.full(hf.shape, np.inf, np.float32)
        np.minimum.at(near, idx, rng)
        seen = np.nonzero(scan > -np.inf)[0]
        wn = 1.0 / (1.0 + (near[seen] / 4.0) ** 2)
        w_old = wf[seen]
        h_old = np.where(w_old > 0, hf[seen], 0.0)
        hf[seen] = (h_old * w_old + scan[seen] * wn) / (w_old + wn)
        wf[seen] = np.minimum(w_old + wn, self.max_weight)

        # Beams: the ground under a beam is lower than the beam.
        if src['ray_stride'] > 0:
            rays = pts[::src['ray_stride']] - o
            length = np.hypot(rays[:, 0], rays[:, 1])
            rays, length = rays[(length > 0.3) & (length < src['ray_range'])], length[(length > 0.3) & (length < src['ray_range'])]
            if len(rays):
                step = self.res / 2
                k = np.arange(1, int(length.max() / step) + 1) * step          # distance along the ground
                frac = k[None, :] / length[:, None]
                valid = k[None, :] < length[:, None] - 0.15                   # stop short of the hit cell
                xs = o[0] + frac * rays[:, 0:1]
                ys = o[1] + frac * rays[:, 1:2]
                zs = o[2] + frac * rays[:, 2:3]
                idx, ok = self.cells(xs[valid], ys[valid])
                np.minimum.at(self.ub.reshape(-1), idx[ok], zs[valid][ok].astype(np.float32))

    # --- rating ------------------------------------------------------------------------------
    def rate(self, rx, ry):
        ci = int((rx - self.origin[0]) / self.res)
        cj = int((ry - self.origin[1]) / self.res)
        i0, i1 = max(ci - self.window, 0), min(ci + self.window, self.n)
        j0, j1 = max(cj - self.window, 0), min(cj + self.window, self.n)
        ny, nx = j1 - j0, i1 - i0
        Pp = int(math.ceil(self.pair_radius / self.res)) + 1     # margin for the pair check
        Pd = int(round(self.drop_radius / self.res))              # margin for the hidden-drop check
        P = Pp + Pd
        # Block with P cells of margin on every side (NaN / inf outside the grid).
        H = np.full((ny + 2 * P, nx + 2 * P), np.nan, np.float32)
        UB = np.full_like(H, np.inf)
        pj0, pj1, pi0, pi1 = max(j0 - P, 0), min(j1 + P, self.n), max(i0 - P, 0), min(i1 + P, self.n)
        dst = (slice(pj0 - j0 + P, pj1 - j0 + P), slice(pi0 - i0 + P, pi1 - i0 + P))
        H[dst] = self.h[pj0:pj1, pi0:pi1]
        UB[dst] = self.ub[pj0:pj1, pi0:pi1]

        with np.errstate(invalid='ignore'):
            # Lethal over the core plus Pd (the hidden-drop check needs it there too).
            my, mx = ny + 2 * Pd, nx + 2 * Pd
            Hm = H[Pp:Pp + my, Pp:Pp + mx]
            short_m = np.zeros((my, mx), bool)
            far_m = np.zeros((my, mx), bool)
            tan = math.tan(self.max_slope)
            for di, dj, d, (si, sj) in self.offsets:
                drop = Hm - H[Pp + dj:Pp + dj + my, Pp + di:Pp + di + mx]
                cond = (drop > self.max_step) & (drop > d * tan)
                nxt = H[Pp + sj:Pp + sj + my, Pp + si:Pp + si + mx]
                if d > self.short_radius:
                    # A hidden wall: the next cell is unseen and, within 3 cells past this one, a
                    # beam went by clearly below this cell's height (on its way down to the
                    # floor). Beams over ground hidden behind a crest (a ramp's deck seen from
                    # below) stay above it: no evidence of a drop.
                    beam = UB[Pp + sj:Pp + sj + my, Pp + si:Pp + si + mx]
                    for k in (2, 3):
                        beam = np.minimum(beam, UB[Pp + k * sj:Pp + k * sj + my, Pp + k * si:Pp + k * si + mx])
                    far_m |= cond & np.isnan(nxt) & (beam < Hm - 0.05)
                else:
                    # The steep part must start at this cell. A flat deck 0.4 m in from its edge
                    # is not lethal, only the brink and the face are.
                    short_m |= cond & (np.isnan(nxt) | (Hm - nxt > math.hypot(si, sj) * self.res * tan))

            # A hidden wall is only confirmed from close by (its floor comes into view within
            # ~2 m of the edge); from far off, range noise fakes it. Once confirmed it stays
            # while the heights still say so.
            gj0, gi0 = j0 - Pd, i0 - Pd
            cj0, cj1, ci0, ci1 = max(gj0, 0), min(gj0 + my, self.n), max(gi0, 0), min(gi0 + mx, self.n)
            block = (slice(cj0 - gj0, cj1 - gj0), slice(ci0 - gi0, ci1 - gi0))
            ys = self.origin[1] + (np.arange(cj0, cj1) + 0.5) * self.res - ry
            xs = self.origin[0] + (np.arange(ci0, ci1) + 0.5) * self.res - rx
            confirmed = far_m[block] & (self.edge[cj0:cj1, ci0:ci1] | (np.hypot(xs[None, :], ys[:, None]) < self.edge_range))
            self.edge[cj0:cj1, ci0:ci1] = confirmed
            far_m[:] = False
            far_m[block] = confirmed
            lethal_m = short_m | far_m

            # Hidden drops: beams passed so low over an unseen cell that its ground is more than
            # max_drop below the ground (known, not an obstacle) within drop_radius.
            ground = np.where(~np.isnan(Hm) & ~lethal_m, Hm, -np.inf)
            near = sliding_max(sliding_max(ground, Pd, 0), Pd, 1)[Pd:Pd + ny, Pd:Pd + nx]
            core = (slice(P, P + ny), slice(P, P + nx))
            Hc = H[core]
            known = ~np.isnan(Hc)
            lethal = lethal_m[Pd:Pd + ny, Pd:Pd + nx]
            hidden = ~known & np.isfinite(UB[core]) & (near - UB[core] > self.max_drop)
            hidden |= known & ~lethal & (sliding_max(sliding_max(np.where(hidden, 1.0, -np.inf), 3, 0), 3, 1) > 0)

            # Slope over +-0.3 m, for the soft cost.
            k = 3
            gx = (H[P:P + ny, P + k:P + k + nx] - H[P:P + ny, P - k:P - k + nx]) / (2 * k * self.res)
            gy = (H[P + k:P + k + ny, P:P + nx] - H[P - k:P - k + ny, P:P + nx]) / (2 * k * self.res)
            slope = np.arctan(np.nan_to_num(np.hypot(gx, gy)))
        soft = np.clip((slope - self.slope_start) / (self.max_slope - self.slope_start), 0, 1) * self.slope_cost

        out = np.full((ny, nx), -1, np.int16)
        out[known] = soft[known].astype(np.int16)
        out[hidden] = np.maximum(out[hidden], self.hidden_cost)
        out[lethal] = 100
        self.cost[j0:j1, i0:i1] = out.astype(np.int8)

    def tick(self):
        try:
            t = self.tf.lookup_transform(self.frame, self.robot_frame, Time())
        except TransformException as e:
            self.get_logger().warn(f'no TF {self.robot_frame} -> {self.frame}: {e}', throttle_duration_sec=5.0)
            return
        rx, ry, rz = t.transform.translation.x, t.transform.translation.y, t.transform.translation.z
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.follow(rx, ry)
        fresh = False
        for src in self.sources.values():
            if src['msg'] is not None:
                self.integrate(src, (rx, ry, rz, math.cos(yaw), math.sin(yaw)))
                fresh = True
        if not fresh:
            return
        if self.skipped:
            self.get_logger().info(f'{self.skipped} scans skipped so far (body tilting fast)', throttle_duration_sec=20.0)
        self.rate(rx, ry)

        grid = OccupancyGrid()
        grid.header.frame_id = self.frame
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = self.res
        grid.info.width = grid.info.height = self.n
        grid.info.origin.position.x, grid.info.origin.position.y = self.origin
        grid.info.origin.orientation.w = 1.0
        grid.data = array.array('b', self.cost.tobytes())
        self.pub.publish(grid)


def main():
    rclpy.init()
    node = TraversabilityMapper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
