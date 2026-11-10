# pcd2pgm live-mode test suite

Not wired into CMake/colcon - these are standalone scripts, run directly with
python3 against a `pcd2pgm_node` started from `config/pcd2pgm_live.yaml`
(`run_all.sh` does that for you). Written 2026-09-15, before the first rover
live test. The synthetic GLIM publisher matches GLIM's real QoS, rate and
"the cloud only grows" behaviour.

## Running

Needs ROS 2 Humble and a built `pcd2pgm` (Linux; not the Mac).

```bash
# From a clone of this repo, after `colcon build --packages-select pcd2pgm`:
PCD2PGM_SETUP=$PWD/install/setup.bash bash src/pcd2pgm/test/run_all.sh

# On the team VM (uses ~/Kratos/pcd2pgm_live_ws by default):
bash ~/Kratos/pcd2pgm_live_ws/src/pcd2pgm/test/run_all.sh
```

The script sets `ROS_DOMAIN_ID=44` itself, so it does not see (or disturb) a
live system on the default domain. It does `pkill -f pcd2pgm_node` before and
after, so do not run it while a real pcd2pgm is running on the same machine.

Exits 0 iff every assertion in every scenario passed. Logs go to a fresh
`mktemp -d /tmp/pcd2pgm_livetest.XXXXXX` directory (printed at the top and
bottom of the run).

Runtime: about 25-30s total (Part 1 ~15s, Part 2 ~10s, dominated by
deliberate `sleep`s for QoS discovery and the two node startups).

## Files

- `common.py` - shared helpers: QoS profiles matching the REAL GLIM
  publisher (reliable + transient_local, depth 1) and pcd2pgm's real `/map`
  publisher QoS; `PointCloud2` construction; fixed-grid cell-index math;
  point-cloud generators (`room_wall_points`, `voxel_cluster`); log-tailing
  helpers; a tiny `Failures` accumulator (`.check(name, cond, detail)`,
  `.finish()` -> exit code).
- `test_main.py` - scenarios A-F and H, run against ONE long-lived node
  instance (faster, and exercises state carried across updates).
- `test_late_join_qos.py` - scenario G. Manages its own node process
  lifecycle (twice) because it needs precise control over start order.
- `run_all.sh` - orchestrates both, does its own process cleanup
  (`pkill -f pcd2pgm_node` etc.) before and after, aggregates exit codes.

## What's covered, and why each scenario is built the way it is

**A - fixed grid info, even as the input cloud grows.** Publishes a small
room, checks `/map` info; publishes that SAME room's points plus a second,
disjoint room (i.e. the cloud only grows, like GLIM's real accumulated
map), checks info again. Asserts width/height/resolution/origin/frame_id are
byte-identical across both, AND (folded into B) that the first room's wall
cells are STILL occupied in the second map - proving the "re-rasterize from
scratch every message" design in `pcd2pgm.cpp`'s `liveCloudCallback` comment
doesn't lose anything as the map grows.

**B - wall points land in the correct cells.** Uses
`cell_index(x,y) = floor((x-ox)/res) + floor((y-oy)/res)*width`, computed in
Python from the exact points published, and checks `/map.data` at those flat
indices. **Gotcha found and fixed while writing this**: test points placed at
exact multiples of the 0.05 m grid resolution (natural, since GLIM's own
voxel grid is 0.5 m = 10x the map resolution) sit exactly on a `floor()`
boundary. Python computes the boundary in double precision; the C++ side
(`pcd2pgm.cpp` `setMapTopicMsgFixedBounds`) computes
`(point.x - fixed_origin_x_) / map_resolution_` with `point.x`/`map_resolution_`
as `float` promoted to `double` because `fixed_origin_x_` is `double` - a
different rounding of a nominally-"exact" division, e.g. `18.0/0.05` can come
out as `359.999999...` instead of `360.0` depending on which precision did
the division, flipping which cell wins by one. Every generated test point is
nudged by `EPS = 0.013` (common.py) - far smaller than the 0.5 m voxel
spacing so it doesn't change radius-filter neighbor relationships, and not a
multiple of 0.05 so it can't recreate the same problem - to land unambiguously
inside a cell instead of on a seam. This isn't just a test artifact: it means
a GLIM point that happens to land near-exactly on a 0.05 m grid line could
render in a different cell than naive hand-calculation predicts; not asserted
as a bug (both cells are "correct enough" at 5 cm resolution) but worth
knowing if you ever hand-verify a specific cell against real data.

**C - z passthrough (ground/ceiling rejection).** Three 3x3 voxel-spaced
clusters (each internally dense enough to survive the radius-outlier filter
on its own, isolating this scenario to the passthrough stage): one at
z=-0.60 (below `thre_z_min=-0.40`), one at z=1.50 (above `thre_z_max=1.20`),
one at z=0.0 as an in-range control. Asserts the two out-of-range cluster
centers are free and the control is occupied - confirms passthrough runs
before rasterization and actually excludes out-of-range points rather than,
say, only affecting the dynamic-bounds (file-mode) path.

**D - radius-outlier noise rejection.** Three single isolated points (no
neighbor within 0.75 m of anything) plus one small voxelized wall (from
`room_wall_points`, spacing 0.5 m, known to fully survive
`thre_radius=0.75/thres_point_count=2` per the yaml's own verified comment).
Asserts the isolated points are gone and the wall survives - the config
comment's claim, checked directly rather than trusted.

**E - out-of-bounds points.** Two 9-point voxel clusters placed outside
+-25 m (x=30 and x=-40). **Gotcha found while writing this**: an EARLIER
version used two lone out-of-bounds points (no neighbors) - they were
removed by the *radius-outlier* filter (same mechanism as D) before ever
reaching the bounds check in `setMapTopicMsgFixedBounds`, so the WARN never
fired and the test was silently checking nothing. Switched to clustered
points (mutually within 0.75 m, so they survive the radius filter and
actually reach rasterization) to genuinely exercise the bounds-drop path.
Asserts the WARN log line appears with a dropped-count >=18 (both 9-point
clusters), and that the node keeps working correctly afterward (control
cluster inside the grid still rasterizes).

**F - empty / all-filtered cloud.** Establishes an occupied map, then
publishes (1) a zero-point cloud and (2) a cloud whose only points are
z=5.0 (entirely removed by passthrough). Asserts that NO new `/map` is
published and the latched map still has its obstacles. History: live mode
originally had no empty-cloud guard, so both cases published an all-free grid
and wiped every obstacle from Nav2's static layer (occupied count went 16 -> 0).
Fixed 2026-09-15: `liveCloudCallback` now logs
`... empty after PassThrough/RadiusOutlier - keeping the previous /map` and
returns. PCL may also print a harmless
`[pcl::KdTreeFLANN::setInputCloud] Cannot create a KDTree with an empty input cloud!`.

**G - publisher QoS + late-joining subscribers.** Two parts, in
`test_late_join_qos.py` because both need control over process/subscriber
start order that a single shared node can't give cleanly:
  - **G1**: confirms pcd2pgm's OWN `/map` publisher really is
    reliable+transient_local+keep_last(1) (`pcd2pgm.cpp` constructor,
    `map_qos`) by subscribing AFTER a publish, with no new
    `/glim_ros/map` input in between, and confirming the late subscriber
    still gets the last grid (arrived in ~20 ms in testing).
  - **G2**: the theoretically interesting case - pcd2pgm's *subscription* to
    `/glim_ros/map` is `rclcpp::QoS(5).reliable()` with **no**
    `.transient_local()` (`pcd2pgm.cpp:53-56`), i.e. default VOLATILE
    durability. By the plain DDS durability-compatibility rule, a fresh
    VOLATILE reader shouldn't receive a sample the writer published before
    the reader existed - only future samples. Tested directly: start a fake
    GLIM publisher (real QoS) that publishes once and then goes idle for
    1.5s (simulating "GLIM has been running, has a retained map, and
    pcd2pgm is (re)started mid-run"), then start pcd2pgm cold and time its
    first `/map`. **Empirical result: pcd2pgm got the retained sample in
    ~0.1s, not after waiting for the next periodic publish.** Likely
    explanation (documented in-line in the script): with KEEP_LAST(1) +
    RELIABLE on the writer, there's always exactly one sample the writer is
    still trying to deliver reliably to every matched reader, independent of
    durability - so at depth 1 the distinction is mostly moot in this RMW
    (Fast-DDS, the Humble default). This is empirical, not a spec guarantee;
    a different RMW (Cyclone DDS, etc.) is not verified. Recorded as an
    observation, not asserted as a hard pass/fail either way - see "Open observations".

**H - timing on a ~500k-point cloud.** Builds a voxel-grid-aligned cloud
over the full 50x50 m arena at 4 z-layers (matching the [-0.40,1.20] range),
duplicated across 14 jittered "passes" (simulating GLIM accumulating
overlapping submap contributions rather than one clean dedup'd map) to reach
537,824 points - close to the requested ~500k while keeping a
GLIM-plausible local density (voxel-sized cells revisited across passes,
not uniform random over the whole area, which would be an unrealistically
dense radius-search workload). Parses the actual `Live map update took X ms`
line for THIS specific event (see log-scan gotcha below) and asserts
`< 10000 ms` (GLIM's own publish period, per the code comment: "If it ever
approaches GLIM's ~10s publish period the grid falls behind"), with a
printed warning if it's over half that. **Gotcha found while writing this**:
an earlier version grepped the whole log for the first line matching
`"Live map update took"`, which matched an OLD line from an earlier,
much-faster scenario (2 ms) instead of this event's line - `common.py`'s
`wait_for_log_match` now takes a `since=<byte offset>` marker
(`log_size(path)` taken right before publishing) so it only searches new log
content.

## Open observations (not bugs today)

- **Live subscription QoS has no `.transient_local()`** (`pcd2pgm.cpp`
  constructor: `rclcpp::QoS(5).reliable()`). GLIM publishes reliable +
  transient_local, depth 1. Scenario G2 shows a restarted pcd2pgm still gets
  GLIM's retained map in ~0.1 s on Fast-DDS, so this does not bite today. A
  different RMW or a deeper GLIM publisher queue could change that; adding
  `.transient_local()` is a one-line hardening if it ever does.
- **Seam rounding** (see B): a point almost exactly on a 0.05 m grid line can
  land in the neighbouring cell. Harmless at 5 cm.

## Known non-goals / limitations of this suite

- Does not test against the REAL GLIM binary/rviz_viewer, only a
  QoS/rate/shape-matched synthetic publisher. (Real-data check, done
  separately on 2026-09-15 with a 577k-point GLIM dump: 1000x1000 grid,
  ~15k occupied cells, 105-125 ms per update.)
- Scenario H's density/duplication model for the 500k-point cloud is an
  approximation of "many overlapping submap contributions", not a captured
  real accumulated map; timing on real data could differ (better or worse)
  depending on actual point distribution.
- G2 only exercises the default RMW on this VM (Fast-DDS via
  `rmw_fastrtps_cpp`, ROS 2 Humble default); not re-tested under Cyclone DDS.
