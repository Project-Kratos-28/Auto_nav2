# Cloudini: compressed point clouds for the laptop

The Orin half of the Cloudini link. The laptop GUI decodes (Auto_Gui, package `kratos_cloudini`,
its README has the laptop setup). Only copies for the laptop are compressed: GLIM, pcd2pgm, Nav2
and RViz on the Orin keep the raw topics.

```
GLIM ─► /glim_ros/points ─► cloudini_points ─► /glim_ros/points/compressed ─► radio ─► laptop decoder ─► GUI RViz
     └► /glim_ros/map ────► cloudini_map ────► /glim_ros/map/compressed ────►
     └► raw topics ─► pcd2pgm, Nav2, RViz on the Orin (unchanged)
```

## Run

```bash
~/kratos_glim/start.sh cloudini:=true                              # 1 cm resolution
~/kratos_glim/start.sh cloudini:=true cloudini_resolution:=0.005   # 5 mm
~/kratos_glim/logs.sh cloudini                                     # "Converted N messages, average compression ratio: X"
```

Or tick **Cloudini** in the GUI's TUI tab before **START** (`start_tui.sh` row 8 passes
`cloudini:=true`). On the laptop: `ros2 launch kratos_gui kratos_gui.launch.py cloudini:=true`.

| Launch argument | Default | Meaning |
|---|---|---|
| `cloudini` | `false` | `true`: start the two encoders |
| `cloudini_resolution` | `0.01` | position resolution (m); error is at most half of it |

## Contract with the laptop decoder

Change both sides together.

| | Scan | Map |
|---|---|---|
| Encoder input | `/glim_ros/points` (GLIM: reliable, volatile) | `/glim_ros/map` (GLIM: reliable, transient local) |
| Encoder output | `/glim_ros/points/compressed` | `/glim_ros/map/compressed` |
| Type | `point_cloud_interfaces/msg/CompressedPointCloud2` | same |
| Output QoS | reliable, volatile | reliable, volatile |
| Laptop decoder input | best effort (matches any publisher) | reliable, volatile |
| Laptop decoded output | `/kratos_gui/glim_points` | `/kratos_gui/glim_map` (latched) |

**Cloudini version:** apt 1.1.0 (format v4) on both sides (`ros-jazzy-cloudini-ros` is in
`docker/Dockerfile.glim`). A source-built 1.3.x writes format v5, which 1.1.0 can't read
(`Unsupported encoding version`): upgrade both or neither.

## Implementation

No custom code: two instances of Cloudini's stock `cloudini_topic_converter`, started by
`src/kratos_bringup/launch/kratos.launch.py`:

```python
cloudini = [Node(
    package='cloudini_ros', executable='cloudini_topic_converter', name=f'cloudini_{name}', output='screen',
    parameters=[{'compressing': True, 'topic_input': f'/glim_ros/{name}',
                 'topic_output': f'/glim_ros/{name}/compressed',
                 'resolution': ParameterValue(arg('cloudini_resolution'), value_type=float)}],
    condition=IfCondition(arg('cloudini')))
    for name in ['points', 'map']]
```

- `compressing: True`: the same executable decodes with `false`.
- `topic_output` is set explicitly to the contract names.
- `resolution` goes through `ParameterValue(..., value_type=float)`: launch arguments are strings,
  and `cloudini_resolution:=1` would otherwise arrive as an integer, which the node refuses.
- The nodes are in the main launch list, **not** in GLIM's 3 s `TimerAction` (see QoS below).
- Input `/glim_ros/points`, not `/livox/lidar`: GLIM's scan is already downsampled
  (`random_downsample_target: 10000`, `config_preprocess.json`) and the raw scan includes the antenna.

### What the converter does (cloudini_ros 1.1.0, `topic_converter.cpp`)

**At start, once:** it picks one QoS from the publishers already on its input topic, and uses it
for both its subscription and its publisher:

| Publishers seen | Reliability | Durability |
|---|---|---|
| none | reliable | volatile |
| all reliable / any best effort | reliable / best effort | |
| all transient local / otherwise | | transient local / volatile |

Started with the launch, 3 s before GLIM, it sees none: reliable + volatile. If it ever saw GLIM
first, the map output would be transient local; the laptop's reliable + volatile subscriber still
matches, so neither order breaks the link.

**Per cloud:**

1. No subscriber on the output (no laptop): return, nothing is compressed. Cost is ~0.
2. Read the raw serialized DDS message in place (no deserialization, no copy).
3. Round the positions to `resolution`, delta-encode neighbouring points, then compress the
   result with a general-purpose compressor (ZSTD/LZ4).
4. Publish `CompressedPointCloud2`: header (stamp, `frame_id`), field layout, `compressed_data`,
   `format`. The resolution is inside `compressed_data`: the decoder doesn't need it.
5. Every 20 clouds, log the average compression ratio (compressed ÷ original; lower is better).

### Why 1 cm

GLIM's scan is downsampled and its map is voxelized at 0.5 m: millimetres are noise, and noise
compresses badly. 1 cm (max error 5 mm) is invisible in RViz. If the ratio on real data is poor,
try `cloudini_resolution:=0.02`.

## Test

Hardware-free wiring check (encode → decode → echo), on its own domain; an empty cloud, so it
checks topics and QoS, not compression:

```bash
docker/run_container.sh 'export ROS_DOMAIN_ID=46
for t in points map; do
  ros2 run cloudini_ros cloudini_topic_converter --ros-args -r __node:=enc_$t -p compressing:=true \
    -p topic_input:=/glim_ros/$t -p topic_output:=/glim_ros/$t/compressed -p resolution:=0.01 &
  ros2 run cloudini_ros cloudini_topic_converter --ros-args -r __node:=dec_$t -p compressing:=false \
    -p topic_input:=/glim_ros/$t/compressed -p topic_output:=/rt/$t &
done; sleep 3
ros2 topic pub -r 2 /glim_ros/points sensor_msgs/msg/PointCloud2 "{header: {frame_id: odom}}" &
timeout 5 ros2 topic echo --once --field header /rt/points; pkill -f "__node:=enc_|__node:=dec_|topic pub"'
```

Expect the header (`frame_id: odom`) echoed from `/rt/points`. Also run on the Orin (2026-09-27)
with 5000 random points on both topics, from a Python publisher (GLIM's QoS, map latched): both
decoded, largest position error 5 mm at 1 cm.

On the rover, in the container:

```bash
ros2 topic list | grep compressed                      # both topics
ros2 topic info -v /glim_ros/map/compressed            # Reliability: RELIABLE, Durability: VOLATILE
ros2 topic bw /glim_ros/points                         # raw (measure on the Orin, never from the laptop)
ros2 topic bw /glim_ros/points/compressed              # compressed; same for /glim_ros/map
```

## Known limits

- **Map not latched across the link:** a GUI started after GLIM's last map update waits for the
  next submap. Fix: an encoder publishing the map transient local with depth 1 (the stock one keeps
  10 messages, which a late laptop would be sent) **and** the laptop decoder's map input switched
  to transient local. Change both together: a transient-local subscriber never matches a volatile
  publisher.
- **The whole map on every update:** compressed, but one large reliable message that grows all
  session. Check its size over the radio first.
- **Humble (laptop) ↔ Jazzy (Orin)** for `CompressedPointCloud2` is not tested yet. Compare
  `ros2 interface show point_cloud_interfaces/msg/CompressedPointCloud2` on both sides.
- **Not measured on real data yet:** ratio, bandwidth, encoder CPU.

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| No `/compressed` topics | started without `cloudini:=true` | restart with it |
| Topics exist, `logs.sh cloudini` never logs a ratio | no laptop subscribed (by design), or QoS/distro mismatch | `ros2 topic info -v` on both sides |
| Laptop: `Unsupported encoding version` | different Cloudini versions | apt 1.1.0 on both |
