# Hardware in the loop: Isaac Sim on the DGX, the stack on the Orin

Isaac Sim renders the rover, its LiDAR and depth camera on the DGX Spark. The autonomy nodes run on
the Orin, in the `kratos_glim` container, on that data and on the sim's `/clock`. Nav2's
`/cmd_vel` goes back over the switch to the sim's swerve controller.

| DGX Spark (`tools/isaac_nav2_sim.sh up --orin=…`) | Orin (`tools/hitl_orin.sh up --sim=…`) |
|---|---|
| Isaac Sim (arena, `/clock`, `/livox/points`, `/odom` + TF, `/sim_camera/*`) | Nav2, 7 nodes: RotationShim + MPPI, `nav2_params_sim.yaml` |
| `athena_controller` bringup: swerve controller on `/cmd_vel`, mount TFs | `traversability_mapper` (LiDAR + depth points) |
| RViz, `goal` | nvblox on the sim depth image (`nvblox_sim.launch.py`) |
| | `--glim`: GLIM (sim config), pcd2pgm `/map`, IMU computed from `/odom` |

The Orin runs `athena_nav_sim`'s `nav_sim.launch.py`, the same bringup the DGX runs by itself
without `--orin`. It is not `kratos.launch.py`: the sim rover has its own frames, mounts and GLIM
config, and neither the Livox driver nor the ZED SDK runs. What this tests is this repo's Nav2,
MPPI, terrain and nvblox code on the Orin's CPU and GPU, with the network in between.

## Network (once)

Both machines go on the same switch, with addresses in one subnet. The Orin's LiDAR port is
`192.168.1.10/24`, so give the DGX's Ethernet port a second address there. Keep it outside
`.100`–`.199`, the MID-360's range:

```bash
# DGX: enP7s7 also keeps its lab address
sudo ip addr add 192.168.1.50/24 dev enP7s7          # until reboot; use nmcli to keep it
ping -c 3 192.168.1.10
```

Socket buffers on the Orin. Point clouds get dropped without this:

```bash
echo -e 'net.core.rmem_max=4194304\nnet.core.wmem_max=4194304' | sudo tee /etc/sysctl.d/60-hitl.conf
sudo sysctl --system
```

## Orin (once, and after every pull)

```bash
git pull
docker/run_container.sh docker/build_ws.sh     # kratos_traversability is new
docker/build_image.sh                          # only for --glim if SciPy is missing (hitl_orin.sh says so)
```

The first `tools/hitl_orin.sh up` makes a sparse clone of `Auto_Isaac_Sim` (branch
`zed2i_testing`) into `.hitl/`, which git ignores, and builds `athena_nav_sim` there. It needs
the Orin's GitHub SSH key. `tools/hitl_orin.sh update` pulls it and rebuilds.

## Run

```bash
# DGX, from the desktop (Isaac opens a window)
tools/isaac_nav2_sim.sh up --orin=192.168.1.10          # waits up to 15 min for the Orin's Nav2

# Orin, either order (waits up to 10 min for the sim's /clock)
tools/hitl_orin.sh up --sim=192.168.1.50                # ground-truth pose, LiDAR + depth camera
tools/hitl_orin.sh up --sim=192.168.1.50 --glim         # GLIM SLAM instead

# DGX: goals and RViz as usual
tools/isaac_nav2_sim.sh goal gates
tools/isaac_nav2_sim.sh status

# Orin
tools/hitl_orin.sh logs                                 # Nav2's log (.hitl/logs/)
tools/hitl_orin.sh stop

# DGX
tools/isaac_nav2_sim.sh stop
```

- **After Stop/Play in Isaac**, sim time starts again at 0: `tools/hitl_orin.sh stop && tools/hitl_orin.sh up`.
- `--no-zed` on the DGX means LiDAR only. The Orin notices the missing camera by itself.
- The Orin side refuses to start while `./start.sh` (the rover stack) runs in the same container, and the reverse holds too.
- Fast DDS on the Orin uses only the switch interface (`.hitl/fastdds.xml`). Otherwise the DGX
  would also send the point clouds to the Orin's Tailscale and Docker addresses.

## Troubleshooting

| Symptom | Check |
|---|---|
| `no /clock from …` | sim playing? `ping`; both `ROS_DOMAIN_ID=0`; retry with `HITL_DDS_XML=0 tools/hitl_orin.sh up …` |
| Nav2 up, the rover does not move | on the DGX: `ros2 topic echo /cmd_vel`; `tools/isaac_nav2_sim.sh status` |
| Choppy costmaps, GLIM drops scans | socket buffers (above); in the container: `ros2 topic hz /livox/points` (Isaac: ~10 Hz) |
| `--glim needs python3-scipy` | `docker/build_image.sh`, or install it in the running container as the message says |
