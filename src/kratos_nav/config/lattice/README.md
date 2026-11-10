# Smac Lattice motion primitives

`output.json` is the set of moves `SmacPlannerLattice` may chain into a path. `config.json` made it:
diff drive (arcs, straights, pivots in place), 1.0 m arc radius, 16 headings, 0.05 m grid.

`grid_resolution` **must equal** the costmap `resolution` in `nav2_params.yaml` (0.05). Change
either, or the arc radius, and regenerate:

```bash
git clone --depth 1 -b jazzy https://github.com/ros-navigation/navigation2.git /tmp/nav2
cd /tmp/nav2/nav2_smac_planner/lattice_primitives
pip install -r requirements.txt    # numpy, matplotlib, rtree
python3 generate_motion_primitives.py \
    --config /workspaces/kratos_glim/src/kratos_nav/config/lattice/config.json \
    --output /workspaces/kratos_glim/src/kratos_nav/config/lattice/output.json
```

Then rebuild `kratos_nav` so the new file lands in `install/`.
