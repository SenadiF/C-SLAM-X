# multi_robot_sim

Two-robot Gazebo test bed for your SLAM + navigation stack, so you can
validate logic without depending on live hardware, and run your three
map-merging methods side by side for comparison.

## Setup

```bash
cd ~/Desktop/C-SLAM-X/robot_ws/src
# copy/extract this multi_robot_sim folder here

cd ~/Desktop/C-SLAM-X/robot_ws
colcon build --packages-select multi_robot_sim
source install/setup.bash
```

**Before first run**, check two things in `launch/multi_robot_sim.launch.py`:
- `ekf_pkg` / `nav_pkg` variable names near the top match your actual
  package names (`robot_localization_config`, `my_navigation`, or
  whatever you called them).
- Your `ransac_icp_map_aligner.py` node from earlier is added to that
  package's `setup.py` entry_points as `ransac_icp_map_aligner`, and
  built.

## Running

```bash
# Method A: known initial poses (deterministic baseline)
ros2 launch multi_robot_sim multi_robot_sim.launch.py merge_mode:=known

# Method B: unknown poses, multirobot_map_merge's built-in feature matching
ros2 launch multi_robot_sim multi_robot_sim.launch.py merge_mode:=unknown

# Method C: unknown poses, your own RANSAC+ICP aligner
ros2 launch multi_robot_sim multi_robot_sim.launch.py merge_mode:=ransac_icp
```

Each brings up: Gazebo (two-room world with a connecting doorway),
both robots, the bridge, EKF x2, SLAM x2, the selected merge method,
and your frontier_explorer / astar_planner / pure_pursuit stack —
then opens RViz.

## In RViz

- Fixed Frame: `map`
- Add **Map** → topic `/map`
- Add **LaserScan** for `/robot1/scan` and `/robot2/scan`
- Add **TF** to watch both robots' frames live

You should see both robots begin exploring on their own via your
frontier_explorer, and (once frontiers bring them near the doorway)
the two maps should visibly connect into one continuous room outline.

## What to actually check for your comparison

- **Alignment error**: does the doorway/wall line up cleanly between
  the two halves, or is there a visible seam/offset? Known poses
  should be near-perfect since it's deterministic given your set
  x/y/yaw. Unknown/feature-matching depends on real overlap existing.
  RANSAC+ICP's terminal output logs its own computed error each cycle
  (`error=... m^2`) — record this over a few runs.
- **Time to first successful merge**: known poses merges essentially
  instantly. Unknown-pose methods need the robots to actually explore
  into overlapping territory first — time this from launch to first
  visible merged map.
- **Robustness to no/low overlap**: try commenting out the doorway gap
  in `worlds/two_room_test.sdf` (make `divider_top`/`divider_bottom`
  a single solid wall) to fully separate the two rooms, and see which
  methods still produce anything useful. Known poses still works
  (it never needed overlap); the other two should visibly fail or
  stall — a good, honest negative result for your report.

## Notes / things that may need small local tweaks

- Gazebo plugin filenames (`gz-sim-diff-drive-system`, etc.) assume
  Gazebo Harmonic, matching ROS2 Jazzy. If your installed Gazebo
  version differs, these filenames may need adjusting — check
  `gz sim --versions` and the corresponding `ros_gz` release notes
  if the DiffDrive/lidar/imu plugins fail to load on first run.
- `robot2`'s spawn pose (`x=-2.0, y=-1.5`) is set to give partial
  overlap near the doorway. Adjust in both
  `launch/multi_robot_sim.launch.py` and
  `config/map_merge_known_sim.yaml` together if you change it —
  they need to match for the known-pose method to be accurate.
- Since `use_sim_time` is true everywhere and Gazebo provides one
  consistent clock, the ESP32 restamper node from your hardware
  pipeline is intentionally **not** used here — not needed in sim.

## Suggested test order (same discipline as hardware debugging)

1. Launch with `merge_mode:=known` first — this is your best chance
   of a fully working run on the first try, and confirms SLAM,
   frontier/A*/pursuit, and EKF are all sound in isolation from any
   merge-method uncertainty.
2. Once that's solid, try `merge_mode:=unknown`.
3. Then `merge_mode:=ransac_icp`.
4. Compare the three using the criteria above for your performance
   analysis slide.
