# Block-Map LiDAR Localization (BBS + ICP)

A Python / Open3D reimplementation of block-map-based LiDAR localization, following a method
presented at IEEE ICRA 2024. Instead of matching every scan against one large global map, the map
is split into overlapping **block maps**; the robot is localized coarsely with a
**branch-and-bound search (BBS)**, then its pose is refined and tracked with **ICP**.

**Write-up and figures:** https://shourya1515.github.io/shourya-robotics-portfolio/project-block-map-localization.html

## How it works

1. **Block-map generation** — LiDAR scans are transformed into the world frame and accumulated
   into the current block until the robot has travelled farther than the block size; then a new
   block starts. Block centroids are indexed in a KD-tree.
2. **Active block selection** — the block whose centroid is nearest to the current position.
3. **Coarse initialization** — branch-and-bound search over a multi-resolution map pyramid.
4. **Tracking** — Open3D point-to-point ICP against the active block, with motion prediction and a
   sliding window of keyframes (20 per block; 5 kept when switching blocks).

## Results

On a ~600 m simulated trajectory, the tracked pose stayed under about 0.2 m error for most of the
run, and each of the 5 block-map switches caused a pose jump under 0.05 m. The estimate drifted
in Y over time; the original paper reports about 0.1 m.

## Scope

A study reimplementation (Autonomous Robotics course project, Northeastern University,
Mar – Apr 2025). It runs on LiDAR point clouds (`.pcd`) with a synthetic trajectory and poses —
there is no ground-truth dataset, ROS integration or hardware.

## Files

| File | Purpose |
|---|---|
| `Block Map Generation` | Main Python script: the `BlockMapLocalization` class (block-map generation, BBS initialization, ICP tracking, visualization) and the `process_pcd_directory()` entry point |
| `Pcd visualization` | Helper script that loads and plots every `.pcd` file in a folder |

## Running

Both files are plain Python scripts, originally written for Google Colab.

```bash
pip install open3d numpy scipy matplotlib ipython
```

Set `pcd_directory` at the bottom of `Block Map Generation` to a folder of `.pcd` scans, then run
it with `python "Block Map Generation"` (or paste it into a Colab / Jupyter cell).
`process_pcd_directory()` builds the block maps, saves them as `block_map_XXX.pcd` with a
`centroids.txt` index, and runs localization along the synthetic trajectory.
