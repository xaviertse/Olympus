# Installation
Follow atg_gui_module

# Module contains toolpath generation function of contour(boundary) and zigzag

# How to use:
## Contour function:
```
rosrun atg_toolpath_generation atg_toolpath_contours <path/filename> <external_on> <internal_on> . . .
```
- filename (e.g. \<workspace\>/data/coupons/clusters/cluster_3.pcd)
- external_on
- internal_on
- resolution
- step_size (not used, can assign 0)
- offset
- path_rotate
- downsample
- ksearch_tp
- normal_flip
- section_range_min
- section_range_max
- reverse_toolpath
- forced_resolution
- hole_patch_size

## Zig Zag function:
```
rosrun atg_toolpath_generation atg_toolpath_zigzag <path/filename> <external_on> <internal_on> . . .
```
- filename (e.g. \<workspace\>/data/coupons/clusters/cluster_3.pcd)
- external_on
- internal_on
- resolution
- step_size
- offset
- path_rotate
- downsample
- ksearch_tp
- normal_flip
- section_range_min (not used, can assign 0)
- section_range_max (not used, can assign 0)
- reverse_toolpath
- forced_resolution
- hole_patch_size

Output will be individual clusters of segments in pcd format and will be stored in:

\<workspace\>/data/coupons/tool_path_back_projected_w_lift_n_trigger.txt for points, normals, angle, quaternion and I/O trigger instructions

\<workspace\>/data/coupons/tool_path_back_projected_w_lift.pcd for PointNormal data which can be loaded in GUI for viewing

Note that output files will be replaced every time a toolpath operation triggers, stored toolpath is suppose to be temp for use within instance
