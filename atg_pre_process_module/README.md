# Installation
Follow atg_gui_module

# Module contains region growing segmentation function

# How to use:
## Region Growing Segmentation function
```
rosrun atg_surface_identification atg_rg_seg <path/filename> <smoothness(in deg)> <curvature(in deg)> <min_cluster_size> <ksearch_no.> <no. of neighbours>
```

- file format include obj and pcd
- smoothness
- curvature
- min_cluster_size is minimum number of points to be considered as a segment cluster
- ksearch is number of neighbouting points used for calculating every point's normals
- neighbour is number of neighbouring points to compare per point for segmentation classification
Output will be individual clusters of segments in pcd format and will be stored in <workspace>/data/coupons/clusters/cluster_0.pcd .. cluster_1.pcd ... rejected_points.pcd
Note that output folder will be inside workspace, not inside package folder, stored cluster is suppose to be temp for use within instance
