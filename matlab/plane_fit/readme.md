These scripts are used to grow planes in a point cloud whose bounding boxes are then used for plane flatness assessment.

There are quite a few tools to draw boxes on point clouds, like cloudcompare and labelcloud.
But they do not work well when the point clouds have lots of occlusions, so we have developed a set of very specialized code for this purpose.
The general workflow can be found in choose_plane_seeds.m.
If the occlusion is heavy, in preprocessing, you may need to align the ref point cloud to gravity and normals to xy axes, and then remove the ceiling by thresholding z.

1. pick up seed points on the reference point cloud
2. grow planes by region growing or circle covering
3. form bounding boxes
4. check the points within the bounding boxes


