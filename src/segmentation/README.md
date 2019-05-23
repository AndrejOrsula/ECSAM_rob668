### segmentation
This package segments the PointCloud acquired from the *D435i* by the use of [`PCL`](http://pointclouds.org/).

#### Nodes
+ `segmentation` performs the segmentation by the use of *RANSAC* to exctract horizontal plane and *Euclidean Cluster Extraction* for further segmentation of the individual objects.

#### Launch files
+ `segmentation.launch` launches `segmentation`
