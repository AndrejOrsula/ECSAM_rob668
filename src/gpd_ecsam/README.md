### gpd_ecsam
This package utilises [`gpd`](https://github.com/atenpas/gpd) to generate grasps and subsequently converts them into poses that are usable for [`MoveIt!`](https://moveit.ros.org/).

#### Nodes
+ `gpd_grasp_to_moveit` converts the `gpd` graps

#### Launch files
The following launch files contain the `gpd` configuration for the *ECSAM*'s *JACO*
+ `j2n6s200_15_channels.launch`
+ `j2n6s200_classifier_15_channels.launch.xml`
+ `j2n6s200_hand_geometry.launch.xml`
