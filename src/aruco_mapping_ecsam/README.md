### aruco_mapping_ecsam
This package provides the *ECSAM* with the means of determining the pose of *ET7* scene camera and the pose of *D435i* in respect to *JACO* coordinate frame.

#### Nodes
These two nodes provide the correlation
+ `aruco_mapping` determines the camera pose in respect to the first detected marker
+ `root_marker_static_tf_broadcaster` broadcasts a static transform between *JACO* and the first marker

#### Launch files
The following two launch files are present within his package:
+ `bringup_d435i.launch` for *D435i*
+ `bringup_et7_scene_camera.launch` for *ET7* scene camera

#### Config files
Files `d435i.ini` and `et7_scene_camera.ini` contain intrinsic parameters of *D435i* color camera and *ET7* scene camera respectively obtained through [camera_calibration](https://wiki.ros.org/camera_calibration)
