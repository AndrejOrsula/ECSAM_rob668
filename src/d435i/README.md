### d435i
This package launches the *RealSense D435i* depth camera and configures it for the use within the *ECSAM*.

#### Launch files
+ `camera.launch` establishes communication with *D435i* and configures it
+ `bringup.launch` includes `camera.launch`, converts depth image into a PointCloud, pass-through filters this PointCloud into a box containing the scene and statistically removes the outlier cloud points
