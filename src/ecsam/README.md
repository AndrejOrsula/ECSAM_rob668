### ecsam
This package collects all other the *ECSAM* packages into a single stack. Furthermore, it utilises `j2n6s200_moveit` class to command the motion of *JACO*.

#### Node
+ `ecsam` subscribes to generated grasps and placement positions and interacts with *JACO* through `j2n6s200_moveit` class

#### Launch files
+ `connected.launch` launches all packages for use with the physical *JACO*
+ `virtual_gazebo.launch` does the same for virtual/Gazebo *JACO*
