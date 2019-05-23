### et7
This package obtains data from the *ET7* for the use within the *ECSAM*.

#### Nodes
+ `gaze_and_scene_video_stream_tcp` establishes TCP/IP communication wiht the *ET7* to obtain gaze estimation coordinated within the scene image together with the JPEG encoded scene image stream

#### Launch files
+ `bringup.launch` launches `gaze_and_scene_video_stream_tcp` and converts JPEG encoded image from the *ET7* into raw image
