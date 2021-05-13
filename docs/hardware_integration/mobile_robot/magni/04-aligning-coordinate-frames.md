# Aligning Coordinate Frames

Right now, the free fleet client is reporting its position according to its own coordinate frame, but the server is expecting positions in its "own" frame ( as referenced in the traffic editor ).

The [transformation parameters](https://github.com/open-rmf/free_fleet/blob/main/ff_examples_ros2/launch/fake_server.launch.xml#L26-L29) describe a rigid 2D transformation between the two frames, so that we can correctly report the robot position in the RMF "frame of reference".

We will have to compute this for our specific use case.
