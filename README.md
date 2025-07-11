# CARSLab-Error-Detection

This is a node for ROS2 (Humble). It subscribes to a topic called /pointcloud for LiDAR and does a rudimentary scan of the values being transmitted from the topic. This code is NOT optimized in any way yet.
It searches for several things:
1. NaNs and Infs (essentially, non-dense data)
2. A large quantity of LiDAR values that are greater than the expected range.
3. Identical Frames
4. Too many or too few points

There is still much to optimize and implement, so this is not the final product. Things that need to be done still:
1. Optimize code so that only one For loop runs instead of two per message
2. Monitor the frequency of messages coming in
3. Redo the logging priorities for each error
