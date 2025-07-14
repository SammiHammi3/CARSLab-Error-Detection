# CARSLab-Error-Detection

This is a node for ROS2 (Humble). It subscribes to a topic called /pointcloud for LiDAR and does a rudimentary scan of the values being transmitted from the topic. This code is NOT optimized in any way yet.
It searches for several things:
1. NaNs and Infs (essentially, non-dense data)
2. A large quantity of LiDAR values that are greater than the expected range.
3. Identical Frames
4. Too many or too few points
5. Too high or low of a message-send frequency

I have since optimized it to the best of my ability, so it should not be a big processing drain. The biggest processing power goes towards scanning each message (each with over 200,00 points) and finding NaN's, Infs, and values greater than expected.
