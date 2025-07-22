# CARSLab-Error-Detection

This is a node for ROS2 (Humble). It subscribes to a topic called /pointcloud for LiDAR and does a rudimentary scan of the values being transmitted from the topic. It sends its findings through /LidarLogs
It searches for several things:
1. NaNs and Infs (essentially, non-dense data)
2. A large quantity of LiDAR values that are greater than the expected range.
3. Identical Frames
4. Too many or too few points
5. Too high or low of a message-send frequency

I have since optimized it to the best of my ability, so it should not be a big processing drain. The biggest processing power goes towards scanning each message (each with over 200,00 points) and finding NaN's, Infs, and values greater than expected.

The search automatically ends if the frame is identical to the previous one, or if the message is empty.

All logs, instead of going to the console, are sent through the topic /LidarLogs   with the format {level:byte, error_name:string, description:string}
0=DEBUG
1=INFO
2=WARN
3=ERROR
4=FATAL
