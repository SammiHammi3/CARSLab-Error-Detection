# LiDAR Error Detection and Documentation
For the University of Delaware's CAR Lab

# Background:
A LiDAR sensor operates by sending out flashes of light and measuring the time it takes to see a reflection. With a large enough sampling rate, you can detect the boundaries of buildings, cars, pedestrians, trees, and more. This makes LiDAR sensing extremely appealing for autonomous vehicle purposes. The issue, of course, is that if the LiDAR sensor stops working for whatever reason, the vehicle can no longer sense obstacles. 

There are a myriad of methods to manipulate the LiDAR's pointcloud to de-noise the results -- sliding mode control, wavelet transform, and a lot more that I don't understand -- but fewer solutions exist to _detecting_ the problems rather than pre-emptively de-noising them. 

# The Project:
This project, overseen by the University of Delaware's CARLab, involves detecting and logging common LiDAR issues:

1. Deviations from the ideal frequency of the sensor
2. Dropped messages
3. Empty pointclouds
4. Identical messages
5. NaNs and Infs
6. A large quantity of point values greater than the reported maximum range
7. Too many or too few points per message

*Issues 6 and 7 require high comutational power, since they involve checking each 230,400 points in every message, so you can turn those functions off with the variable DoIndividualPointChecking. When I was testing with the CARLab automated vehicle, analysis was significantly slower when searching every point -- the frequency fell from 10hz to ~1.7 hz. So unless it is extremely important to check for values outside the maximum, or NaNs or Infs, I recommend toggling it off. I am currently (7/29/25) working on making another subscriber that samples a message every so often (instead of every message) to speed everything up. 

# Technical Stuff:

This repository is made for Robot Operating System 2 (ROS2), Humble distro. The custom node (named "pointcloud_checker") subscribes to a topic called /pointcloud, which is used for LiDAR, and does a scan through all the messages being transmitted from that topic. There are two packages in this repository: the listener and analysis package named "lidar_error_checker", and the package that creates a custom interface (message) type for the LiDAR error logging named "lidar_interfaces".

After detecting an issue, it sends an alert through the topic /LidarLogs in the following format:
   Level (byte), Error Name (string), Description (string)
"Level" refers to the severity of the error.
0 -- Debug
1 -- Info
2 -- Warning
3 -- Error
4 -- Fatal

All the ideal values and thresholds can be changed in the code to fit the specifications of any LiDAR sensor. I did my best to comment anything that's potentially confusing, which should make modification easier.

The QOS settings are the standard ones for sensors. The most important setting to know is that it uses Best Effort instead of Reliable.

# Guide for running the node and editing the node on Windows:
Prerequisites:

  Visual Studio Code
    
    Required extension: Dev Containers
    
    Recommended extensions: CMake Tools, Pylance, Python
    
  Docker Desktop

1. Download the src folder and all its contents, and put it in your own folder on your PC. Alternatively, download the zip and extract it.
2. Download the Dockerfile and devcontainer.json files and put them in a folder named ".devcontainer" on the same level as the src folder.
3. Make sure the Dockerfile is not a .txt and that it's just a generic file instead
4. Open Docker Desktop
5. Open Visual Studio Code and start the high-level folder (the one that has the .devcontainer and src folders in it)
6. Download the Dev Containers extension if you haven't already
7. Find the little blue logo in the bottom left corner that looks like a lightning bolt
8. Select "Reopen in Container"
9. Sit back and wait for the container to finish constructing. Warning, it takes a while!


Now, you're at a position where you can run the codes and navigate through the terminal. You can access the terminal with ctrl + ` 

11. Navigate to the top level (something like workspace/ros2_ws/) and do "colcon build"

At this point, you can start running code!


You can run the node with Ros2 run lidar_error_checker pointcloud_checker
