# LiDAR Error Detection and Documentation
For the University of Delaware's CAR Lab

# Background:
A LiDAR sensor operates by sending out flashes of light and measuring the time it takes to see a reflection. With a large enough sampling rate, you can detect the boundaries of buildings, cars, pedestrians, trees, and more. This makes LiDAR sensing extremely appealing for autonomous vehicle purposes. The issue, of course, is that if the LiDAR sensor stops working for whatever reason, the vehicle can no longer sense obstacles (which is bad). 

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

* Issues 6 and 7 are done in a separate node, so that they don't throttle the other fault detections. I did some in-person testing and found that if they are in the same node, the node missed 7 in 10 messages, so I have separated the intense stuff from the less-intense stuff.


# Technical Stuff:

This repository is made for Robot Operating System 2 (ROS2), Humble distro. There are two custom nodes -- one for the less-intensive searches, and one for the more-intensive searches. The nodes are called "pointcloud_checker" and "message_searcher" respectively. "pointcloud_checker" does a fault detection on every message, while "message_searcher" checks every 20 messages (the things it searches for are unlikely to change between messages, so it can save processing power by not checking every one of them).

There are two packages in this repository: the listener and analysis package named "lidar_error_checker", and the package that creates a custom interface (message) type for the LiDAR error logging named "lidar_interfaces".

After detecting an issue, it sends an alert through the topic /lidar_logs in the following format:

Level (byte), Error Name (string), Description (string)

"Level" refers to the severity of the error.


0 -- Debug

1 -- Info

2 -- Warning

3 -- Error

4 -- Fatal


All the ideal values and thresholds can be changed in the code to fit the specifications of any LiDAR sensor. I did my best to comment anything that's potentially confusing, which should make modification easier.

The QOS settings are the standard ones for sensors. The most important setting to know is that it uses Best Effort instead of Reliable.

# Guide for running the node and editing the node:
WINDOWS:

Prerequisites:

  Visual Studio Code
    
   Required extension: Dev Containers
   
   Recommended extensions: CMake Tools, Pylance, Python
    
  Docker Desktop


1. Download the src folder and all its contents, and put it in your own folder on your PC. Alternatively, download the zip and extract it.
2. Download the Dockerfile and devcontainer.json files and put them in a folder named ".devcontainer" on the same level as the src folder.
3. Make sure the Dockerfile is not a .txt and that it's just a generic file instead.
4. Open Docker Desktop
5. Open Visual Studio Code and start the high-level folder (the one that has the .devcontainer and src folders in it)
6. Download the Dev Containers extension if you haven't already
7. Find the little blue logo in the bottom left corner that looks like a lightning bolt
8. Select "Reopen in Container"
9. Sit back and wait for the container to finish constructing. Warning, it takes a while!
10. Now, you're at a position where you can run the codes and navigate through the terminal. You can access the terminal with ctrl + ` 


LINUX:

Prerequisites:

   Good luck, I don't understand Linux. Probably you need Ubuntu and ROS2 or something :)



Anyway, once you've got the terminal up:

Navigate to the top level (something like workspace/ros2_ws/) and do "colcon build". At this point, you can start running code! Below are some helpful commands for debugging or modifying:

      colcon build (from top level)
      ros2 run lidar_error_checker pointcloud_checker
      ros2 run lidar_error_checker message_searcher
      cd .. (to go up one level)
      ls (to list all directories in the folder you're in)
      ros2 topic echo /lidar_logs
      ros2 topic hz /pointcloud
Don't forget to     source /opt/ros/humble/setup.bash




# Other Information

Q: How do I change the format of the custom message type? For example, I want the message to include a timestamp and a frame number!

A: Navigate to the lidar_interfaces package, and open LidarAlert.msg. There are comments detailing how to add or subtract data. Then, change the messaging in the main python files to accomodate the new data.
<br> </br>
Q: How do I change the name of the topic that these nodes publish to?

A: Navigate to the " self.publisher = self.create_publisher(LidarAlert, 'lidar_logs', 10) " lines in both nodes and change " lidar_logs " to your desired name.
<br> </br>
Q: How do I change the topic that these nodes subscribe to?

A: Navigate to the "self.subscription = self.create_subscription( PointCloud2, '/pointcloud'. self.listener_callback, qos_rofile_sensor_data ) " line and change " '/pointcloud' " to your desired name.
<br> </br>
Q: How much wood could a woodchuck chuck if a woodchuck could chuck wood?

A: A woodchuck would chuck as much wood as a woodchuck could chuck if a woodchuck could chuck wood.
<br> </br>
Q: I want to make another node that does new analyses! How can I do that?

A: The way I did it was to copy one of the existing nodes into a new file (in the same directory), change all references to the node name to what you want, then add the entry to the setup.py file. There are comments everywhere detailing what needs to be changed.
<br> </br>
Q: How can I contact you for questions?

A: Open an issue here and I'll try to get to it! 
