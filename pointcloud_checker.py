import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from time import time
import sys
import math
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs_py import point_cloud2
import hashlib

sys.stdout.reconfigure(line_buffering=True)


# how far from expected values can we deviate?
percent_deviation_allowed = 0.01 # % deviation allowed from the ideal point count

percent_invalid_allowed = 0.05 # what % of points can be invalid before we flag the message as bad
percent_exceptional_allowed = 0.05 # what % of points can be outside the maximum range before we flag the message as bad
message_rate_threshold = 2 # 2 hz in either direction flags a warning

ideal_message_rate = 10 #hz
ideal_point_count = 230400
maximum_range = 200 # meters, with the pandar lidar at 10% reflectivity

max_points = ideal_point_count * (1 + percent_deviation_allowed)
min_points = ideal_point_count * (1 - percent_deviation_allowed)



class PointCloudChecker(Node):
    def __init__(self):
        super().__init__('pointcloud_checker')
        
        self.subscription = self.create_subscription(
            PointCloud2,
            '/pointcloud',
            self.listener_callback,
            qos_profile_sensor_data
)
        self.subscription  # prevent unused variable warning
        self.prev_data_hash = None
        self.prev_msg_time = None

    def listener_callback(self, msg):
        #self.get_logger().info(msg.header.frame_id) #velodyne_top_base_link
        self.check_data(msg)
        

        # TODO: Implement logic here to analyze PointCloud2 data
        pass

    def check_data(self, msg):
        
        log = self.get_logger()
 
        num_points = msg.width * msg.height
        
        
        #If we're getting too few or too many points
        if num_points < min_points:
            log.warn(f'BELOW expected number of points ({num_points}).')
        if num_points > max_points:
            log.warn(f'ABOVE expected number of points ({num_points}).')    

        exceptional_count = 0
        invalid_count = 0
        
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False):
            x, y, z = point


            # Check if any coordinate is NaN or Inf
            if not all(math.isfinite(v) for v in (x, y, z)):
                invalid_count += 1
            # Check if the point is within the maximum range

            if any(abs(v) > maximum_range for v in (x, y)):
                exceptional_count += 1
                
        
        if num_points == 0:
            log.fatal("Received PointCloud2 message with zero points.")
        else:
            exceptional_ratio = exceptional_count / num_points
            invalid_ratio = invalid_count / num_points
            if exceptional_ratio > percent_exceptional_allowed:
                log.warn(f'[{time():.2f}] Too many points outside the maximum range: {exceptional_count} out of {num_points}.')
            if invalid_ratio > percent_invalid_allowed:
                log.fatal(f'[{time():.2f}] Too many invalid points in the PointCloud2 message: {invalid_count} out of {num_points}.')

        # Hashes the current and previous messages to check for identical frames
        current_hash = hashlib.md5(msg.data).hexdigest()
        if self.prev_data_hash == current_hash:
            log.warn("Frame is identical to the previous frame.")
        else:
            self.prev_data_hash = current_hash    
        

        current_time = self.get_clock().now().nanoseconds / 1e9  # ROS time in seconds

        if self.prev_msg_time is not None:
            dt = current_time - self.prev_msg_time
            frequency = 1.0 / dt if dt > 0 else 0

            if abs(frequency - ideal_message_rate) > message_rate_threshold:
                self.get_logger().warn(f"Frequency deviation: expected ~10Hz, got {frequency:.2f} Hz")

        self.prev_msg_time = current_time





def main(args=None):
    print("Starting PointCloudChecker Node")
    rclpy.init(args=args)
    node = PointCloudChecker()

    print("Node initialized, spinning now.")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    print("PointCloudChecker Node has been shut down.")
    pass