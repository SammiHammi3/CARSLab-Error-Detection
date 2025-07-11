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
percent_deviation_allowed = 0.01

ideal_message_rate = 10 #hz
ideal_point_count = 230400
maximum_range = 200 # meters, with the pandar lidar at 10% reflectivity

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


    def listener_callback(self, msg):
        #self.get_logger().info(msg.header.frame_id) #velodyne_top_base_link
        self.check_message_rate(msg.header)
        self.check_point_count(msg)
        self.check_point_values(msg)
        self.check_frame_duplicity(msg)

        # TODO: Implement logic here to analyze PointCloud2 data
        pass


    def check_message_rate(self, header):
        # Determine if the message rate is acceptable
        pass


    def check_point_count(self, msg):
        num_points = msg.width * msg.height
        max_points = ideal_point_count * (1 + percent_deviation_allowed)
        min_points = ideal_point_count * (1 - percent_deviation_allowed)
        
         # If we aren't getting any points
        if num_points == 0:
            self.get_logger().fatal('Received PointCloud2 message with zero points.')
        
        #If we're getting too few or too many points
        if num_points < min_points:
            self.get_logger().warn(f'Received PointCloud2 message with {num_points} points, which is below the minimum expected threshold.')
        if num_points > max_points:
            self.get_logger().warn(f'Received PointCloud2 message with {num_points} points, which exceeds the maximum expected threshold.')# Determine if the number of points in the PointCloud2 message is within expected limits

        return


    def check_point_values(self, msg):

        # Check if the point values are within expected ranges
        exceptional_count = 0
        exceptional_list = []
        max_exceptional_value = 0

        for point in point_cloud2.read_points(msg, field_names=("x", "y"), skip_nans=True):
            x, y = point
            if abs(x) > maximum_range or abs(y) > maximum_range:
                exceptional_count += 1
                exceptional_list.append(int(max(abs(x), abs(y))))

        max_exceptional_value = max(exceptional_list) if exceptional_list else 0
        if max_exceptional_value > 1.2 * maximum_range:
            self.get_logger().error(f' Found {exceptional_count} points with values exceeding max range, with  {max_exceptional_value} as the highest measured  ')


        # Check if the z values are NaNs or Infs, and if the points are within the maximum range.
        invalid_count = 0
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False):
            num_points = msg.width * msg.height
            x, y, z = point
            # Check if any coordinate is NaN or Inf
            if (math.isnan(x) or math.isnan(y) or math.isnan(z) or math.isinf(x) or math.isinf(y) or math.isinf(z)):
                invalid_count += 1
        if (invalid_count)/(num_points) > 0.05:
            self.get_logger().fatal(f'[{time():.2f}] Too many invalid points in the PointCloud2 message: {invalid_count} out of {num_points} total points.')
       
        return


    def check_frame_duplicity(self,msg):
        current_hash = hashlib.md5(msg.data).hexdigest()
        if self.prev_data_hash == current_hash:
            self.get_logger().warn("Frame is identical to the previous frame.")
        else:
            self.prev_data_hash = current_hash
        return


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
