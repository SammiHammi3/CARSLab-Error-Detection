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
delta_variance_allowed = 0.1 # what is the percent variance from the delta time between messages that we allow before we flag a warning

message_rate_threshold = 1 # hz in either direction flags a warning
missed_messages_allowed = 5 # how many messages can we miss before we flag a warning

ideal_message_rate = 10 # frequency we expect to receive messages at, in Hz
ideal_point_count = 230400 # ideal number of points in a message
maximum_range = 200 # meters, with the pandar lidar at 10% reflectivity
msg_delta = 1/ideal_message_rate # seconds, this is the time between messages we expect

max_points = ideal_point_count * (1 + percent_deviation_allowed)
min_points = ideal_point_count * (1 - percent_deviation_allowed)

frequency_list = []

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
        #Map for RVIZ2: velodyne_top_base_link
        self.check_data(msg)
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
                
        # this to change stuff!!!!!

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
        
            
            
            
        #the problem with this is that if a message is dropped, the frequency decreases. 
        #what we need to do is artificially give each dropped message a timestamp of ~0.1s + previous message time
        #what this will do is allow us to guesstimate where the message would have been in the frequency list
        #and thus not skew the frequency too much
        #we can alert the user if more than a couple messages are dropped in a row using this method

        # The current message time, in seconds and nanoseconds
        msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.prev_msg_time is not None:
            time_diff = msg_time - self.prev_msg_time #checks time between two most recent received messages. note that missed messages may lurk between!
            if time_diff > (delta_variance_allowed+1) * msg_delta: # if the time difference is greater than the allowed variance, it means >0 messages missed
                missed_messages = int((msg_time - self.prev_msg_time) / msg_delta)  # messages missed = delta time / expected delta time
                if missed_messages > missed_messages_allowed:
                    log.warn(f"Missed {missed_messages} messages, assuming they were dropped.")
            else:
                missed_messages = 0
            
            # frequency is 1 / time difference times the number of missed messages. 
            #for example, 1 missed message halves the frequency, so multiply by 2.
            #2 missed messages thirds the frequency (from 10 to 3.333) so multiply by 3. etc.
            frequency = (1 / time_diff)*(1+missed_messages) if time_diff > 0 else 0 
            
            if abs(frequency - ideal_message_rate) > message_rate_threshold:
                log.warn(f"Message rate deviation: expected ~{ideal_message_rate:.2f}Hz, got {frequency:.2f} Hz")

        # update the frequency so the current one becomes the old one
        self.prev_msg_time = msg_time
            
            
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