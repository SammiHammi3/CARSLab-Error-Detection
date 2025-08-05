import rclpy
from rclpy.node import Node                     # use this to tell ROS2 that this is a ROS2 node
from sensor_msgs.msg import PointCloud2         # needed for interpreting the pointcloud messages
import sys
import math     #used for IsNaN / IsInf
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs_py import point_cloud2         # needed for interpreting the pointcloud messages

from lidar_interfaces.msg import LidarAlert     # From [the name of the package] import [the name of the message type]

sys.stdout.reconfigure(line_buffering=True)


message_counter = 0
maximum_range = 200                             # maximum range, in meters, according to the Pandar website

# how far from expected values can we deviate?
# note that 0.01 means 1%, not 0.01% (for example)
percent_invalid_point_count_allowed = 0.001     # what portion of points can be invalid before we flag the message
percent_exceptional_point_count_allowed = 0.01  # what portion of points can be outside the maximum range before we flag the message



class MessageSearcher(Node):        # This is the name of the class of node!
    def __init__(self):
        super().__init__('message_searcher')    # This is the name of the node!
        
        self.subscription = self.create_subscription(
            PointCloud2,                        # This is the type of message we are subscribing to
            '/pointcloud',                      # This is the name of the topic to subscribe to
            self.listener_callback,
            qos_profile_sensor_data             # This is the QoS profile for the subscription, which is set to sensor data
)
        self.subscription  
        self.publisher = self.create_publisher(LidarAlert, 'lidar_logs', 10)     # Message Type, name of topic to send to, queue depth

    def listener_callback(self, msg):
        self.check_data(msg)
        pass

# Sample message:
# self.publisher.publish(LidarAlert(
#     level=b"2",
#     error_name="Sample Error",
#     description="This is a sample error message."
# ))

    def check_data(self, msg):
        global message_counter
        message_counter += 1
        
        exceptional_count = 0
        invalid_count = 0
        num_points = msg.width * msg.height if msg.is_dense else len(msg.data) // (msg.point_step)
        
        if message_counter == 20:
            message_counter = 0

            for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False):
                x, y, z = point

                # Check if any coordinate is NaN or Inf
                if not all(math.isfinite(v) for v in (x, y, z)):
                    invalid_count += 1
                        
                # Check if the point is within the maximum range using pythagorean theorem
                distance = math.sqrt(x*x + y*y + z*z)
                if distance > maximum_range:
                    exceptional_count += 1
                    
                    
            # If the ratio of exceptional to total is greater than the percent allowed, we publish a warning
            if (exceptional_count / num_points) > percent_exceptional_point_count_allowed:
                    self.publisher.publish(LidarAlert(
                    level=b"2", # WARN level
                    error_name="Exceptional Point Count",
                    description=f"Received {exceptional_count} points outside the maximum range of {maximum_range} meters."
                    ))
                        
                    # If the ratio of invalid to total is greater than the percent allowed, we publish an error alert
            if (invalid_count / num_points) > percent_invalid_point_count_allowed:
                    self.publisher.publish(LidarAlert(
                    level=b"3", # ERROR level
                    error_name="Invalid Point Count",
                    description=f"Received {invalid_count} invalid points"
                    ))
        return

    
def main(args=None):
    print("Starting MessageSearcher Node")
    rclpy.init(args=args)
    node = MessageSearcher()            # This should match the class name earlier

    print("Node initialized, spinning now.")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    print("MessageSearcher Node has been shut down.")
