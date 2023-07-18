import argparse
import logging
import sys

import rclpy
import yaml
from imp_msgs.msg import ImpRosNetworkMeasurements
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self):
        super().__init__("imp_ros_network_stats_talker")
        self.msg_seq_num = 0
        self.pub = self.create_publisher(ImpRosNetworkMeasurements, "chatter", 10)
        self.MAX_SEQ_NUM = 2**31

        self.declare_parameters(
            namespace="",
            parameters=[
                ("window", 100),  # Window size for averaging PDR
                ("timer_period", 0.1),  # Timer period in seconds for sending messages
            ],
        )

        self.window = self.get_parameter("window").value
        self.timer_period = self.get_parameter("timer_period").value
        self.tmr = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        msg = ImpRosNetworkMeasurements()

        # Timestamp calculation
        current_time = self.get_clock().now().to_msg()
        time_nanosecond = current_time.nanosec * pow(10, -9)
        timestamp = current_time.sec + time_nanosecond

        msg.timestamp = timestamp
        msg.msg_seq_num = self.msg_seq_num
        self.get_logger().info(
            "I sent msg: [%s], Timestamp: %.9f seconds"
            % (msg.msg_seq_num, msg.timestamp)
        )
        self.pub.publish(msg)

        self.msg_seq_num += 1
        if self.msg_seq_num >= self.MAX_SEQ_NUM:
            self.msg_seq_num = 0


def main(args=None):
    rclpy.init(args=args)

    talker_node = Talker()
    try:
        rclpy.spin(talker_node)
    except KeyboardInterrupt:
        pass
    talker_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
