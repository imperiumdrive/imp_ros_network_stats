# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from imp_msgs.msg import ImpRosNetworkMeasurements, ImpRosNetworkStats
from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):
    def __init__(self):
        super().__init__("imp_ros_network_stats_listener")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("window", 100),  # Window size for averaging PDR
                ("timer_period", 0.1),  # Timer period in seconds for sending messages
                (
                    "order_size",
                    0,
                ),  # Window size for the number of backlog packets to be considered
            ],
        )

        self.WINDOW = self.get_parameter(
            "window"
        ).value  # window specifing the number of packets after which the average values need to be calculated
        self.TIMER_PERIOD = self.get_parameter(
            "timer_period"
        ).value  # time interval after which the next packets needs to be sent
        self.ORDER_SIZE = self.get_parameter(
            "order_size"
        ).value  # range of packets (both previous and future) that will be accepted by the listener, in case of irregular order packets reception

        self.sub = self.create_subscription(
            ImpRosNetworkMeasurements, "chatter", self.chatter_callback, 10
        )
        self.num_received = 0  # number of messages received in a window
        self.total_delay = 0.0  # total delay in a window
        self.msg_seq_num = 0  # message sequence number
        self.last_msg_seq_num = 0  # last message sequence number
        self.last_window_size = 0  # size of the last window (for instance, after the completion of 1-100 window, the last_window_size will be 100)

        self.statistics_msg = ImpRosNetworkStats()
        self.pub = self.create_publisher(
            ImpRosNetworkStats,
            "ros_network_stats",
            10,
        )
        self.measurements_msg = ImpRosNetworkMeasurements()
        self.pub_continously = self.create_publisher(
            ImpRosNetworkMeasurements,
            "ros_network_measurements",
            10,
        )

    def check_and_publish_stats(self):
        # Count number of received packets while also considering packets with an order size
        # if self.msg_seq_num in range (self.last_msg_seq_num - self.ORDER_SIZE, self.last_msg_seq_num + self.ORDER_SIZE):
        if self.msg_seq_num > self.last_msg_seq_num:
            self.last_msg_seq_num = self.msg_seq_num
            if (self.msg_seq_num > self.last_window_size + self.WINDOW) and (
                self.num_received != 0
            ):  # the previous issue was when the lister receives the first message that is out of the window.
                self.last_window_size = (
                    self.msg_seq_num // self.WINDOW
                ) * self.WINDOW  # calculate the last window size
                self.publish_measurements(self.total_delay, self.num_received, self.WINDOW)
                self.num_received = 1
            elif self.msg_seq_num <= self.last_window_size - self.WINDOW:
                pass
            else:
                self.num_received += 1
                if self.msg_seq_num % (self.WINDOW) == 0:
                    self.last_window_size = self.msg_seq_num
                    self.publish_measurements(self.total_delay, self.num_received, self.WINDOW)

    def publish_measurements(self, total_delay, num_received, window):
        average_delay = total_delay / num_received
        packet_delivery_ratio = 100.0 * num_received / window
        self.get_logger().info("Average Delay: %.4f seconds" % average_delay)
        self.get_logger().info("Packet Delivery Ratio: %.2f%%" % packet_delivery_ratio)
        self.num_received = 0
        self.total_delay = 0
        self.statistics_msg.avg_delay = average_delay
        self.statistics_msg.avg_pdr = packet_delivery_ratio
        self.statistics_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.statistics_msg)

    def chatter_callback(self, msg):
        # received_timestanp calculation
        current_time = self.get_clock().now().to_msg()
        received_timestamp = current_time.sec + (current_time.nanosec * pow(10, -9))

        sent_timestamp = msg.timestamp
        delay = received_timestamp - sent_timestamp
        self.total_delay += delay

        self.msg_seq_num = msg.msg_seq_num
        self.measurements_msg.msg_seq_num = self.msg_seq_num
        self.measurements_msg.delay = delay
        self.measurements_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_continously.publish(self.measurements_msg)
        self.get_logger().info(
            "I heard msg: [%s], Delay: %.4f seconds"
            % (self.measurements_msg.msg_seq_num, self.measurements_msg.delay)
        )
        self.check_and_publish_stats()


def main(args=None):
    rclpy.init(args=args)

    listener_node = Listener()
    try:
        rclpy.spin(listener_node)
    except KeyboardInterrupt:
        pass

    listener_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
