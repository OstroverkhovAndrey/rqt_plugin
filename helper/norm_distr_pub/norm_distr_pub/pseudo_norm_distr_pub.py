
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64
import random


class PseudoNormalDistributionPublisher(Node):

    def __init__(self):
        super().__init__("dseudo_normal_distribution_publisher")
        self.publisher_ = self.create_publisher(Float64, "pseudo_norm_distr", 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float64()
        s = []
        s += [0.0] * 10
        s += [0.1] * 7
        s += [-0.13] * 6
        s += [0.2] * 3
        s += [-0.22] * 2
        msg.data = random.choice(s)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = PseudoNormalDistributionPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
