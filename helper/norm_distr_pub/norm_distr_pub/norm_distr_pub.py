
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64


class NormalDistributionPublisher(Node):

    def __init__(self):
        super().__init__("normal_distribution_publisher")
        self.publisher_ = self.create_publisher(Float64, "norm_distr", 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float64()
        mu, sigma = 0, 0.1
        s = np.random.normal(mu, sigma, 1)
        msg.data = s[0]
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = NormalDistributionPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
