import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String


class Listener(Node):
    def __init__(self):
        super().__init__("listener")

        parameter = self.declare_parameter(
            "exit_after_receive", False, ParameterDescriptor()
        )
        self._exit_after_receive = parameter.value
        self.should_exit = False

        qos_profile = QoSProfile(depth=1)
        self._subscription = self.create_subscription(
            String, "chatter", self._callback, qos_profile
        )

    def _callback(self, message):
        self.get_logger().info("I heard {!r}".format(message.data))
        if self._exit_after_receive:
            self.get_logger().info(
                "Requested to exit after message received. Exiting now."
            )
            self.should_exit = True


def main(args=None):
    rclpy.init(args=args)

    listener = Listener()

    while rclpy.ok():
        rclpy.spin_once(listener)
        if listener.should_exit:
            break

    listener.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
