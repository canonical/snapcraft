import rclpy
from std_msgs.msg import String


class ListenerNode:
    def __init__(self, node_name):
        self._node = rclpy.Node(node_name)
        self._node.create_subscription(String, "chatter", self.callback)

    def callback(self, message):
        print("I heard: {}".format(message.data))
        self._node.destroy_node()
        rclpy.shutdown()

    def run(self):
        rclpy.spin(self._node)


def main():
    rclpy.init()

    node = ListenerNode("listener")
    node.run()


if __name__ == "__main__":
    main()
