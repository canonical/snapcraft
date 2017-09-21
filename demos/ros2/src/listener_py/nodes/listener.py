import rclpy
from std_msgs.msg import String


def callback(message):
    print('I heard: {}'.format(message.data))


def main():
    rclpy.init()

    node = rclpy.Node('listener')

    node.create_subscription(String, 'chatter', callback)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
