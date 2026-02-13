import rclpy
from rclpy.node import Node

from pynput import keyboard

class WASDNode(Node):

    def __init__(self):
        super().__init__("wasd_node")

        #initialize the keyboard listener
        with keyboard.Listener(on_press=self.on_press) as listener:
            listener.join()

    def on_press(self):
        self.get_logger().info("Got a key")


        

def main(args=None):
    rclpy.init(args=args)

    wasd_node = WASDNode()

    rclpy.spin(wasd_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wasd_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()