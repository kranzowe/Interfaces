import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class ParamsNode(Node):
    def __init__(self):
        super().__init__("params_node")
        self.declare_parameters(
                namespace="",
                parameters=[
                    ("max_steer", Parameter.Type.DOUBLE),
                    ("track_width", Parameter.Type.DOUBLE),
                    ("wheelbase", Parameter.Type.DOUBLE),
                    ("front_area", Parameter.Type.DOUBLE),
                ])

def main(args=None):
    rclpy.init(args=args)
    node = ParamsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
