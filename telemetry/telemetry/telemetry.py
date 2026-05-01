import base64
from io import BytesIO
from flask import Flask
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np
import os
import json
from rclpy.qos import qos_profile_sensor_data
from ament_index_python import get_package_share_directory

from telemetry.imu_handler import ImuHandler
from telemetry.gyro_handler import GyroHandler
from telemetry.scan_handler import ScanHandler
from telemetry.map_handler import MapHandler

class TelemetryNode(Node):
    def __init__(self):
        super().__init__("telemetry_node")

        self.app = app = Flask(__name__)
        self.app.add_url_rule('/', view_func=self.update_vis)

        self.flask_thread = threading.Thread(target=self.run_flask)
        self.flask_thread.daemon = True
        self.flask_thread.start()

        self.declare_parameter("refresh_rate", 0.5)
        self.declare_parameter("gc_rate", 4.0)
        self.declare_parameter("debug", False)
        self.refresh_rate = self.get_parameter("refresh_rate").value
        self.gc_rate = self.get_parameter("gc_rate").value
        self.debug = self.get_parameter("debug").value
        self.create_timer(0.5, self.param_callback)

        self.gyro_handler = GyroHandler(self, self.debug)
        self.imu_handler = ImuHandler(self, self.debug)
        self.scan_handler = ScanHandler(self, self.debug)
        self.map_handler = MapHandler(self, self.debug)

    def update_vis(self):
        imu_fig = self.imu_handler.update_imu_plot()
        imu_buf = BytesIO()
        imu_fig.savefig(imu_buf, format="png")
        imu_data = base64.b64encode(imu_buf.getbuffer()).decode("ascii")

        gyro_fig = self.gyro_handler.update_imu_plot()
        gyro_buf = BytesIO()
        gyro_fig.savefig(gyro_buf, format="png")
        gyro_data = base64.b64encode(gyro_buf.getbuffer()).decode("ascii")

        scan_fig = self.scan_handler.update_scan_plot()
        scan_buf = BytesIO()
        scan_fig.savefig(scan_buf, format="png")
        scan_data = base64.b64encode(scan_buf.getbuffer()).decode("ascii")

        map_fig = self.map_handler.update_map_plot()
        map_buf = BytesIO()
        map_fig.savefig(map_buf, format="png")
        map_data = base64.b64encode(map_buf.getbuffer()).decode("ascii")

        return f"""
        <html>
            <head></head>
            <body style="background-color: black;">
                <h2 style="color: white; text-align: center; font-family: 'Monaco', Courier, monospace;">Clanker Collective Telemetry Stream</h1>
                <div>
                    <meta http-equiv="refresh" content="{self.refresh_rate}">
                    <div style="display: flex; gap: 10px; padding: 10px;">
                        <img src='data:image/png;base64,{map_data}' style="width: 50%;"/>
                        <img src='data:image/png;base64,{scan_data}' style="width: 50%;"/>
                    </div>
                    <div style="display: flex; gap: 10px; padding: 10px;">
                        <img src='data:image/png;base64,{imu_data}' style="width: 50%;"/>
                        <img src='data:image/png;base64,{gyro_data}' style="width: 50%;"/>
                    </div>
                </div>
            </body>
        </html>
        """

    def run_flask(self):
        self.app.run(host='0.0.0.0', port=5001, debug=False, use_reloader=False)

    def param_callback(self):
        self.refresh_rate = self.get_parameter("refresh_rate").value
        self.gc_rate = self.get_parameter("gc_rate").value

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
