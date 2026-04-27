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

class TelemetryNode(Node):
    def __init__(self):
        super().__init__("telemetry_node")

        self.app = app = Flask(__name__)
        self.app.add_url_rule('/', view_func=self.update_vis)

        self.flask_thread = threading.Thread(target=self.run_flask)
        self.flask_thread.daemon = True
        self.flask_thread.start()
        self.refresh_rate = 0.5

        debug = False
        self._init_imu(debug)
        self._init_scan(debug)
    
    def _init_imu(self, debug):
        self.time_window_s = 10.0 # 5 seconds
        self.time_buffer = []
        self.imu_x_buffer = []
        self.imu_y_buffer = []
        self.imu_z_buffer = []
        self.imu_sub = self.create_subscription(
            Vector3, '/imu/accel', self.imu_callback, qos_profile_sensor_data)
        
        # self.plot_update_tick = self.create_timer(0.2, self.update_vis)
        self.imu_fig = Figure(layout="constrained")
        plt.style.use('dark_background')
        self.ax_x, self.ax_y, self.ax_z = self.imu_fig.subplots(3,1)
        self.ax_x.set_title("IMU X Acceleration")
        self.ax_y.set_title("IMU Y Acceleration")
        self.ax_z.set_title("IMU Z Acceleration")
        self.line_x, self.line_y, self.line_z = None, None, None

        if debug:
            self.debug_imu_stream = self.create_timer(0.01, self.dummy_imu_callback)

    def _init_scan(self, debug):
        self.scan_x = []
        self.scan_y = []
        self.integral_scan_x = []
        self.integral_scan_y = []
        self.control_angle = None

        self.lidar_range = 12.0
        self.lidar_scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.integral_scan_sub = self.create_subscription(
            LaserScan, '/integral_scan', self.integral_scan_callback, qos_profile_sensor_data)
        self.control_angle_sub = self.create_subscription(
            Float32, '/optimal_angle', self.control_angle_callback, qos_profile_sensor_data)

        self.scan_fig = Figure(layout="constrained")
        plt.style.use('dark_background')
        self.ax_scan = self.scan_fig.subplots(1)
        self.ax_scan.set_title("Lidar Integration")
        self.ax_scan.set_xlim([-self.lidar_range-1.0,self.lidar_range+1.0])
        self.ax_scan.set_ylim([-self.lidar_range-1.0,self.lidar_range+1.0])
        self.ax_scan.scatter([0], [0], c='b') # Origin
    
        self.scatter_scan = None
        self.scatter_integral_scan = None
        self.control_angle_arrow = None

        if debug:
            self.control_angle = np.pi/6
            scan_path = os.path.join(get_package_share_directory('telemetry'), 'data/laserscan.json')
            with open(scan_path, 'r') as f:
                lidar_scan_load = json.load(f)
                dummy_scan = LaserScan()
                dummy_scan.ranges = [float(r) if r is not None else float('inf') for r in lidar_scan_load['ranges']]
            self.scan_callback(dummy_scan)

    def dummy_imu_callback(self):
        time = self.get_clock().now().nanoseconds / 1e9
        self.time_buffer.append(time)
        self.imu_x_buffer.append(np.sin(time*10))
        self.imu_y_buffer.append(np.cos(time*10))
        self.imu_z_buffer.append(-9.8)
        while (self.time_buffer[-1] - self.time_buffer[0]) > self.time_window_s:
            self.time_buffer.pop(0)
            self.imu_x_buffer.pop(0)
            self.imu_y_buffer.pop(0)
            self.imu_z_buffer.pop(0)

    def imu_callback(self, msg):
        time = self.get_clock().now().nanoseconds / 1e9
        self.time_buffer.append(time)
        self.imu_x_buffer.append(msg.x)
        self.imu_y_buffer.append(msg.y)
        self.imu_z_buffer.append(msg.z)
        while (self.time_buffer[-1] - self.time_buffer[0]) > self.time_window_s:
            self.time_buffer.pop(0)
            self.imu_x_buffer.pop(0)
            self.imu_y_buffer.pop(0)
            self.imu_z_buffer.pop(0)

    def scan_callback(self, msg):
        pts = np.linspace(np.pi, -np.pi, len(msg.ranges))
        self.scan_x = [np.sin(pts[i]) * msg.ranges[i] for i in range(len(msg.ranges)) if np.isfinite(msg.ranges[i])]
        self.scan_y = [np.cos(pts[i]) * msg.ranges[i] for i in range(len(msg.ranges)) if np.isfinite(msg.ranges[i])]

    def integral_scan_callback(self, msg):
        pts = np.linspace(np.pi, -np.pi, len(msg.ranges))
        self.integral_scan_x = [np.sin(pts[i]) * msg.ranges[i] for i in range(len(msg.ranges)) if np.isfinite(msg.ranges[i])]
        self.integral_scan_y = [np.cos(pts[i]) * msg.ranges[i] for i in range(len(msg.ranges)) if np.isfinite(msg.ranges[i])]

    def control_angle_callback(self, msg):
        self.control_angle = msg.data

    def update_imu_plot(self):
        if self.time_buffer:
            if self.line_x is None:
                self.line_x, = self.ax_x.plot(self.time_buffer, self.imu_x_buffer, c='g')
                self.line_y, = self.ax_y.plot(self.time_buffer, self.imu_y_buffer, c='g')
                self.line_z, = self.ax_z.plot(self.time_buffer, self.imu_z_buffer, c='g')
            else:
                self.ax_x.set_xlim([np.min(self.time_buffer), np.max(self.time_buffer)])
                self.ax_y.set_xlim([np.min(self.time_buffer), np.max(self.time_buffer)])
                self.ax_z.set_xlim([np.min(self.time_buffer), np.max(self.time_buffer)])
                self.ax_x.set_ylim([np.min(self.imu_x_buffer)-1e-3, np.max(self.imu_x_buffer)+1e-3])
                self.ax_y.set_ylim([np.min(self.imu_y_buffer)-1e-3, np.max(self.imu_y_buffer)+1e-3])
                self.ax_z.set_ylim([np.min(self.imu_z_buffer)-1e-3, np.max(self.imu_z_buffer)+1e-3])
                self.line_x.set_data(self.time_buffer, self.imu_x_buffer)
                self.line_y.set_data(self.time_buffer, self.imu_y_buffer)
                self.line_z.set_data(self.time_buffer, self.imu_z_buffer)

    def update_scan_plot(self):
        if self.scan_x:
            if self.scatter_scan is None:
                self.scatter_scan = self.ax_scan.scatter(self.scan_x, self.scan_y, c='r', s=2)
            else:
                pts = list(zip(self.scan_x, self.scan_y))
                self.scatter_scan.set_offsets(pts)
        if self.integral_scan_x:
            if self.scatter_integral_scan is None:
                self.scatter_integral_scan = self.ax_scan.scatter(self.integral_scan_x, self.integral_scan_y, c='g', s=2)
            else:
                pts = list(zip(self.integral_scan_x, self.integral_scan_y))
                self.scatter_integral_scan.set_offsets(pts)
        if self.control_angle is not None:
            if self.control_angle_arrow is None:
                self.control_angle_arrow = self.ax_scan.arrow(0, 0, np.sin(-self.control_angle), np.cos(-self.control_angle), color='blue')
            else:
                self.control_angle_arrow.set_data(dx=np.sin(self.control_angle), dy=np.cos(self.control_angle))
            

    def update_vis(self):
        self.update_imu_plot()
        self.update_scan_plot()

        imu_buf = BytesIO()
        self.imu_fig.savefig(imu_buf, format="png")
        imu_data = base64.b64encode(imu_buf.getbuffer()).decode("ascii")

        scan_buf = BytesIO()
        self.scan_fig.savefig(scan_buf, format="png")
        scan_data = base64.b64encode(scan_buf.getbuffer()).decode("ascii")

        return f"""
        <html>
            <head></head>
            <body style="background-color: black;">
                <h2 style="color: white; font-family: 'Lucida Console', Courier, monospace;">Clanker Collective Data Stream</h1>
                <div style="display: flex; gap: 10px;">
                    <meta http-equiv="refresh" content="{self.refresh_rate}">
                    <img src='data:image/png;base64,{imu_data}' style="width: 50%;"/>
                    <img src='data:image/png;base64,{scan_data}' style="width: 50%;"/>
                </div>
            </body>
        </html>
        """

    def run_flask(self):
        self.app.run(host='0.0.0.0', port=5001, debug=False, use_reloader=False)
        

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
