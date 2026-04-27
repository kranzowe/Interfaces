import base64
from io import BytesIO
from flask import Flask
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import numpy as np
from rclpy.qos import qos_profile_sensor_data

class TelemetryNode(Node):
    def __init__(self):
        super().__init__("telemetry_node")

        self.app = app = Flask(__name__)
        self.app.add_url_rule('/', view_func=self.update_vis)

        self.flask_thread = threading.Thread(target=self.run_flask)
        self.flask_thread.daemon = True
        self.flask_thread.start()

        self._init_imu()

    
    def _init_imu(self):
        self.time_window_s = 10.0 # 5 seconds
        self.time_buffer = []
        self.imu_x_buffer = []
        self.imu_y_buffer = []
        self.imu_z_buffer = []
        self.imu_sub = self.create_subscription(
            Vector3, '/imu/accel', self.imu_callback, qos_profile_sensor_data)
        
        # self.plot_update_tick = self.create_timer(0.2, self.update_vis)
        self.fig = Figure(layout="constrained")
        plt.style.use('dark_background')
        self.ax_x, self.ax_y, self.ax_z = self.fig.subplots(3,1)
        self.ax_x.set_title("IMU X Acceleration")
        self.ax_y.set_title("IMU Y Acceleration")
        self.ax_z.set_title("IMU Z Acceleration")
        self.line_x, self.line_y, self.line_z = None, None, None

        debug = False
        if debug:
            self.debug_data_stream = self.create_timer(0.01, self.dummy_imu_callback)


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
        print(len(self.time_buffer))
        print(msg)
        time = node.get_clock().now().nanoseconds / 1e9
        self.time_buffer.append(time)
        self.imu_x_buffer.append(msg.x)
        self.imu_y_buffer.append(msg.y)
        self.imu_z_buffer.append(msg.z)
        while (self.time_buffer[-1] - self.time_buffer[0]) > self.time_window_s:
            self.time_buffer.pop(0)
            self.imu_x_buffer.pop(0)
            self.imu_y_buffer.pop(0)
            self.imu_z_buffer.pop(0)
    
    def update_imu_data(self):
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

    def update_vis(self):
        self.update_imu_data()

        buf = BytesIO()
        self.fig.savefig(buf, format="png")
        data = base64.b64encode(buf.getbuffer()).decode("ascii")

        return f"""
        <html>
            <head><meta http-equiv="refresh" content="0.2"></head>
            <body style="background-color: black;">
                <h2 style="color: white; font-family: 'Lucida Console', Courier, monospace;">Clanker Collective Data Stream</h1>
                <img src='data:image/png;base64,{data}'/>
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
