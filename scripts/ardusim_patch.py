#!/usr/bin/env python3

'''
This is the code forked from "https://github.com/bvibhav/stonefish_bluerov2". In my setup I got different small problems, 
mainly given to different coordinate system and time sync. 

Note that this bridge is ment to work with stonefish 1.3. If you are using stonefish 1.5 or above there is no need to add 
the gravity component as it is already included. 

The code works by reading the ros2 topic opened by Stonefish_ros2 bridge and sending to the Ardupilot udp port in order
to pass the necessary parameters. In https://github.com/ardupilot/ardupilot/blob/master/libraries/SITL/examples/JSON/readme.md)
is possible to find all the parameter needed to be sent, along side the reference system required.

Reference system:
-both ardusim and stonefish works in NED, so the z axis is pointing down.
-notabily mavros works in ENU, and if you echo such topic the Z/depth is still negative. Thats not an error

TODO: 
- check if the compass computed in the ardupilot simulation loop

'''
import rclpy
from rclpy.node import Node

import socket
import struct
import json
import time 
import math 
import threading
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np
from tf_transformations import quaternion_from_euler, euler_from_quaternion, quaternion_matrix

import numpy as np

class Patch(Node):
    def __init__(self, node_name, namespace):
        super().__init__(node_name, namespace=namespace)

        self.namespace = self.get_namespace()[1:]

        # Subscribers
        self.create_subscription(Imu, "imu", self._imu_callback, 1)
        self.create_subscription(NavSatFix, "gps", self._gps_callback, 1)
        # senza mettere lo slash davanti prende il nome del namespace, con "/" è assoluto
        self.create_subscription(Odometry, "odometry", self._odom_callback, 1)

        # Publishers
        self.pub_pwm = self.create_publisher(Float64MultiArray, "setpoint/pwm", 1)

        PORT = 9002 # Default port

        self.sock_sitl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_sitl.bind(('', PORT))
        
        # timeout to check if the SITL is still sending on the port
        self.sock_sitl.settimeout(1.0)

        self.imu = None
        self.gps = None
        self.odom = None
        self.last_frame_count = None
        
        # The callback for the IMU and odom runs on the spin thread. The SITL loop works on another thread,
        # we receive the PWM and give back the state for each frame (which is defined by SITL)
        self._running = True
        self._SITL_thread = threading.Thread(target=self._sitl_loop, daemon=True)
        self._SITL_thread.start()
        
        # From previus code (still haven't looked into)
        # self.gps_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # IPV4, UDP
        # self.gps_addr = ("127.0.0.1", 25100)

    def _imu_callback(self, msg):
        self.imu = msg

    def _gps_callback(self, msg):
        self.gps = msg

    def _odom_callback(self, msg):
        self.odom = msg

    def _sitl_loop(self):
        # Lock step: we stop wehen receiving a pwm and send back. No timer but we give back as we have a frame_in
        while self._running and rclpy.ok():
            try:
                data, address = self.sock_sitl.recvfrom((100))
            except socket.timeout:
                self.get_logger().warn(
                    "No frame from SITL, is it running?",
                throttle_duration_sec=2.0)
                continue
            except Exception:
                continue
            
            self._handle_frame(data, address)

    
    def _handle_frame(self, data, address):
        parse_format = 'HHI16H'
        magic = 18458
        
        # Check if the data is in the format I need
        if len(data) != struct.calcsize(parse_format):
            self.get_logger().warn(
                "got packet of len %u, expected %u" % (len(data), struct.calcsize(parse_format)),
                throttle_duration_sec = 1.0)
            return 
        
        decoded = struct.unpack(parse_format, data)
        
        if magic != decoded[0]:
            self.get_logger().warn(
                "Incorrect protocol magic %u should be %u" % (decoded[0], magic),
                 throttle_duration_sec = 1.0)
            return 
        
        frame_rate_hz = decoded[1]
        frame_count = decoded[2]
        
        if self.last_frame_count is not None:
            delta = frame_count - self.last_frame_count
            if delta !=1:
                self.get_logger().warn(
                    f"SITL frame jump/backlog: delta={delta}",
                    throttle_duration_sec=1.0)
                
            
        self.last_frame_count = frame_count
        
        pwm = decoded[3:]
        pwm_setpoint = []
       
        if self.namespace=='bluerov2':
            pwm_thrusters = pwm[0:8]
            pwm_setpoint = [(x-1500)/400 for x in pwm_thrusters]
            
        # TODO add our simulated sub
        
        # ------------- Publish the PWM to Stonefish -------------
        msg_pwm = Float64MultiArray(data=pwm_setpoint)
        # Publish pwm message
        self.pub_pwm.publish(msg_pwm)

        # ------------- Receive and elaborate the sensors/pose -------------
        # We wait for IMU and odom before sending
        if self.imu is None or self.odom is None:
            self.get_logger().info("Waiting for callbacks", throttle_duration_sec=1.0)
            return

        self.get_logger().info("Callbacks received", once=True)
        
        quaternion = [
            self.odom.pose.pose.orientation.x,
            self.odom.pose.pose.orientation.y,
            self.odom.pose.pose.orientation.z,
            self.odom.pose.pose.orientation.w,
        ]
        g_w = np.array([0,0,-9.81]) # la reazione alla gravità è verso l'alto. siccome siamo in NED è negativo 
        R = quaternion_matrix(quaternion)[:3, :3]  # estrai solo la parte 3x3
        g_b = R.T@g_w
        
        # Set mesasges
        accel_nogravitiy = np.array([self.imu.linear_acceleration.x, 
                            self.imu.linear_acceleration.y, 
                            self.imu.linear_acceleration.z])
        accel = tuple(accel_nogravitiy + g_b)
        
        # # log PRIMA della conversione
        # self.get_logger().info(
        #     "Acceleration  ax={:+.2f}  ay={:+.2f}  az={:+.2f}".format(
        #         accel[0], accel[1], accel[2],
        #     ), throttle_duration_sec=1.0)
                
        gyro = (self.imu.angular_velocity.x, 
                self.imu.angular_velocity.y, 
                self.imu.angular_velocity.z)
        
        pose_position = (
            self.odom.pose.pose.position.x,
            self.odom.pose.pose.position.y,
            self.odom.pose.pose.position.z
        )

        r, p, y = euler_from_quaternion(quaternion)
        
        pose_attitude = [r, p, y]
        # self.get_logger().info(
        #     "Yaw:", np.rad2deg(pose_attitude[2]), throttle_duration_sec=1.0
        # )
        
        
        twist_linear = (
            self.odom.twist.twist.linear.x,
            self.odom.twist.twist.linear.y,
            self.odom.twist.twist.linear.z,
        )
        v_body = np.array(twist_linear)
        v_world = R@v_body

        # The time is taken b the SITL (frame_count/frame_rate_hz), ROS time was probably 
        # giving jitter before.
        c_time = frame_count / frame_rate_hz if frame_rate_hz else 0.0

        # build JSON format
        IMU_fmt = {
            "gyro" : gyro,
            "accel_body" : accel
        }
        JSON_fmt = {
            "timestamp" : c_time,
            "imu" : IMU_fmt,
            "position" : pose_position,
            "attitude" : pose_attitude,
            "velocity" : tuple(v_world),                          
        }
        JSON_string = "\n" + json.dumps(JSON_fmt,separators=(',', ':')) + "\n"

        # Send to AP
        self.sock_sitl.sendto(bytes(JSON_string,"ascii"), address)
        
        # print(self.gps.latitude)

        # gps_data = {
        #         'time_usec' : int(c_time/1e3),                        # (uint64_t) Timestamp (micros since boot or Unix epoch)
        #         'gps_id' : 0,                           # (uint8_t) ID of the GPS for multiple GPS inputs
        #         # 'ignore_flags' : 8,                     # (uint16_t) Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum). All other fields must be provided.
        #         # 'time_week_ms' : 0,                     # (uint32_t) GPS time (milliseconds from start of GPS week)
        #         # 'time_week' : 0,                        # (uint16_t) GPS week number
        #         # 'fix_type' : 3,                         # (uint8_t) 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
        #         'lat' : int(self.gps.latitude*1e7),                              # (int32_t) Latitude (WGS84), in degrees * 1E7
        #         'lon' : int(self.gps.longitude*1e7),                              # (int32_t) Longitude (WGS84), in degrees * 1E7
        #         'alt' : 0,                              # (float) Altitude (AMSL, not WGS84), in m (positive for up)
        #         # 'hdop' : 1,                             # (float) GPS HDOP horizontal dilution of position in m
        #         # 'vdop' : 1,                             # (float) GPS VDOP vertical dilution of position in m
        #         # 'vn' : 0,                               # (float) GPS velocity in m/s in NORTH direction in earth-fixed NED frame
        #         # 've' : 0,                               # (float) GPS velocity in m/s in EAST direction in earth-fixed NED frame
        #         # 'vd' : 0,                               # (float) GPS velocity in m/s in DOWN direction in earth-fixed NED frame
        #         # 'speed_accuracy' : 0,                   # (float) GPS speed accuracy in m/s
        #         # 'horiz_accuracy' : 0,                   # (float) GPS horizontal accuracy in m
        #         # 'vert_accuracy' : 0,                    # (float) GPS vertical accuracy in m
        #         # 'satellites_visible' : 7                # (uint8_t) Number of satellites visible.
        # }

        # gps_data = json.dumps(gps_data)
        # self.gps_sock.sendto(gps_data.encode(), ("127.0.0.1", 25100))
            
    
def main(args=None):
    rclpy.init(args=args)

    # patch = Patch(node_name="ardusim_patch")
    patch = Patch(node_name="ardusim_patch", namespace='bluerov2')
    
    try:
        rclpy.spin(patch)
    except KeyboardInterrupt:
        pass

    # We stop the communication thread before closing
    patch._running = False

    # Destroy the node explicitly, otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    patch.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
