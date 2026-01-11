#!/usr/bin/python3
# SPDX-FileCopyrightText: 2025-2026 Fujitake Hiroto
# SPDX-License-Identifier: MIT

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Float32
from geopy.distance import geodesic
import csv
import os
from ament_index_python.packages import get_package_share_directory

class Navigation_node(Node):
    def __init__(self):
        super().__init__('navigation')
        self.pub = self.create_publisher(NavSatFix, 'gnss_fix', 10)
        self.pub_name = self.create_publisher(String, 'nearest_location', 10)
        self.pub_distance = self.create_publisher(Float32, 'distance_to_target', 10)

        self.create_timer(0.1, self.cb)

        self.stations =[]

        package_name = 'virtual_travel'
        pkg_share = get_package_share_directory(package_name)
        csv_path = os.path.join(pkg_share, 'config', 'location.csv')
        self.load_stations(csv_path)

        self.current_index = 0

        if self.stations:
            self.latitude = self.stations[0][1]
            self.longitude = self.stations[0][2]
        else:
            self.latitude = 0.0
            self.longitude = 0.0

        self.latitude_step = 0
        self.longitude_step = 0
        self.remaining_steps = 0

        if len(self.stations) > 1:
            self.plan_next_trip()

    def load_stations(self, path):
        try:
            with open(path, 'r', encoding='utf-8') as f:
                reader = csv.reader(f)
                next(reader, None)
                for row in reader:
                    try:
                        if len(row) < 3: 
                            continue
                        self.stations.append((row[0], float(row[1]), float(row[2])))
                    except ValueError:
                        continue
            self.get_logger().info(f"CSV読み込み成功: {path} ({len(self.stations)}件)")
        except FileNotFoundError:
            self.get_logger().error(f"ファイルが見つかりません: {path}")
            self.stations = []

    def plan_next_trip(self):
        if self.current_index >= len(self.stations) - 1:
            return None

        current_name = self.stations[self.current_index]
        next_name = self.stations[self.current_index + 1]

        position_current = (current_name[1], current_name[2])
        position_next = (next_name[1], next_name[2])
        distance = geodesic(position_current, position_next).meters

        speed_per_step = 100.0

        steps = int(distance / speed_per_step)
        self.remaining_steps = max(1, steps)

        self.latitude_step = (next_name[1] - current_name[1]) / self.remaining_steps
        self.longitude_step = (next_name[2] - current_name[2]) / self.remaining_steps

    def cb(self):
        msg = NavSatFix()
        msg.latitude = self.latitude
        msg.longitude = self.longitude
        self.pub.publish(msg)

        if not self.stations:
            self.pub_name.publish(String(data="Goal"))
            self.pub_distance.publish(Float32(data=0.0))
            return

        target_name = "Goal"
        distance = 0.0

        if self.current_index < len(self.stations) - 1:
            next_info = self.stations[self.current_index + 1]
            target_name = next_info[0]
            position_current = (self.latitude, self.longitude)
            position_next = (next_info[1], next_info[2])
            distance = geodesic(position_current, position_next).meters

        self.pub_name.publish(String(data=target_name))
        self.pub_distance.publish(Float32(data=distance))

        if self.remaining_steps > 0:
            self.latitude += self.latitude_step
            self.longitude += self.longitude_step
            self.remaining_steps -= 1

        elif self.current_index < len(self.stations) - 1:
            next_name = self.stations[self.current_index + 1]
            self.latitude = next_name[1]
            self.longitude = next_name[2]
            self.current_index += 1
            self.plan_next_trip()

def main():
    rclpy.init()
    node = Navigation_node ()
    rclpy.spin(node)
