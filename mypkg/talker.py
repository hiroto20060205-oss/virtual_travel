#!/usr/bin/python3
# SPDX-FileCopyrightText: 2025 Fujitake Hiroto
# SPDX-License-Identifier: MIT

import rclpy
from rclpy.node import Node
from mypkg_msgs.msg import Location
from sensor_msgs.msg import NavSatFix
from geopy.distance import geodesic

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.pub = self.create_publisher(NavSatFix, 'gnss_fix', 10)
        self.create_timer(0.1, self.cb)

        # 宿場町リスト
        self.stations = [
            ("日本橋", 35.6835, 139.7713),
            ("品川宿", 35.6209, 139.7402),
            ("川崎宿", 35.5323, 139.7042),
            ("神奈川宿", 35.4741, 139.6300),
            ("保土ヶ谷宿", 35.4452, 139.5960),
            ("戸塚宿", 35.3954, 139.5298),
            ("藤沢宿", 35.3448, 139.4886),
            ("平塚宿", 35.3275, 139.3330),
            ("大磯宿", 35.3090, 139.3150),
            ("小田原宿", 35.2536, 139.1554),
            ("箱根宿", 35.1917, 139.0255),
            ("三島宿", 35.1192, 138.9130),
            ("沼津宿", 35.1001, 138.8596),
            ("原宿", 35.1255, 138.7967),
            ("吉原宿", 35.1627, 138.6847),
            ("蒲原宿", 35.1172, 138.6041),
            ("由比宿", 35.0935, 138.5563),
            ("興津宿", 35.0506, 138.5205),
            ("江尻宿", 35.0195, 138.4844),
            ("府中宿", 34.9715, 138.3828),
            ("丸子宿", 34.9547, 138.3562),
            ("岡部宿", 34.9195, 138.2868),
            ("藤枝宿", 34.8703, 138.2581),
            ("島田宿", 34.8322, 138.1742),
            ("金谷宿", 34.8188, 138.1257),
            ("日坂宿", 34.8055, 138.0773),
            ("掛川宿", 34.7733, 138.0145),
            ("袋井宿", 34.7505, 137.9254),
            ("見付宿", 34.7176, 137.8546),
            ("浜松宿", 34.7047, 137.7335),
            ("舞阪宿", 34.6833, 137.6083),
            ("新居宿", 34.6933, 137.5619),
            ("白須賀宿", 34.6995, 137.5303),
            ("二川宿", 34.7247, 137.4262),
            ("吉田宿", 34.7675, 137.3912),
            ("御油宿", 34.8341, 137.3160),
            ("赤坂宿", 34.8553, 137.3069),
            ("藤川宿", 34.9133, 137.2307),
            ("岡崎宿", 34.9587, 137.1601),
            ("池鯉鮒宿", 35.0039, 137.0392),
            ("鳴海宿", 35.0805, 136.9507),
            ("宮宿", 35.1271, 136.9080),
            ("桑名宿", 35.0664, 136.6974),
            ("四日市宿", 34.9658, 136.6264),
            ("石薬師宿", 34.8963, 136.5513),
            ("庄野宿", 34.8833, 136.5292),
            ("亀山宿", 34.8519, 136.4542),
            ("関宿", 34.8517, 136.3934),
            ("坂下宿", 34.8913, 136.3533),
            ("土山宿", 34.9333, 136.2731),
            ("水口宿", 34.9655, 136.1683),
            ("石部宿", 35.0064, 136.0542),
            ("草津宿", 35.0189, 135.9622),
            ("大津宿", 35.0088, 135.8617),
            ("三条大橋", 35.0054, 135.7697)
        ]

        self.current_index = 0

        start_name, start_latitude, start_longitude = self.stations[0]
        self.latitude = start_latitude
        self.longitude = start_longitude

        self.latitude_step = 0
        self.longitude_step = 0
        self.remaining_steps = 0

        self.plan_next_trip()

    def plan_next_trip(self):
        if self.current_index >= len(self.stations) - 1:
            return None

        current_name, current_latitude, current_longitude = self.stations[self.current_index]
        next_name, next_latitude, next_longitude = self.stations[self.current_index + 1]

        position_current = (current_latitude, current_longitude)
        position_next = (next_latitude, next_longitude)
        distance = geodesic(position_current, position_next).meters

        speed_per_step = 100.0

        steps = int(distance / speed_per_step)
        self.remaining_steps = max(1, steps)

        self.latitude_step = (next_latitude - current_latitude) / self.remaining_steps
        self.longitude_step = (next_longitude - current_longitude) / self.remaining_steps

    def cb(self):
        msg = NavSatFix()

        msg.latitude = self.latitude
        msg.longitude = self.longitude

        self.pub.publish(msg)

        if self.remaining_steps > 0:
            self.latitude += self.latitude_step
            self.longitude += self.longitude_step
            self.remaining_steps -= 1

            if self.remaining_steps % 10 == 0 and self.remaining_steps != 0:
                target_name = self.stations[self.current_index + 1][0]
                self.get_logger().info(f"{target_name}へ移動中...🚶 (残り{self.remaining_steps}ステップ)")

        elif self.current_index < len(self.stations) - 1:
            next_name, next_latitude, next_longitude = self.stations[self.current_index + 1]
            self.latitude = next_latitude
            self.longitude = next_longitude

            self.current_index += 1
            self.plan_next_trip()

        else:
            self.get_logger().info("京都（三条大橋）に到着しました！旅は終わりです。")

def main():
    rclpy.init()
    node = Talker()
    rclpy.spin(node)
