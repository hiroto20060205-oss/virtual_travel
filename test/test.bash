#!/bin/bash
# SPDX-FileCopyrightText: 2025 Hiroto Fujitake
# SPDX-License-Identifier: MIT

dir=~
[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws
colcon build --packages-select virtual_travel
source $dir/.bashrc

CSV_PATH="install/virtual_travel/share/virtual_travel/config/location.csv"
res=0

ng () {
    echo "Error at line $1"
    res=1
}

cp $CSV_PATH /tmp/location.csv.bak

# 正常な入力
echo "name,latitude,longitude" > $CSV_PATH
echo "Tokyo,35.0,139.0" >> $CSV_PATH
echo "Osaka,34.0,135.0" >> $CSV_PATH

ros2 run virtual_travel gnss_simulator > /tmp/gnss.log 2>&1 &
NODE_PID=$!
sleep 3

out=$(ros2 topic echo /nearest_location --once --field data)

if [ $? -ne 0 ]; then
    echo "Command failed: ros2 topic echo"
    ng "$LINENO"
fi

echo "Test1 Output: $out"

if [ "$out" != "Osaka" ]; then
    ng "$LINENO"
fi

kill $NODE_PID

cp /tmp/location.csv.bak $CSV_PATH

#CSVファイルがない場合の入力

if [ $res -eq 0 ]; then
    echo "ok"
    exit 0
else
    echo "failed"
    exit 1
fi
