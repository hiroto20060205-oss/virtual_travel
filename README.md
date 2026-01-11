# 仮想GNSSを使った経由地移動シミュレーター
![test](https://github.com/hiroto20060205-oss/virtual_travel/actions/workflows/test.yml/badge.svg)

## 概要
本リポジトリの`config`ディレクトリにある[location.csv](https://github.com/hiroto20060205-oss/virtual_travel/blob/dev/config/location.csv)の地名、緯度、経度を読み込んで、現在地と目的地までの距離を算出するパッケージです。
次の地点まで一定の速度で進行し、その間の座標、距離、地名をトピックとしてPublishします。

## ノード
### gnss_simulator
- 

## トピック
### gnss_fix (sensor_msgs/NavSatFix)

### nearest_location (std_msg/String)

### distance_to_target (std_msgs/Float32)

## Geopyのインストール
```bash
sudo apt install python3-geopy
```

## 実行
```bash
ros2 run virtual_travel gnss_simulator
```

## 必要なソフトウェア
- **python: 3.12.3**
- **GeoPy**

## 動作環境
- **Ubuntu 24.04.3 LTS**
- **ROS2 Humble**

## ライセンス
- © 2025 Hiroto Fujitake
- このパッケージはMIT licenseに基づいて公開されています.
- ライセンスの全文は[LICENSE](./LICENSE)から確認できます.

## 参考資料
- https://qiita.com/aquahika/items/6e3753d4498a190bddb1
- https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html
- https://geopy.readthedocs.io/en/stable/#module-geopy.distance
