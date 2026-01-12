# 仮想GNSSを使った経由地移動シミュレーター
![test](https://github.com/hiroto20060205-oss/virtual_travel/actions/workflows/test.yml/badge.svg)

## 概要
本リポジトリの`config`ディレクトリにある[location.csv](https://github.com/hiroto20060205-oss/virtual_travel/blob/dev/config/location.csv)の地名、緯度、経度を読み込んで、現在地と目的地までの距離を算出するパッケージです。

次の地点まで一定の速度で進行し、その間の座標、距離、地名をトピックとしてPublishします。(別ターミナルで`ros2 topic echo /トピック名`といったコマンドを入力する必要があります)

## ノード
### gnss_simulator
- [location.csv](https://github.com/hiroto20060205-oss/virtual_travel/blob/dev/config/location.csv)から経由地リストを読み込む
- `GeoPy` を用いて現在地と次地点の距離を計算する
- 等速直線運動で座標を更新し、現在地・目的地・距離を publish する
- 全ての経由地を通過すると "Goal" を出力して停止する

## トピック
### gnss_fix (sensor_msgs/NavSatFix)
- 現在のシミュレーション上の緯度・経度
- その他のパラメータとして高度や共分散があるが、定義していないので0で出力される

### nearest_location (std_msg/String)
- 次に向かっている経由地の名前
- 全ての経由地を通過した場合は "Goal" となる

### distance_to_target (std_msgs/Float32)
- 次の経由地までの直線距離
- 単位: メートル [m]

## Geopyのインストール
地球が球体であることを考慮して、緯度・ 経度をメートル（距離）に変換するライブラリです
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
