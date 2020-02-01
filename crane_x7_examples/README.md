# crane_x7_examples

CRANE-X7のためのパッケージ、 `crane_x7` で用いるサンプルをまとめたパッケージです。

## システムの起動方法



CRANE-X7の制御信号ケーブルを制御用パソコンへ接続します。
Terminalを開き、`crane_x7_bringup`の`demo.launch`を起動します。
このlaunchファイルには次のオプションが用意されています。

- fake_execution (default: true)

実機を使用する/使用しない

### シミュレータを使う場合


実機無しで動作を確認する場合、
制御信号ケーブルを接続しない状態で次のコマンドを実行します。

```sh
roslaunch crane_x7_bringup demo.launch fake_execution:=true
```

### 実機を使う場合


実機で動作を確認する場合、
制御信号ケーブルを接続した状態で次のコマンドを実行します。

```sh
roslaunch crane_x7_bringup demo.launch fake_execution:=false
```

ケーブルの接続ポート名はデフォルトで`/dev/ttyUSB0`です。
別のポート名(例: /dev/ttyUSB1)を使う場合は次のコマンドを実行します。

```sh
roslaunch crane_x7_bringup demo.launch fake_execution:=false port:=/dev/ttyUSB1
```

### Gazeboを使う場合

次のコマンドで起動します。実機との接続やcrane_x7_bringupの実行は必要ありません。

```sh
roslaunch crane_x7_gazebo crane_x7_with_table.launch
```


## サンプルの実行方法



`demo.launch`を実行している状態で各サンプルを実行することができます。

### google_assistant_robot.pyの実行

GoogleAssistantを使用し音声認識、RealSenseD435を使用し物体認識、物体を掴み振るコードです。

この動作は実機、Gazebo上で動作不可能です（RealSense関係以外は可能）

モーションは[set_angle.py](https://github.com/ryuichiueda/crane_x7_ros/blob/master/crane_x7_examples/scripts/)で各部位の角度を設計し、関節の位置と回転角度を指定します。

#### 実行コマンド

・RealSenseD435を起動
```
roslaunch realsense2_camera rs_camera.launch
 ```
・darknet_rosを起動
```
roslaunch darknet_ros detect_penlight.launch
```
・CraneX7を起動
```
roslaunch crane_x7_examples google_assistant_robot.launch
```
・

・実行
```
rosrun crane_x7_examples swing_object.py
```

・説明

csvファイルから各部位の角度を読み込み、順に実行します。

csvファイルの記入方法はset_angle.pyの記入方法と同じです。

![swing_object](https://github.com/HayatoKitaura/crane_x7_ros/blob/master/crane_x7_examples/demo.gif)


