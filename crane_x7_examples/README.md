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

今回はデモとしてペンライトを使用しました。

この動作は実機、Gazebo上で動作不可能です（RealSense関係以外は可能）

モーションは[set_angle.py](https://github.com/ryuichiueda/crane_x7_ros/blob/master/crane_x7_examples/scripts/)で各部位の角度を設計し、関節の位置と回転角度を指定します。


[スライド・期末発表](https://docs.google.com/presentation/d/1nHuU0X9NXfnAbDoUjV0XEDaFVCdB3_XfKulnwhmLkig/edit?usp=sharing)


[スライド・結果報告](https://onedrive.live.com/view.aspx?resid=814F23BD7044D0DC!473&ithint=file%2cpptx&authkey=!ALmIcA6SCFlU8P0)

[デモ動画](https://www.youtube.com/watch?v=0n9izmeAt4Q)

[![](https://img.youtube.com/vi/0n9izmeAt4Q/0.jpg)](https://www.youtube.com/watch?v=0n9izmeAt4Q)

#### 環境構築
+ crane_x7_rosをインストール
```
git clone https://github.com/HayatoKitaura/crane_x7_ros.git
```
+ realsense-rosをインストール
```
git clone https://github.com/IntelRealSense/realsense-ros.git
 ```
+ darknet_rosをインストール
```
git clone https://github.com/HayatoKitaura/darknet_ros.git
```
+ ペンライトを学習した重みをダウンロード
```oo
https://drive.google.com/file/d/1wX5fMArZlW0T0s1JZhx2QuuGTuoWYt5P/view?usp=sharing
```
+ google_assistant_rosをインストール
```
git clone https://github.com/HayatoKitaura/google_assistant.git
```
GoogleAssistantAPIの認証、GoogleAssistantSDKのインストールなどの初期設定が多いのでこのサイトを参考にさせていただきました。

[Google Assistant をROSで使ってみる](https://qiita.com/Nenetti/items/a4c3cffd8008f328855f)


+ catkin_wsに移動してmake
```
catkin_make
```

#### 実行
+ RealSenseを起動
```
roslaunch realsense2_camera rs_camera.launch
```
+ Yoloを起動
```
roslaunch darknet_ros detect_penlight.launch
```
+ GoogleAssistantを有効にする
```
rosrun google_assistant recognition.py
#別のタブを開く
rosrun google_assistant publish.py
```

+ google_assistant_robot.launchを起動
```
roslaunch crane_x7_examples google_assistant_robot.launch
```
+ 説明
    - realsense-rosはRealSenseD435からの画像をpublishします。

    - darknet_rosはrealsense-rosからの画像をsubscribeし、物体を認識、その結果をpublishします。

    - google_assistant/recognition.pyで「ペンライトを振って」など音声を認識すると動作開始。

    - crane_x7_examples/bbox_pos_server.pyで認識結果から物体の中心座標を計算し、クライアントからリクエストをもらうとレスポンスとしてその座標を返します。

    - crane_x7_examples/detect_object_swing.pyでCrane-X7の制御を行っています。上記のクライアントはこのファイル内に書かれています。
