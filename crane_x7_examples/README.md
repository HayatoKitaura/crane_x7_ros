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

### swing_object.pyの実行

物体（今回はペンライト）を振るコード例です。

この動作は実機、Gazebo上で動作可能です。

モーションは[set_angle.py](https://github.com/ryuichiueda/crane_x7_ros/blob/master/crane_x7_examples/scripts/)で各部位の角度を設計し、csvファイルに記入します。

次のコマンドで物体を掴み振るモーションを再生できます。

・crane_examples/scriptsへ移動
```
cd catkin_ws/src/crane_x7_ros/crane_x7_examples/scripts
```
・ファイルだけ持ってきたい場合
```
#ファイルをダウンロード
wget https://raw.githubusercontent.com/HayatoKitaura/crane_x7_ros/master/crane_x7_examples/scripts/swing_object.py

wget https://raw.githubusercontent.com/HayatoKitaura/crane_x7_ros/master/crane_x7_examples/scripts/swing_object.csv

#ファイルに実行権限を与える
chmod 764 swing_object.py
```

・実行
```
rosrun crane_x7_examples swing_object.py
```

・説明

csvファイルから各部位の角度を読み込み、順に実行します。

csvファイルの記入方法はset_angle.pyの記入方法と同じです。

![swing_object](https://github.com/HayatoKitaura/crane_x7_ros/blob/master/crane_x7_examples/demo.gif)


