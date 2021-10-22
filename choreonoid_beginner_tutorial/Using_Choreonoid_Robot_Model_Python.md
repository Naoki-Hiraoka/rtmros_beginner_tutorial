# Using Choreonoid Robot Model (Python)

## 1. Choreonoid Robot Model

Pythonのプログラムで, モデルを読み込む方法を説明する.

サンプルコードは[sample_choreonoid_model](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/choreonoid_beginner_tutorial/sample_choreonoid_model)にある

## 2. The Code

ここでは、`/opt/ros/melodic/share/openhrp3/share/OpenHRP-3.1/sample/model/sample1.wrl`にあるSampleRobotモデルを読み込むコードで説明する. ROSのDISTROが違う場合にはパスを修正すること.

[sample.py](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/choreonoid_beginner_tutorial/sample_choreonoid_model/scripts/sample.py)
```python
#!/usr/bin/env python

# export PYTHONPATH=`pkg-config choreonoid --variable plugindir`/python:$PYTHONPATH
from cnoid import Body

bodyLoader = Body.BodyLoader()
robot = bodyLoader.load("/opt/ros/melodic/share/openhrp3/share/OpenHRP-3.1/sample/model/sample1.wrl")
if not robot:
    print("failed to load model")
    exit(1)

for i in range(robot.numJoints()):
    print(robot.joint(i).name())
```

## 3. The Code Explained

```python
# export PYTHONPATH=`pkg-config choreonoid --variable plugindir`/python:$PYTHONPATH
from cnoid import Body
```
ロボットモデルのクラスが定義されたモジュールをインポートしている. 事前に環境変数`PYTHONPATH`にchoreonoidのモジュールへのパスをセットしておく必要がある.

```python
bodyLoader = Body.BodyLoader()
robot = bodyLoader.load("/opt/ros/melodic/share/openhrp3/share/OpenHRP-3.1/sample/model/sample1.wrl")
if not robot:
    print("failed to load model")
    exit(1)
```

ここでモデルをロードしている.

```python
for i in range(robot.numJoints()):
    print(robot.joint(i).name())
```
読み込んだロボットの関節名を表示させている.

## 5. Run Example

上記を作成したものが[sample_choreonoid_model](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/choreonoid_beginner_tutorial/sample_choreonoid_model)にある。

ビルド
```bash
catkin build sample_choreonoid_model
```

実行
```bash
export PYTHONPATH=`pkg-config choreonoid --variable plugindir`/python:$PYTHONPATH
rosrun sample_choreonoid_model sample.py
```

出力(抜粋)
```
RLEG_HIP_R
RLEG_HIP_P
RLEG_HIP_Y
RLEG_KNEE
RLEG_ANKLE_P
RLEG_ANKLE_R
RARM_SHOULDER_P
RARM_SHOULDER_R
RARM_SHOULDER_Y
RARM_ELBOW
RARM_WRIST_Y
RARM_WRIST_P
RARM_WRIST_R
LLEG_HIP_R
LLEG_HIP_P
LLEG_HIP_Y
LLEG_KNEE
LLEG_ANKLE_P
LLEG_ANKLE_R
LARM_SHOULDER_P
LARM_SHOULDER_R
LARM_SHOULDER_Y
LARM_ELBOW
LARM_WRIST_Y
LARM_WRIST_P
LARM_WRIST_R
WAIST_P
WAIST_R
CHEST
```
