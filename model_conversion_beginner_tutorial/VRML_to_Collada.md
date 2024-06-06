# VRML to COLLADA

## 1. Example

ここでは、`/opt/ros/melodic/share/openhrp3/share/OpenHRP-3.1/sample/model/sample1.wrl`にあるSampleRobotモデルを変換する例で説明する. ROSのDISTROが違う場合にはパスを修正すること.

```bash
openhrp-export-collada -i /opt/ros/melodic/share/openhrp3/share/OpenHRP-3.1/sample/model/sample1.wrl -o /tmp/SampleRobot.dae
```

`openhrp-export-collada`は、openHRP3をインストールすると使用可能になるスクリプトである.

- Options
  - `-i`: 変換元のVRMLファイルのパスを指定する
  - `-o`: 変換先のCOLLDAファイルのパスを指定する