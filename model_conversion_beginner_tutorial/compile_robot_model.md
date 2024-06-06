# compile robot model

- INPUT
  - VRMLモデル
  - 設定ファイル(yaml) ([Collada to Eus](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/model_conversion_beginner_tutorial/Collada_to_Eus.md)参照)
- OUTPUT
  - Colladaモデル
  - URDFモデル
  - eusモデル
  - ros_control用controller_config.yaml
  - hrpsys用設定ファイル(.conf)
  - その他もろもろ

## example
サンプルが[sample_model_conversion](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/model_conversion_beginner_tutorial/sample_model_conversion)にある.
ここではopenhrp3のSampleRobotモデルを変換する例で説明する

```
catkin build sample_model_conversion
```
modelsディレクトリ以下に、変換結果が生成される.