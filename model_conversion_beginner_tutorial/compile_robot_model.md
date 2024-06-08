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

## Tips

cmakeの依存関係の記述が不十分であるため、入力のファイルに変更があった場合にうまく反映できない場合がある.
```
cd YOUR_PACKAGE
git clean -dxf .
catkin clean YOUR_PACKAGE
cakin build YOUR_PACKAGE
```
として、src、build両方を初期化してからビルドし直すと良い
