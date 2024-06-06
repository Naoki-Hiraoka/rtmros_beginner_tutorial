# COLLADA to URDF

## 1. Example

ここでは、[VRML to Collada](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/model_conversion_beginner_tutorial/VRML_to_Collada.md)で生成した`/tmp/SampleRobot.dae`を変換する例で説明する.

```bash
rosrun collada_urdf_jsk_patch collada_to_urdf /tmp/SampleRobot.dae -O /tmp/SampleRobot.urdf
```

`openhrp-export-collada`は、openHRP3をインストールすると使用可能になるスクリプトである.

- Options
  - 第一引数: 変換元のCOLLADAファイルのパスを指定する
  - '-O ファイルパス': 変換先のURDFファイルのパスを指定する
  - `--mesh_output_dir ディレクトリパス`: 生成されたurdfファイルのメッシュを生成するディレクトリを指定する.
  - `--mesh_prefix ディレクトリパス`: デフォルトだと`<mesh filename="file:///tmp/WAIST_LINK0_mesh.dae" scale="1 1 1" />`とメッシュのパスが絶対パスで記述される. 全ての`file:///tmp`の部分を、この値で置換する. `package://プロジェクト名/models/`などとして`package://`を使用すると便利.
