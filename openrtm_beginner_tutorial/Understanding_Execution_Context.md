# Understanding Execution Context

[公式ドキュメント](https://openrtm.org/openrtm/ja/doc/developersguide/basic_rtc_programming/rtcdevelflow)

## 1. Execution Context

RTコンポーネントが生成されると, 実行コンテキストが生成され, そのRTコンポーネントに関連付けられる. 実行コンテキストは, RTコンポーネントの`onExecute`などのメンバ関数の実行タイミングを管理する.

特に、周期的に実行される関数である`onExecute`を、どの周期で、どのタイミングで実行するかを決める役割が重要である.

## 2. Periodic Rate

[Writing Simple Publisher Subscriber RTC](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/Writing_Simple_Publisher_Subscriber_RTC.md)で作成したPublisherを用いて説明する.

実行時に引数で`-o "exec_cxt.periodic.rate:<周期>"`を与えると, コンポーネントが`exec_cxt.periodic.rate`にセットした値の周期で実行されるようになる.

[publisher_1hz.launch](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_ec/launch/publisher_1hz.launch)では、`-o "exec_cxt.periodic.rate:1"`を与えている. 以下を実行するとサンプルを動かせる.
```
cakin build sample_ec
rtmlaunch sample_ec publisher_1hz.launch
```
1hzおきに出力が出てくるのが分かる.

[publisher_10hz.launch](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_ec/launch/publisher_10hz.launch)では、`-o "exec_cxt.periodic.rate:10"`を与えている. 以下を実行するとサンプルを動かせる.
```
rtmlaunch sample_ec publisher_10hz.launch
```
今度は10hzおきに出力が出てくるのが分かる.

## 3. Periodic Type

実行コンテキストにはいくつかの種類がある.

デフォルトは`PeriodicExecutionContext`である. これは、`exec_cxt.periodic.rate`の周期でコンポーネントを実行する.

その他のタイプは[公式ドキュメント](https://openrtm.org/openrtm/ja/doc/developersguide/basic_rtc_programming/comp_conf_reference)にまとめられている. 例えば, `PeriodicExecutionContext`は実際の時刻に対する周期を指定するが、シミュレーターを用いる場合にはシミュレーター内の時刻に対する周期を指定する必要があるため、専用の実行コンテキストがある. また、実機の制御のためによりリアルタイム性を高めた実行コンテキスト([hrpExecutionContext](https://github.com/fkanehiro/hrpsys-base/tree/master/ec/hrpEC))も存在する.

実行時に引数で`-o "exec_cxt.periodic.type:<タイプ>"`を与えると, 指定したタイプの実行コンテキストが生成される.
