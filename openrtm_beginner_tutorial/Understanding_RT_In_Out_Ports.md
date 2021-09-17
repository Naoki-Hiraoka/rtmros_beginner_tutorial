# Understanding RT In/Out Ports

[公式ドキュメント](https://www.openrtm.org/openrtm/ja/doc/developersguide/basic_rtc_programming/dataport)

## 1. Setup

### 1.1 ネームサーバー
```bash
rm -fr /tmp/omninames-* ## remove old log files
omniNames -start 15005 -logdir /tmp
```
### 1.2 RTコンポーネント(Out)

新しいターミナルで以下を実行
```bash
rosrun openrtm_aist SeqOutComp -o "corba.nameservers:localhost:15005" -o "naming.formats:%n.rtc"
```
別のターミナルで以下を実行
```bash
rtact localhost:15005/SequenceOutComponent0.rtc
```
### 1.3 RTコンポーネント(In)
新しいターミナルで以下を実行
```bash
rosrun openrtm_aist SeqInComp -o "corba.nameservers:localhost:15005" -o "naming.formats:%n.rtc"
```
別のターミナルで以下を実行
```bash
rtact localhost:15005/SequenceInComponent0.rtc
```

## 2. In/Out ports

### 2.1 In/Out ports

Inportは、ROSのSubscriberと同様の役割を果たす. Outportは、ROSのPublisherと同様の役割を果たす.

`rtcat localhost:15005/SequenceOutComponent0.rtc`を実行すると`SequenceOutComponent0.rtc`には複数のOutPortがあることが分かる.

`rtcat localhost:15005/SequenceInComponent0.rtc`を実行すると`SequenceInComponent0.rtc`には複数のInPortがあることが分かる.

### 2.1 rtprint
新しいターミナルを開き以下を実行する.
```bash
rtprint localhost:15005/SequenceOutComponent0.rtc:Long -t 10
```
すると例えば以下のような出力が10秒間続く.
```
comp_args: rtprint_reader0?exec_cxt.periodic.type=PeriodicExecutionContext&exec_cxt.periodic.rate=100.0
[0.000000000] 1670
[0.000000000] 1671
[0.000000000] 1672
[0.000000000] 1673
[0.000000000] 1674
[0.000000000] 1675
```

`rtprint`はROSの`rostopic echo`と同様の役割を果たし、RTコンポーネントのOutPortから出力されているデータを表示する.

この例だと、`SequenceOutComponent0.rtc`という名前のRTコンポーネントの、`Long`という名前のOutPortから出力されているデータを表示する.

`rtprint`コマンドの詳細は[公式ドキュメント](https://www.openrtm.org/openrtm/ja/doc/toolmanuals/rtshell/command_reference/rtprint)参照

## 2.2 rtcon

ROSでは、同じトピック名のPublisherとSubscriberを起動すると勝手に通信が始まる. ところがOpenRTMでは、OutPortとInPortを用意するだけでは通信が始まらない. また、Port名は同じであろうとなかろうと関係ない.

OpenRTMでは、通信させたいOutPortとInPortとを明示的に接続する必要がある. 新しいターミナルを開き以下を実行する.
```
rtcon localhost:15005/SequenceOutComponent0.rtc:Long localhost:15005/SequenceInComponent0.rtc:Long
```

すると、`SequenceInComponent0.rtc`を走らせていたターミナルの出力が変化していることが分かる.

また、`rtcat localhost:15005/SequenceInComponent0.rtc -l`や`rtcat localhost:15005/SequenceOutComponent0.rtc -l`を実行すると、ポート同士が接続されていることが分かる.

`rtcon`コマンドの詳細は[公式ドキュメント](https://www.openrtm.org/openrtm/ja/doc/toolmanuals/rtshell/command_reference/rtcon)参照

## 2.3 Choreonoid GUI (Optional)

ROSの`rqt_graph`のようなビジュアライズ機能や、GUIを用いたポート間の接続が行える.

詳細は[公式ドキュメント](https://choreonoid.org/ja/manuals/1.7/openrtm/openrtm-view.html)参照