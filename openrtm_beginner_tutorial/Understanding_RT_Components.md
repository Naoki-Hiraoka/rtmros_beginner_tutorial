# Understanding RT Components

## 1. ネームサーバー

ネームサーバー(nameserver)は, OpenRTMを使うときに最初に起動させなければならないプロセスである.
```bash
rm -fr /tmp/omninames-* ## remove old log files
omniNames -start 15005 -logdir /tmp
```

ネームサーバーはROSの`roscore`と同様の役割を果たす。

複数のネームサーバーを立ち上げることが可能で、ネームサーバーを区別するためにport番号を指定している.(この例では15005)

ROSでは`roslaunch`を使うと自動で`roscore`が立ち上がるが、同様に後で述べるシェルコマンド`rtmlaunch`を使うと自動でネームサーバーが立ち上がる

## 2. RTコンポーネント

ネームサーバーを走らせたまま新しいターミナルを開き以下を実行する.
```bash
rosrun openrtm_aist SeqOutComp -o "corba.nameservers:localhost:15005" -o "naming.formats:%n.rtc"
```

RTコンポーネントはROSの`node`と同様の役割を果たす.

引数`corba.nameservers:localhost:15005`でネームサーバーのポート番号を指定している.

引数`naming.formats:%n.rtc`でRTCをネームサーバに登録する際の名前のフォーマットを指定している.

利用可能な引数の詳細は[公式ドキュメント1](https://www.openrtm.org/openrtm/ja/doc/developersguide/basic_rtc_programming/rtc_conf_reference), [公式ドキュメント2](https://www.openrtm.org/openrtm/ja/doc/developersguide/basic_rtc_programming/comp_conf_reference)参照


## 3. rtls

ネームサーバーとRTコンポーネントを走らせたまま新しいターミナルを開き以下を実行する.
```bash
rtls localhost:15005/
```
すると以下の情報が表示される
```
SequenceOutComponent0.rtc
```

`rtls`はROSの`rosnode list`と同様の役割を果たし、ネームサーバーに登録されているRTコンポーネントの一覧を表示する.

引数でネームサーバーのポート番号を指定している.

`rtls`コマンドの詳細は[公式ドキュメント](https://www.openrtm.org/openrtm/ja/doc/toolmanuals/rtshell/command_reference/rtls)参照

## 4. rtcat

ネームサーバーとRTコンポーネントを走らせたまま新しいターミナルを開き以下を実行する.
```bash
rtcat localhost:15005/SequenceOutComponent0.rtc
```
すると以下の情報が表示される
```
SequenceOutComponent0.rtc  Inactive
  Category       example
  Description    Sequence OutPort component
  Instance name  SequenceOutComponent0
  Type name      SequenceOutComponent
  Vendor         Noriaki Ando, AIST
  Version        1.0
  Parent         
  Type           Monolithic
 +Extra properties
+Execution Context 0
+DataOutPort: Octet
+DataOutPort: Short
+DataOutPort: Long
+DataOutPort: Float
+DataOutPort: Double
+DataOutPort: OctetSeq
+DataOutPort: ShortSeq
+DataOutPort: LongSeq
+DataOutPort: FloatSeq
+DataOutPort: DoubleSeq
```

`rtls`はROSの`rosnode list`と同様の役割を果たし、RTコンポーネントの情報を表示する.

`rtcat`コマンドの詳細は[公式ドキュメント](https://www.openrtm.org/openrtm/ja/doc/toolmanuals/rtshell/command_reference/rtcat)参照

## 5. rtact

`4. rtcat`で`rtcat localhost:15005/SequenceOutComponent0.rtc`を実行した際に、`Inactive`と表示されていた. この状態では、RTコンポーネントは生成されただけの状態で、実行されていない.

ネームサーバーとRTコンポーネントを走らせたまま新しいターミナルを開き以下を実行する.
```bash
rtact localhost:15005/SequenceOutComponent0.rtc
```

`rtact`はRTコンポーネントを実行中にするコマンドである.

RTコンポーネントを走らせていたターミナルの出力が変化していること、`rtcat localhost:15005/SequenceOutComponent0.rtc`を実行すると`Active`と表示されていることが確認できるだろう.

`rtact`コマンドの詳細は[公式ドキュメント](https://www.openrtm.org/openrtm/ja/doc/toolmanuals/rtshell/command_reference/rtact)参照

RTCの状態遷移の詳細は[公式ドキュメント](https://www.openrtm.org/openrtm/ja/doc/developersguide/basic_rtc_programming/rtcdevelflow)参照

## 5. rtdeact

ネームサーバーとRTコンポーネントを走らせたまま新しいターミナルを開き以下を実行する.
```bash
rtdeact localhost:15005/SequenceOutComponent0.rtc
```

`rtdeact`はRTコンポーネントを停止中にするコマンドである. `rtact`と反対の機能である.

RTコンポーネントを走らせていたターミナルの出力が止まっていること、`rtcat localhost:15005/SequenceOutComponent0.rtc`を実行すると`Inactive`と表示されていることが確認できるだろう.

`rtdeact`コマンドの詳細は[公式ドキュメント](https://www.openrtm.org/openrtm/ja/doc/toolmanuals/rtshell/command_reference/rtdeact)参照

