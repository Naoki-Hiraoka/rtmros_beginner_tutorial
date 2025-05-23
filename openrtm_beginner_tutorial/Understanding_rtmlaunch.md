# Understanding rtmlaunch
サンプルコードは[sample_rtmlaunch](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_rtmlaunch)にある

## 1. Using rtmlaunch

`rtmlaunch`は`roslaunch`の代わりに使用することができるコマンドである.

Usage:
```bash
rtmlaunch [package] [filename.launch]
```

`rtmlaunch`は`roslaunch`と同じ機能をそのまま持っている.

`roslaunch`では実行時に`roscore`が存在しなければ自動で`roscore`を立ち上げるが、`rtmlaunch`ではそれに加えて、実行時にネームサーバーが存在しなければ自動でネームサーバーを立ち上げる.

デフォルトではポート15005にネームサーバーを立ち上げるが、以下のように引数を与えることでポート番号を指定可能.
```bash
rtmlaunch [package] [filename.launch] corbaport:=2809
```

## 2. Using rtmlaunch.py

### 2.1 rtmlaunch.py

`rtmlaunch.py`は、`launch`ファイルを読んで、ポート間の接続(`rtcon`)及びRTコンポーネントの実行(`rtact`)を自動的に行ってくれる.

Usage:
```bash
rosrun openrtm_tools rtmlaunch.py [fullfilepath.launch]
```

デフォルトではポート15005のネームサーバーに接続するが、環境変数`RTCTREE_NAMESERVERS`をセットすることで変更可能.
```bash
RTCTREE_NAMESERVERS=localhost:2809
```

### 2.2 The Launch File

`rtmlaunch.py`を利用することで、[Understanding RT In/Out Ports](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/Understanding_RT_In_Out_Ports.md)で起動したのと同じものが以下のlaunchファイルによって起動できる.

```xml
<launch>
  <arg name="openrtm_args" value='-o "corba.nameservers:localhost:15005" -o "naming.formats:%n.rtc" ' />

  <!-- RTC SeqIn/SeqOut sample -->
  <node name="seqin"  pkg="openrtm_aist" type="SeqInComp" args='$(arg openrtm_args)' output="screen"/>
  <node name="seqout" pkg="openrtm_aist" type="SeqOutComp" args='$(arg openrtm_args)' output="screen"/>

  <!-- BEGIN:openrtm connection -->
  <node name="rtmlaunch_py" pkg="openrtm_tools" type="rtmlaunch.py"
        args="$(find sample_rtmlaunch)/launch/sample_rtmlaunch.launch" >
  </node>
  <rtactivate component="SequenceInComponent0.rtc" />
  <rtactivate component="SequenceOutComponent0.rtc" />
  <rtconnect from="SequenceInComponent0.rtc:Long"     to="SequenceOutComponent0.rtc:Long" />
  <!-- END:openrtm connection -->
</launch>
```

同じファイルが[sample_rtmlaunch.launch](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_rtmlaunch/launch/sample_rtmlaunch.launch)にある.

### 2.3 The Launch File Explained

```xml
<node name="rtmlaunch_py" pkg="openrtm_tools" type="rtmlaunch.py"
      args="$(find sample_rtmlaunch)/launch/sample_rtmlaunch.launch" >
</node>
```
`rtmlaunch.py`にこのlaunchファイルを読み込ませている. `rtmlaunch.py`は、このlaunchファイル中の`rtactivete`タグと`rtconnect`タグを読んで処理を行う.

```xml
<rtactivate component="SequenceInComponent0.rtc" />
<rtactivate component="SequenceOutComponent0.rtc" />
```
`rtmlaunch.py`は、`rtactivate`タグを読み込むと、引数で与えられたRTコンポーネントを実行中にする.

```xml
<rtconnect from="SequenceInComponent0.rtc:Long"     to="SequenceOutComponent0.rtc:Long" />
```
`rtmlaunch.py`は、`rtconnect`タグを読み込むと、引数で与えられたポート同士を接続する.

### 2.4 rtactivate tag

`rtmlaunch.py`は、`rtactivate`タグを読み込むと、引数で与えられたRTコンポーネントを実行中にする.
- Attributes
  - component="instance_name"
    - 実行中にするコンポーネント名

### 2.5 rtconnect tag

`rtmlaunch.py`は、`rtconnect`タグを読み込むと、引数で与えられたポート同士を接続する.
- Attributes
  - from="port_name"
    - 接続するポート名
  - to="port_name"
    - 接続するポート名
  - subscription_type="flush|new|periodic" (default "flush")
  - push_policy="all|fifo|skip|new" (default "all")
  - push_rate="double" (default 50.0)
  - buffer_length="int" (default "8")

各引数の意味は[公式ドキュメント](https://www.openrtm.org/openrtm/ja/doc/developersguide/basic_rtc_programming/dataport)参照.

#### 2.5.1 rtconnectの各引数の詳細

subscription_type
| 値 | 意味 |
| ---- | ---- |
| new | 送信側でwrite()関数が呼ばれると、送信バッファにデータが書き込まれる. 送信バッファのデータは別スレッドでできるだけ早くに受信側の受信バッファに送られる. ROSのpublisherと似ている. |
| flush | 送信側でwrite()関数から戻った時にはデータが受信側の受信バッファに届いていることが保証される. 複数のRTCを直列に実行する場合には、次のRTCの処理が始まる前に次のRTCへのデータ送信が完了している必要があるので適する. 異なるPC間で通信する場合などで通信に時間がかかる場合には、write()関数で長い時間待たされることになるので適さない. |
| periodic | 送信側でwrite()関数が呼ばれると、送信バッファにデータが書き込まれる. 送信バッファのデータは、別スレッドで周期的(push_rate)に受信側の受信バッファに送られる. 通信の周期を落として通信量を減らしたい場合に用いる. ROSのtopic_tools/throttleと似ている. |

push_policy
| 値 | 意味 |
| ---- | ---- |
| all | 送信バッファに残っているデータをすべて送信 |
| new | 送信バッファの最新値のみ送信し、古い値は捨てる |
| fifo | 先入れ先だし方式で、送信バッファのデータを一つずつ送信 |
| skip | n 個おきに送信バッファのデータを送信し、それ以外は捨てる(どうやってnを設定するんだろう?) |

buffer_length
| 意味 |
| ---- |
| 送信バッファと受信バッファのサイズを設定する.受信側でread()を呼ぶと受信バッファから一番古い値が取り出される.常に最新のデータのみを利用したい場合は1に設定すること.ROSのサブスクライバのqueue_sizeと似ている. |

高周期低遅延で直列実行する制御モジュール間の通信は、`subscription_type="flush" push_policy="new" buffer_length="1"`などとする.

普通の制御モジュール間の通信は、`subscription_type="new" push_policy="new" buffer_length="1"`などとする.

ネットワーク越しに通信する場合、`subscription_type="periodic" push_policy="new" push_rate="50.0", buffer_length="8"`などとする.

### 2.6 Tips for rtmlaunch.py

#### 2.6.1 check connection/activation

`rtmlaunch.py`は、10秒おきに`<rtactivate>`タグに設定されたコンポーネントの実行状態をチェックし、実行中で無ければ実行中にする. 同様に、10秒おきに`<rtconnect>`タグに設定されたポートの接続状態をチェックし、接続されて無ければ接続にする.

環境変数`RTC_CONNECTION_CHECK_ONCE`を`true`セットすると, 初回以降はこのチェックを行わない. 膨大な数のポートのチェックを行うとコンポーネントの実行速度が遅くなるため、リアルタイムタスクでは無効化することを推奨する.
```
RTC_CONNECTION_CHECK_ONCE=true
```

#### 2.6.2 Substitution args

`rtmlaunch.py`はrtconnectタグやrtactivateタグをパースするに際して、roslaunchのsubstitution argsの機能のうちの一部しか利用できず、それらも通常とは異なる形で解決される.

- `$(arg foo)`を解決するためには、環境変数`foo`をその値にセットする必要がある.
- `if="$(arg foo)"` `unless="$(arg foo)"`は、rtconnectタグ・rtactivateタグ・深さ1のgroupタグのものしか解釈されず、`rtmlaunch.py`の引数に`foo=true`または`foo=false`を与える必要がある.
- それ以外のsubstitution argsは利用不可

## 3 rtmlaunching

以下のコマンドで実行できる.
```bash
catkin build sample_rtmlaunch
rtmlaunch sample_rtmlaunch sample_rtmlaunch.launch
```

2つのRTコンポーネントの出力が重なっているため見づらいが、以下のような出力が表示される.
```
---------------------------------------------------------------------
---------------------------------------------------------------------
---------------------------------------------------------------------
           Double        Float         Long        Short        Octet
---------------------------------------------------------------------
0: 281.345 281.12 281 281 27[ ]                             ---------[ ]
                             Basic type                             -
---------------------------------------------------------------------[ ]
1: 282.345 282.12 282 282 28[ ]                              --------
                0            0           28            0            0[ ]
-: 42.345 42.12 42 42 42[*]                                ----------[ ]
2: 283.345 283.12 283 283 29[ ]                                      
0: 411.345 411.12 411 411 157[ ]                                     
1: 412.345 412.12 412 412 158[ ]                             --------
---------------------------------------------------------------------
---------------------------------------------------------------------
           Double        Float         Long        Short        Octet
---------------------------------------------------------------------
---------------------------------------------------------------------
           Double        Float         Long        Short        Octet[ ]
---------------------------------------------------------------------[ ]
                             Basic type                             -
---------------------------------------------------------------------
                0            0           42            0            0[ ]
---------------------------------------------------------------------
                            Sequence type                            
---------------------------------------------------------------------
 0:             -            -            -            -            -
```
