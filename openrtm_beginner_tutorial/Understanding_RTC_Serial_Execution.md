# Understanding RTC Serial Execution

## 1. Serial Execution

実時間制御では、一つの周期の間に, センサ値を読むコンポーネントを実行して、出力のセンサ値を状態推定コンポーネントに渡して推定処理を実行して、その出力の状態推定値を制御器コンポーネントに渡して制御処理を実行して, その出力をアクチュエータに書き込むコンポーネントに渡して書き込み処理を実行する、というように複数のコンポーネントをシリアルに順番に実行したいケースがある.

一つの周期の間に複数のコンポーネントをシリアルに順番に実行する方法を述べる.

## 2. Sample

まず、[Understanding Manager and rtcd](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/Understanding_Manager_and_rtcd.md)と同様にして、シリアル実行したいRTコンポーネント郡を一つのマスターマネージャーのプロセス中に実行する.

ここでは、[Writing Simple Publisher Subscriber RTC](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/Writing_Simple_Publisher_Subscriber_RTC.md)で作成した[sample_io_rtc](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_io_rtc)の`Publisher`と`Subscriber`を実行する.

ターミナル1
```bash
$ rm -fr /tmp/omninames-* ## remove old log files
$ omniNames -start 15005 -logdir /tmp
```

ターミナル2
```bash
$ rosrun openrtm_aist rtcd -o "corba.nameservers:localhost:15005" -o "naming.formats:%n.rtc" -o "manager.is_master:YES" -o "corba.master_manager:localhost:2810" -o "manager.naming_formats:%n.mgr"
```

ターミナル3
```python
$ ipython
import sys
from omniORB import CORBA
import OpenRTM_aist
import RTM

orb = CORBA.ORB_init(sys.argv, CORBA.ORB_ID)
obj = orb.string_to_object("corbaloc:iiop:localhost:2810/manager")
mgr = obj._narrow(RTM.Manager)

import pkgconfig

mgr.load_module(str(pkgconfig.variables("sample_io_rtc")["prefix"])+"/lib/Publisher.so","PublisherInit")
publisher = mgr.create_component("Publisher")

mgr.load_module(str(pkgconfig.variables("sample_io_rtc")["prefix"])+"/lib/Subscriber.so","SubscriberInit")
subscriber = mgr.create_component("Subscriber")
```

ターミナル3のpythonで続けて以下を行うことで、これらのRTコンポーネントをシリアルに実行するようになる.
```python
subscriber.get_owned_contexts()[0].stop()
publisher.get_owned_contexts()[0].add_component(subscriber)
```
1行目で、subscriberの実行コンテキストを無効化している.2行目で、publisherの実行コンテキストが管理する対象にsubscriberを追加している.

実行コンテキストは自身が管理するRTコンポーネントを、追加された順にシリアルに実行する. これによって、同一周期の間に`Publisher`->`Subcriber`の順に実行されるようになる.

なお、subscriberをアクティベートするためには, 初期化時と違う実行コンテキストを使うため
```bash
$ rtact localhost:15005/Publisher0
$ rtact localhost:15005/Subscriber0 -e 1
```
と`-e 1`オプションをつける必要がある.

## 3. connection

シリアルに実行する場合は、コンポーネント間の通信は次のコンポーネントを実行開始するときには完了していることが必要である.そのためport間の通信の設定は、サブスリクション型は`flush`, データ送信ポリシーは`new`がよい.

[公式ドキュメント](https://openrtm.org/openrtm/ja/doc/developersguide/basic_rtc_programming/dataport)参照
