# Understanding Manager and rtcd.md

## 1. Manager

### 1.1 Master Manager and Slave Manager

[公式ドキュメント](https://openrtm.org/openrtm/ja/doc/developersguide/advanced_rt_system_programming/mastermanager_slavemanager)

マネージャーはRTCを管理する仕組みで、1つのプロセスに1つのマネージャーが起動する.

マネージャーには、マスターマネージャーとスレーブマネージャーの2種類がある. マスターマネージャーはポート番号と対応付けられており,ユーザーが外部からAPIを呼び出すことでRTCの生成や削除が行える. スレーブマネージャーはマスターマネージャーに登録することで、マスターマネージャーのAPIを通じて外部からユーザーがRTCの生成や削除が行える. デフォルトではスレーブマネージャーが起動する.

### 1.2 Learn Master Manager

ネームサーバーを立ち上げる.
```bash
$ rm -fr /tmp/omninames-* ## remove old log files
$ omniNames -start 15005 -logdir /tmp
```

別のターミナルを開き次を実行する.
```bash
$ rosrun openrtm_aist SeqOutComp -o "corba.nameservers:localhost:15005" -o "naming.formats:%n.rtc"
```
マネージャー(デフォルトのスレーブマネージャー)と、実行コンテキスト(デフォルトの`PeriodicExecutionContext`)と、RTコンポーネント(`SequenceOutComponent0.rtc`)が同時に生成される.

別のターミナルで`rtls localhost:15005/`を実行すると、以下の結果を得る
```
SequenceOutComponent0.rtc
```

マネージャーを生成するときに,`manager.is_master:YES`を設定すると,マスターマネージャーになる.`corba.master_manager:localhost:2810`でポート番号を指定する. 別のターミナルで次を実行する.
```bash
$ rosrun openrtm_aist SeqInComp -o "corba.nameservers:localhost:15005" -o "naming.formats:%n.rtc" -o "manager.is_master:YES" -o "corba.master_manager:localhost:2810" -o "manager.naming_formats:%n.mgr"
```
`manager.naming_formats:%n.mgr`ではマネージャーをネームサーバーに登録する際のフォーマットを指定している. マネージャー(マスターマネージャー)と、実行コンテキスト(デフォルトの`PeriodicExecutionContext`)と、RTコンポーネント(`SequenceInComponent0.rtc`)が同時に生成される.

別のターミナルで`rtls localhost:15005/`を実行すると、以下の結果を得る
```
SequenceOutComponent0.rtc  SequenceInComponent0.rtc  manager.mgr/
```
マスターマネージャーは`rtls`ではディレクトリ(`manager.mgr`)として表示される.

別のターミナルで`rtls localhost:15005/manager.mgr/`を実行すると、このマスターマネージャーに登録されているRTコンポーネントの一覧を得る
```
SequenceInComponent0.rtc
```

スレーブマネージャーを生成するときに`corba.master_manager:localhost:2810`を設定すると、指定したポート番号のマスターマネージャーにそのスレーブマネージャーを登録できる. (デフォルトは`localhost:2810`). 別のターミナルで次を実行する.
```bash
rosrun openrtm_aist MyServiceProviderComp -o "corba.nameservers:localhost:15005" -o "naming.formats:%n.rtc" -o "corba.master_manager:localhost:2810"
```
マネージャー(デフォルトのスレーブマネージャー)と、実行コンテキスト(デフォルトの`PeriodicExecutionContext`)と、RTコンポーネント(`MyServiceProvider0.rtc`)が同時に生成される.

別のターミナルで`rtls localhost:15005/manager.mgr/`を実行すると、マスターマネージャーに`MyServiceProvider0.rtc`が登録されていることが分かる
```
MyServiceProvider0.rtc  manager/  SequenceInComponent0.rtc
```

## 2. rtcd

`rtcd`は, RTコンポーネントや実行コンテキストを起動することなく, マネージャーのみを起動するプログラムである.

ネームサーバーを立ち上げる.
```bash
$ rm -fr /tmp/omninames-* ## remove old log files
$ omniNames -start 15005 -logdir /tmp
```

別のターミナルで次を実行する.
```bash
$ rosrun openrtm_aist rtcd -o "corba.nameservers:localhost:15005" -o "naming.formats:%n.rtc" -o "manager.is_master:YES" -o "corba.master_manager:localhost:2810" -o "manager.naming_formats:%n.mgr"
```

別のターミナルで`rtls localhost:15005/`を実行すると、マスターマネージャーのみが起動していることがわかる
```
manager.mgr/
```

## 3. Create Component on Manager

### 3.1 Python Interface

マネージャーを起動した後で、後からPythonのインターフェースから、マネージャーにRTコンポーネントを追加できる. マネージャーのプロセスに動的に`.so`ファイルをロードし、同一プロセス内にRTコンポーネント(と実行コンテキスト)が追加される.

ネームサーバーを立ち上げる.
```bash
$ rm -fr /tmp/omninames-* ## remove old log files
$ omniNames -start 15005 -logdir /tmp
```

別のターミナルで次を実行し、マスターマネージャーを立ち上げる.
```bash
$ rosrun openrtm_aist rtcd -o "corba.nameservers:localhost:15005" -o "naming.formats:%n.rtc" -o "manager.is_master:YES" -o "corba.master_manager:localhost:2810" -o "manager.naming_formats:%n.mgr"
```

このport 2810にあるマスターマネージャーに、[Writing Simple Publisher Subscriber RTC](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/Writing_Simple_Publisher_Subscriber_RTC.md)で作成した[sample_io_rtc](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_io_rtc)の`Publisher.so`をロードし,`Publisher`RTコンポーネント(と実行コンテキスト)を生成させる.

別のターミナルで以下のPythonスクリプトを実行する。
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

rtc = mgr.create_component("Publisher")
```

別のターミナルで`rtls localhost:15005/`を実行すると、`Publisher0.rtc`(と実行コンテキスト)が生成されていることが分かる
```
Publisher0.rtc  manager.mgr/
```

`mgr.create_component("Publisher")`を再度実行して、別のターミナルで`rtls localhost:15005/`を実行すると、`Publisher1.rtc`(と実行コンテキスト)がもう一つ生成されていることが分かる.
```
Publisher0.rtc Publisher1.rtc manager.mgr/
```

これらのコンポーネントは別スレッドでパラレルに実行される. これらのコンポーネント間のport通信は, マスターマネージャーの同一プロセス間で行われるため通常よりも高速であると期待される.

### 3.2 create_component()

`create_component`関数の引数のオプションについて述べる.

`mgr.create_component("<component name>?option1&option2&option3")`

のように、&でつなぐことで複数のオプションを与えることが可能.

- `instance_name=<instance name>`: 生成したRTコンポーネントをネームサーバーに登録するときのインスタンス名を指定する
- `exec_cxt.periodic.rate=500`: 生成したRTコンポーネントの実行周期を指定する
- `exec_cxt.periodic.type=PeriodicExecutionContext`: RTコンポーネントと同時に生成される実行コンテキストのタイプ指定する

### 3.3 related options

[公式ドキュメント](https://openrtm.org/openrtm/ja/doc/developersguide/basic_rtc_programming/rtc_conf_reference)
- `manager.modules.load_path:<path1>,<path2>,<path3>`: モジュールロード時にマネージャーは指定されたサーチパスリストからモジュールを探索する.
- `manager.modules.preload:<module1.so>,<module2.so>,<module3.so>`: マネージャー起動時に自動的に指定されたモジュールをロードする
- `manager.components.precreate:<component name1>,<component name2>,<component name3>`: マネージャー起動時に指定されたコンポーネントを自動的に作成する