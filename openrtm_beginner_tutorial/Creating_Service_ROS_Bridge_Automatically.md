# Creating Service ROS Bridge Automatically

## 1. サービスブリッジの自動生成

`rtmbuild`には、`idl`で定義されたインタフェースを読んで、ROSのサービス型を自動生成し、さらに、

ROSのサービスサーバーとして機能し、ROSのクライアントからサービスコールが呼ばれると内部でOpenRTMのサービスサーバーにOpenRTMのサービスコールをして、結果をROSのクライアントに返すノードのプログラムを自動で生成する機能がある.

Service Client(ROS node) `--[ROS Service Call]->` ServiceBridge(Automatically Generated) `--[OpenRTM Service Call]->` Service Server(RTC)

サンプルコードは[sample_service_bridge](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_service_bridge)にある.

## 2. Usage

CMakeLists.txtに, 以下のように書く.
```
find_package(catkin REQUIRED COMPONENTS rtmbuild roscpp)

# initialize rtmbuild. Call BEFORE catkin_package
rtmbuild_init()

catkin_package()

# generate idl
rtmbuild_genidl()

# generate bridge
rtmbuild_genbridge()
```
最後の`rtmbuild_genbridge`は、これまでのチュートリアルにはなかったコマンドである. このコマンドによって、`idl`ディレクトリ以下にあるファイルで定義されているインタフェースごとに、サービスのROSブリッジが自動生成される.

## 3. ROS Bridge

- executable name: `<インタフェース名>ROSBridgeComp`
- RTC instance_name: `<ROSのノード名と同じ>`

### Services(ROS)
- `~<関数名>` (`<パッケージ名>/<モジュール名>_<インターフェース名>_<関数名>`型)
  このインターフェースが持つ関数それぞれに対応したサービスが提供される.

### Service Ports(OpenRTM)
- `<インタフェース名>`
  サービスクライアント. ROSからサービスが呼ばれると、このportにつながったOpenRTMのサービスサーバーにサービスコールをする. idlに書かれたinterface名がそのままポート名になる。インスタンス名はデフォルトでは"service0"で、rosparam(`~service_port`)で指定できる.

## 4. Run Sample

[sample_service_bridge](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_service_bridge)にサンプルがある.

このサンプルでは、3つのことをしている.
1. idlの定義 (idl/MyBridgeService.idl)
2. ROSBridgeの自動生成 (CMakeLists.txtにて)
3. サーバーRTCの作成 (rtc/MyServer)

このサンプルではidlファイルに次のようなインターフェースを定義している.
```
#include "BasicDataType.idl"

module sample_service_bridge
{
  interface MyBridgeService
  {
    boolean addTwoInts(in long a, in long b, out long sum);
  };
};
```

CMakeLists.txtに`rtmbuild_genbridge()`を記述することで、このインターフェースに対応したROSBridgeを自動生成している。

また、rtcディレクトリ以下に、[Writing Simple Service Server Client RTC](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/Writing_Simple_Service_Server_Client_RTC.md)と同様にしてこのインターフェースに対応したサーバーのRTコンポーネントを作成している. ROSBridgeを経由して、ROSのレイヤからサービスコールをして、このRTコンポーネントの`addTwoInts`を呼ぶ. 

ビルド
```bash
catkin build sample_service_bridge
```

起動
```
rtmlaunch sample_service_bridge server.launch
```

ROSからサービスコールを送る
```
rosservice call /MyBridgeServiceROSBridge/addTwoInts 1 2
```
