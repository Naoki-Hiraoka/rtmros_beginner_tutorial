# Understanding RT Services

[公式ドキュメント](https://www.openrtm.org/openrtm/ja/doc/developersguide/basic_rtc_programming/servcieport)

## 1. Setup

### 1.1 ネームサーバー
```bash
rm -fr /tmp/omninames-* ## remove old log files
omniNames -start 15005 -logdir /tmp
```
### 1.2 RTコンポーネント(Provider)

新しいターミナルで以下を実行
```bash
rosrun openrtm_aist MyServiceProviderComp -o "corba.nameservers:localhost:15005" -o "naming.formats:%n.rtc"
```
別のターミナルで以下を実行
```bash
rtact localhost:15005/MyServiceProvider0.rtc
```
### 1.3 RTコンポーネント(Consumer)
新しいターミナルで以下を実行
```bash
rosrun openrtm_aist MyServiceConsumerComp -o "corba.nameservers:localhost:15005" -o "naming.formats:%n.rtc"
```
別のターミナルで以下を実行
```bash
rtact localhost:15005/MyServiceConsumer0.rtc
```

## 2. Service ports

### 2.1 Service ports

ServiceはROSのサービスと同様の役割を果たす.

`rtcat localhost:15005/MyServiceProvider0.rtc -l`を実行すると、次のように表示される(一部抜粋)
```
-CorbaPort: MyService
  port.port_type  CorbaPort
  Interface:
    Instance name  myservice0
    Type name      MyService
    Polarity       Provided
```
`MyServiceProvider0.rtc`には`MyService`というポート名のServicePortがあることが分かり、`Polarity Provided`と表示されていることからサーバー側であることが分かる. サービスの型が`MyService`でインスタンス名が`myservice0`であり、これらはサーバー側とクライアント側で一致している必要がある。

`rtcat localhost:15005/MyServiceConsumer0.rtc -l`を実行すると次のように表示される(一部抜粋)
```
-CorbaPort: MyService
  port.port_type  CorbaPort
  Interface:
    Instance name  myservice0
    Type name      MyService
    Polarity       Required
```
`MyServiceConsumer0.rtc`には`MyService`というポート名のServicePortがあることが分かり、`Polarity Required`と表示されていることからクライアント側であることが分かる.サービスの型が`MyService`でインスタンス名が`myservice0`であり、これらはサーバー側とクライアント側で一致している必要がある。

## 2.2 rtcon

ROSでは、同じトピック名のサーバーとクライアントを起動すると勝手に通信が始まる. ところがOpenRTMでは、ServicePortを用意するだけでは通信が始まらない. また、ポート名は同じであろうとなかろうと関係ない.

OpenRTMでは、通信させたいPortどうしをポート名を指定して明示的に接続する必要がある. 新しいターミナルを開き以下を実行する.
```
rtcon localhost:15005/MyServiceProvider0.rtc:MyService localhost:15005/MyServiceConsumer0.rtc:MyService
```

`rtcat localhost:15005/MyServiceProvider0.rtc -l`や`rtcat localhost:15005/MyServiceConsumer0.rtc -l`を実行すると、ポート同士が接続されていることが分かる.

`rtcon`コマンドの詳細は[公式ドキュメント](https://www.openrtm.org/openrtm/ja/doc/toolmanuals/rtshell/command_reference/rtcon)参照

## 2.3 Service Call

すると、`MyServiceConsumer0.rtc`を走らせているターミナルに表示されているインタプリタに、
```
echo Hello
```
```
set_val 1
```
```
get_val
```
などと打ち込むと、`MyServiceProvider0.rtc`側で対応するサービスコールが呼ばれているのが分かる.
