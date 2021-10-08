# Creating OpenRTM idl
サンプルコードは[sample_idl](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_idl)にある.

## 1. Introduction to idl

ROSでは、msgファイルに通信で用いるデータの型を定義し、srvファイルにサービスコールの関数の型を定義する。OpenRTMでは、idlファイルに通信で用いるデータの型とサービスコールの関数の型を定義する.

ROSの`std_msgs`のように、既に用意されているidlもある.
- [BasicDataType.idl](https://github.com/OpenRTM/OpenRTM-aist/blob/master/src/lib/rtm/idl/BasicDataType.idl)
- [ExtendedDataTypes.idl](https://github.com/OpenRTM/OpenRTM-aist/blob/master/src/lib/rtm/idl/ExtendedDataTypes.idl)
- [InterfaceDataTypes.idl](https://github.com/OpenRTM/OpenRTM-aist/blob/master/src/lib/rtm/idl/InterfaceDataTypes.idl)

## 2. Using Original Data Types

### 2.1 Creating Data Type

ROSのmsgに相当する、通信で用いるデータの型を定義する.

以下のサンプルでは、`sample_idl::MyData`型を定義している.
```
#include "BasicDataType.idl"

module sample_idl
{
  struct MyData
  {
    RTC::Time tm;
    short shortVariable;
    long longVariable;
    sequence<double> data;
  };
};
```
同じ内容のファイルが[MyDataSample.idl](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_idl/idl/MyDataSample.idl)にある.

データ型は`struct`で宣言し、名前空間は`module`で宣言する. 名前空間はパッケージ名と同一にしておくとpythonからのロード時などで後々楽である. 一つのファイルで複数のデータ型を定義してもよい.

変数として利用可能なデータ型は、以下の通り
- 基本型 ([公式ドキュメント](https://www.openrtm.org/openrtm/ja/doc/developersguide/dataport_advanced)より改変)

| 型 | 意味 | 宣言例 |
| ---- | ---- | ---- |
| short | short型整数 | short shortVariable; |
| long | long型整数 | long longVariable; |
| unsinged short | short型整数 | unsigned short ushortVariable; |
| unsigned long | long型整数 | unsigned long ulongVariable; |
| float | 単精度浮動小数点 | float floatVariable; |
| double | 倍精度浮動小数点数 | double doubleVariable; |
| char | 文字型 | char charVariable; |
| wchar | wchar文字型 | char charVariable; |
| boolean | bool型 | bool shortVariable; |
| octet | octet型 | octet octetVariable; |
| longlong | longlong型整数 | longlong longlongVariable; |
| ulonglong | unsinged longlong型整数 | ulonglong ulonglongVariable; |
| sequence<T> | シーケンス型 | sequence<double> doubleSeqVariable; |
| string | 文字列型 | string stringVariable; |

- ユーザ定義型

  上記サンプルでは、`BasicDataType.idl`で定義されているRTC::Time型を使用している. そのために、一行目で`BasicDataType.idl`をincludeしている.

### 2.2 Creating Service Type
ROSのsrvに相当する、通信で用いるサービスの型を定義する.

以下のサンプルでは、`addTwoInts`, `addTwoTime`, `addTwoTimedDoubleSeq`という3つのサービスコールをもったインターフェース(`sample_idl::MyOriginalService`)を定義している.
```
#include "BasicDataType.idl"

module sample_idl
{
  interface MyOriginalService
  {
    long addTwoInts(in long a, in long b);

    boolean addTwoTime(in RTC::Time a, in RTC::Time b, out RTC::Time sum);

    boolean addTwoTimedDoubleSeq(in RTC::TimedDoubleSeq a, in RTC::TimedDoubleSeq b, out RTC::TimedDoubleSeq sum);

  };
};
```

同じ内容のファイルが[MySrvSample.idl](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_idl/idl/MySrvSample.idl)にある.

一つのファイルで複数のインターフェースを定義してもよい.

引数に`in`をつけると入力となり、`out`をつけると出力となる.

### 2.3 Build

package.xmlに以下を追加する
```
<depend>rtmbuild</depend>
```

CMakeLists.txtを以下のように書く
```
find_package(catkin REQUIRED COMPONENTS rtmbuild)

# initialize rtmbuild. Call BEFORE catkin_package
rtmbuild_init()

catkin_package()

# add_custom_command to compile idl/*.idl file into c++
rtmbuild_genidl()

add_custom_target(genidl ALL DEPENDS RTMBUILD_${PROJECT_NAME}_genrpc)
```

これらを行ったパッケージが、[sample_idl](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/openrtm_beginner_tutorial/sample_idl)にある.

ビルドする
```
catkin build sample_idl
```

### 2.4 Use (for cpp)

CORBAの通信で用いるSkelとStubが生成されており、これらをリンクすることでcppのプログラムで使用できる
```bash
$ roscd
$ ls lib
libMyDataSampleSkel.so libMyDataSampleStub.so libMySrvSampleSkel.so libMySrvSampleStub.so
```

プログラム中では次のように使用する
```c++
#include <sample_idl/idl/MyDataSample.hh>

sample_idl::MyData a;
```

### 2.4 Use (for python)
プログラム中では次のように使用する
```python
import OpenRTM_aist
import sample_idl
from sample_idl import MyDataSample_idl

sample_idl.MyData
```

### 2.5 Use (for shell)

(独自に定義した型を`rtprint`で表示しようとするとうまくいかない場合がある.)
