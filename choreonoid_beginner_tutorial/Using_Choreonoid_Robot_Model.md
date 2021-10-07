# Using Choreonoid Robot Model

## 1. Choreonoid Robot Model

Choreonoidは、VRML(OpenHRP形式)モデルやBodyモデルを読み込んで、運動学・動力学計算を行うことができる. OpenHRP3の進化形として設計されているため、APIがよく似ている.

C++のプログラムで, モデルを読み込む方法を説明する.

サンプルコードは[sample_choreonoid_model](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/choreonoid_beginner_tutorial/sample_choreonoid_model)にある

## 2. The Code

ここでは、`/opt/ros/melodic/share/openhrp3/share/OpenHRP-3.1/sample/model/sample1.wrl`にあるSampleRobotモデルを読み込むコードで説明する. ROSのDISTROが違う場合にはパスを修正すること.

[sample.cpp](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/choreonoid_beginner_tutorial/sample_choreonoid_model/src/sample.cpp)
```c++
#include <iostream>

#include <cnoid/Body>
#include <cnoid/BasicSensors>

#include <cnoid/BodyLoader>

int main(void){
  // load robot
  cnoid::BodyLoader bodyLoader;
  cnoid::BodyPtr robot = bodyLoader.load("/opt/ros/melodic/share/openhrp3/share/OpenHRP-3.1/sample/model/sample1.wrl");
  if (!robot){
    std::cerr << "failed to load model" << std::endl;
    return 1;
  }

  // print robot names
  for(int i=0;i<robot->numJoints() ;i++){
    std::cout << robot->joint(i)->name() << std::endl;
  }

  return 0;
}
```

## 3. The Code Explained

```c++
#include <cnoid/Body>
#include <cnoid/BasicSensors>
```
ロボットモデルのクラスが定義されたファイル郡

```c++
#include <cnoid/BodyLoader>
```
モデルをロードするために必要なファイル

```c++
int main(void){
  // load robot
  cnoid::BodyLoader bodyLoader;
  cnoid::BodyPtr robot = bodyLoader.load("/opt/ros/melodic/share/openhrp3/share/OpenHRP-3.1/sample/model/sample1.wrl");
  if (!robot){
    std::cerr << "failed to load model" << std::endl;
    return 1;
  }
```

ここでモデルをロードしている.

`cnoid::Body`クラスがロボットモデルに対応したクラスである. `cnoid::BodyPtr`クラスはc++のsharedポインタのような役割を果たすが、c++のsharedポインタと比べて、生のポインタで渡してもオブジェクトカウントが正しく増減するという性質を持つ.

```c++
  // print robot names
  for(int i=0;i<robot->numJoints() ;i++){
    std::cout << robot->joint(i)->name() << std::endl;
  }
```
読み込んだロボットの関節名を表示させている.

## 4. Build Code

CMakeLists.txtに以下を記述する.
```
find_package(catkin REQUIRED)
find_package(choreonoid REQUIRED)
catkin_package()

include_directories(${CHOREONOID_INCLUDE_DIRS})
link_directories(${CHOREONOID_LIBRARY_DIRS})
add_executable(sample src/sample.cpp)
target_link_libraries(sample ${CHOREONOID_BODY_LIBRARIES_ABS})

install(TARGETS sample
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

Choreonoidはcatkinのパッケージではないため、`find_package(catkin REQUIRED COMPONENTS choreonoid)`で自動的にinclude, linkを設定することができない.

## 5. Run Example

上記を作成したものが[sample_choreonoid_model](https://github.com/Naoki-Hiraoka/rtmros_beginner_tutorial/blob/master/choreonoid_beginner_tutorial/sample_choreonoid_model)にある。

ビルド
```bash
catkin build sample_choreonoid_model
```

実行
```bash
rosrun sample_choreonoid_model sample
```

出力(抜粋)
```
RLEG_HIP_R
RLEG_HIP_P
RLEG_HIP_Y
RLEG_KNEE
RLEG_ANKLE_P
RLEG_ANKLE_R
RARM_SHOULDER_P
RARM_SHOULDER_R
RARM_SHOULDER_Y
RARM_ELBOW
RARM_WRIST_Y
RARM_WRIST_P
RARM_WRIST_R
LLEG_HIP_R
LLEG_HIP_P
LLEG_HIP_Y
LLEG_KNEE
LLEG_ANKLE_P
LLEG_ANKLE_R
LARM_SHOULDER_P
LARM_SHOULDER_R
LARM_SHOULDER_Y
LARM_ELBOW
LARM_WRIST_Y
LARM_WRIST_P
LARM_WRIST_R
WAIST_P
WAIST_R
CHEST
```
