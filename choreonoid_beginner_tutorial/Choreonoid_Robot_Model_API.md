# Choreonoid Robot Model API

[ソースコード](https://github.com/choreonoid/choreonoid/tree/master/src/Body)のヘッダファイルを読むのが一番

親関節と子リンクが一体で扱われ,`Link`クラスを構成する点が特徴. 各`Link`の名前は,親関節の名前になる.

行列計算は`Eigen`が用いられている. [EigenTypes.h](https://github.com/choreonoid/choreonoid/blob/master/src/Util/EigenTypes.h)に行列型が定義されている

## cnoid::Bodyクラス
### 主なメンバ関数
- `const std::string& name() const`: ロボット名を返す. 通常は空文字列になっている.
- `const std::string& modelName() const`: ロボット名を返す. 通常はモデルファイル上のロボット名になっている.
- `int numJoints() const`: VRMLファイルで`jointId`が振られているfixedでない関節の総数を返す.
- `int numVirtualJoints() const`: VRMLファイルで`jointId`が振られていないfixedでない関節の総数を返す.
- `int numAllJoints() const`: fixedでない関節の総数を返す.
- `Link* joint(int id) const`: 指定した`jointId`の`Link`を返す.
- `int numLinks() const`: 全`Link`の総数を返す.
- `Link* link(int index) const`: 指定された番目の`Link`を返す.
- `Link* link(const std::string& name) const`: 指定された名前の`Link`を返す. (実際には親関節名であることに注意)
- `Link* rootLink() const`: ルートリンクを返す.
- `template<class DeviceType> DeviceList<DeviceType> devices() const`: 指定したタイプの`Device`のリストを返す
- `template<class DeviceType> DeviceType* findDevice(const std::string& name) const`: 指定したタイプの指定した名前の`Device`を返す
- `double mass() const`: 全身の重量の和を返す
- `void calcForwardKinematics(bool calcVelocity = false, bool calcAcceleration = false)`: 各`Link`の`q`とルートリンクの`p`,`R`から、全`Link`の`p`,`R`を計算する. `calcVelocity`がtrueなら、各`Link`の`q`,`dq`とルートリンクの`p`,`v`,`R`,`w`から、全`Link`の`p`,`v`,`R`,`w`を計算する. さらに`calcAcceleration`がtrueなら、各`Link`の`q`,`dq`,`ddq`とルートリンクの`p`,`v`,`dv`,`R`,`w`,`dw`から、全`Link`の`p`,`v`,`dv`,`R`,`w`,`dw`を計算する.
- `const Vector3& calcCenterOfMass()`: 各`Link`の`q`とルートリンクの`p`,`R`から、重心位置を計算して返す. 事前に`calcForwardKinematics()`が必要.
- `const Vector3& centerOfMass() const`: キャッシュされた重心位置を返す.
- `void calcTotalMomentum(Vector3& out_P, Vector3& out_L)`: 各`Link`の`q`,`dq`とルートリンクの`p`,`v`,`R`,`w`から、全身の運動量とルートリンク周りの角運動量を計算する.事前に`void calcForwardKinematics(True)`が必要.

## ユーティリティー関数
- `void cnoid::calcMassMatrix(Body* body, const Vector3& g, MatrixXd& out_M)`: `6+n`x`6+n`の慣性行列を計算する. 事前に`void calcForwardKinematics()`と`calcCenterOfMass()`が必要.release-1.7にはバグがあるが、最新の開発版では修正されている. (`#include <cnoid/src/Body/MassMatrix.h>`)
- `void cnoid::Vector6 cnoid::calcInverseDynamics(Link* link)`: 各`Link`の`q`,`dq`,`ddq`とルートリンクの`p`,`v`,`dv`,`R`,`w`,`dw`から、全`Link`の`u`を計算する.事前に`void calcForwardKinematics()`が必要. 重力を考慮するにはルートリンクを鉛直上向きに重力加速度で加速させる. (`#include <cnoid/src/Body/InverseDynamics.h>`)
- `JointPathPtr cnoid::getJointPath(Link* baseLink, Link* targetLink)`: `baseLink`から`targetLink`への間にある関節の鎖を返す
- `void cnoid::calcCMJacobian(Body* body, Link* base, Eigen::MatrixXd& J)`: 各`Link`の`q`とルートリンクの`p`,`R`から、重心ヤコビアンを計算して返す. 事前に`void calcForwardKinematics()`と`calcCenterOfMass()`が必要. (`#include <cnoid/Jacobian>`)
- `void cnoid::calcAngularMomentumJacobian(Body* body, Link* base, Eigen::MatrixXd& H)`: 各`Link`の`q`とルートリンクの`p`,`R`から、角運動量ヤコビアンを計算して返す. 事前に`void calcForwardKinematics()`と`calcCenterOfMass()`が必要. release-1.7にはバグがあるが、最新の開発版では修正されている. (`#include <cnoid/Jacobian>`)

## cnoid::Linkクラス
### 主なメンバ関数
- `double& q()`: 関節角度
- `double& dq()`: 関節角速度
- `double& ddq()`: 関節角加速度

以下のメンバ関数は、ルートリンク以外は、自動で計算されるのでユーザーが自分でセットすることは稀である.
- `Position& T()`: リンクのworld位置姿勢
- `Vector3& p()`: リンクのworld位置
- `Vector3& v()`: リンクのworld速度
- `Vector3& dv()`: リンクのworld加速度
- `Matrix3& R()`: リンクのworld姿勢
- `Vector3& w()`: リンクのworld角速度
- `Vector3& dw()`: リンクのworld角加速度

以下のメンバ関数は、自動で計算されるのでユーザーが自分でセットすることは稀である.
- `double& u()`: 関節トルク

以下のメンバ変数は、モデルファイルから読み込まれるのでユーザーが自分でセットすることは稀である.
- `const std::string& name() const`: 関節の名前
- `JointType jointType() const { return jointType_; }`: 関節のタイプ(`REVOLUTE_JOINT | ROTATIONAL_JOINT | PRISMATIC_JOINT | FREE_JOINT | FIXED_JOINT`)
- `int jointId() const`: jointId
- `const Vector3& a() const`: joint axis(self local)
- `Position::ConstTranslationPart b() const`: relative position (parent local)
- `const Vector3& c() const`: center of mass (self local)
- `double m() const`: mass
- `const Matrix3& I() const`: inertia tensor (self local, around c)
- `Body* body()`
- `Link* parent const`
- `Link* sibling const`
- `Link* child const`

## cnoid::Device クラス
子クラスにcnoid::ForceSensorクラス, cnoid::RateGyroSensorクラス, cnoid::AccelerationSensorクラスなどがある
### 主なメンバ関数
以下のメンバ変数は、モデルファイルから読み込まれるのでユーザーが自分でセットすることは稀である.
- `const std::string& name() const`: センサの名前
- `const int id() const`: sensorId
- `const Link* link() const`: 親リンク
- `Isometry3::ConstLinearPart R_local() const`: センサの姿勢(parent local)
- `Isometry3::ConstTranslationPart p_local() const`: センサの位置(parent local)
- `Isometry3& T_local()`: センサの位置姿勢(parent local)
