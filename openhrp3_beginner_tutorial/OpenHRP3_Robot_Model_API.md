# OpenHRP3 Robot Model API

[ソースコード](https://github.com/fkanehiro/openhrp3/tree/master/hrplib/hrpModel)を読むのが一番

親関節と子リンクが一体で扱われ,`Link`クラスを構成する点が特徴. 各`Link`の名前は,親関節の名前になる.

行列計算は`Eigen`が用いられている. [EigenTypes.h](https://github.com/fkanehiro/openhrp3/blob/master/hrplib/hrpUtil/EigenTypes.h)に行列型が定義されている.

## hrp::Bodyクラス
### 主なメンバ関数
- `const std::string& name()`: ロボット名を返す.
- `inline unsigned int numJoints() const`: VRMLファイルで`jointId`が振られている関節の総数を返す.
- `inline Link* joint(int id) const`: 指定した`jointId`の`Link`を返す.
- `inline unsigned int numLinks() const`: 全`Link`の総数を返す.
- `inline Link* link(int index) const`: 指定された番目の`Link`を返す.
- `Link* link(const std::string& name) const`: 指定された名前の`Link`を返す. (実際には親関節名であることに注意)
- `inline Link* rootLink() const`: ルートリンクを返す.
- `inline Sensor* sensor(int sensorType, int sensorId) const`: 指定したタイプの指定した`sensorId`の`Sensor`を返す
- `inline unsigned int numSensors(int sensorType) const`: 指定したタイプの全`Sensor`の総数を返す
- `double calcTotalMass()`: 全身の重量の和を計算しキャッシュする.
- `inline double totalMass() const`: キャッシュされた全身の重量の和を返す
- `void calcForwardKinematics(bool calcVelocity = false, bool calcAcceleration = false)`: 各`Link`の`q`とルートリンクの`p`,`R`から、全`Link`の`p`,`R`を計算する. `calcVelocity`がtrueなら、各`Link`の`q`,`dq`とルートリンクの`p`,`v`,`R`,`w`から、全`Link`の`p`,`v`,`R`,`w`を計算する. さらに`calcAcceleration`がtrueなら、各`Link`の`q`,`dq`,`ddq`とルートリンクの`p`,`v`,`dv`,`R`,`w`,`dw`から、全`Link`の`p`,`v`,`dv`,`R`,`w`,`dw`を計算する.
- `Vector3 calcCM()`: 各`Link`の`q`とルートリンクの`p`,`R`から、重心位置を計算して返す. 事前に`calcForwardKinematics()`が必要.
- `void calcMassMatrix(dmatrix& out_M)`: `6+n`x`6+n`の慣性行列を計算する.
- `void calcInverseDynamics(Link* link, Vector3& out_f, Vector3& out_tau)`: 各`Link`の`q`,`dq`,`ddq`とルートリンクの`p`,`vo`,`dvo`,`R`,`w`,`dw`から、全`Link`の`u`を計算する.事前に`void calcForwardKinematics()`が必要. 重力を考慮するにはルートリンクを鉛直上向きに重力加速度で加速させる.
- `void calcTotalMomentum(Vector3& out_P, Vector3& out_L)`: 各`Link`の`q`,`dq`とルートリンクの`p`,`v`,`R`,`w`から、全身の運動量とルートリンク周りの角運動量を計算する.事前に`calcForwardKinematics(True)`が必要. 重力を考慮するにはルートリンクを鉛直上向きに重力加速度で加速させる.
- `JointPathPtr getJointPath(Link* baseLink, Link* targetLink)`: `baseLink`から`targetLink`への間にある関節の鎖を返す
- `void calcCMJacobian(Link *base, dmatrix &J)`: 各`Link`の`q`とルートリンクの`p`,`R`から、重心ヤコビアンを計算して返す. 事前に`calcForwardKinematics()`と`calcCM()`が必要.
- `void calcAngularMomentumJacobian(Link *base, dmatrix &H)`: 各`Link`の`q`とルートリンクの`p`,`R`から、角運動量ヤコビアンを計算して返す. 事前に`calcForwardKinematics()`と`calcCM()`が必要.

## hrp::Linkクラス
### 主なメンバ変数
- `double q`: 関節角度
- `double dq`: 関節角速度
- `double ddq`: 関節角加速度

以下のメンバ変数は、ルートリンク以外は、自動で計算されるのでユーザーが自分でセットすることは稀である.
- `Vector3 p`: リンクのworld位置
- `Vector3 v`: リンクのworld速度
- `Vector3 dv`: リンクのworld加速度
- `Vector3 vo`: リンクのworld速度(spacial velocity)
- `Vector3 dvo`: リンクのworld加速度(spacial velocity)
- `Matrix33 R`: リンクのworld姿勢
- `Vector3 w`: リンクのworld角速度
- `Vector3 dw`: リンクのworld角加速度

以下のメンバ変数は、自動で計算されるのでユーザーが自分でセットすることは稀である.
- `double u`: 関節トルク

以下のメンバ変数は、モデルファイルから読み込まれるのでユーザーが自分でセットすることは稀である.
- `std::string name`: 関節の名前
- `JointType jointType`: 関節のタイプ(`FIXED_JOINT | FREE_JOINT | ROTATIONAL_JOINT | SLIDE_JOINT`)
- `int jointId`: jointId
- `Vector3 a`: rotation joint axis(self local)
- `Vector3 d`: translation joint axis(self local)
- `Vector3 b`: relative position (parent local)
- `Vector3 c`: center of mass (self local)
- `double m`: mass
- `Matrix33 I`: inertia tensor (self local, around c)
- `Body* body`
- `Link* parent`
- `Link* sibling`
- `Link* child`

## hrp::Sensor クラス
### 主なメンバ変数
以下のメンバ変数は、モデルファイルから読み込まれるのでユーザーが自分でセットすることは稀である.
- `std::string name`: センサの名前
- `int type`: センサのタイプ(`FORCE | RATE_GYRO | ACCELERATION | PRESSURE | PHOTO_INTERRUPTER | VISION | TORQUE | RANGE | NUM_SENSOR_TYPES`)
- `int id`: sensorId
- `Link* link`: 親リンク
- `Matrix33 localR`: センサの姿勢(parent local)
- `Vector3 localPos`: センサの位置(parent local)
