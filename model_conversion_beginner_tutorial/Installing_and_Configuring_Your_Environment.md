# Installing and Configuring Your Environment

## 1. ROSのインストール

[ROS installation instructions](http://wiki.ros.org/ROS/Installation)に従い、ROSをインストール・セットアップする

## 2. OpenHRP3のインストール

```bash
sudo apt install ros-${ROS_DISTRO}-openhrp3
source /opt/ros/${ROS_DISTRO}/setup.bash
```
`.vrml`を`.dae`に変換するスクリプトがある

## 3. collada_urdfのインストール

```bash
sudo apt install ros-${ROS_DISTRO}-collada-urdf
sudo apt install ros-${ROS_DISTRO}-collada-urdf-jsk-patch
source /opt/ros/${ROS_DISTRO}/setup.bash
```
`.dae`を`.urdf`に変換するスクリプトがある

## 4. euscolladaのインストール
```bash
sudo apt install ros-${ROS_DISTRO}-euscollada
source /opt/ros/${ROS_DISTRO}/setup.bash
```
`.dae`を`.l`に変換するスクリプトがある

## 5. hrpsys_ros_bridgeのインストール
```bash
sudo apt install ros-${ROS_DISTRO}-hrpsys-ros-bridge
source /opt/ros/${ROS_DISTRO}/setup.bash
```
内部でOpenHRP3,collada_urdf,euscolladaを用いて`.vrml`を`.dae`,`.l`,`.urdf`に一括で変換するスクリプトがある
