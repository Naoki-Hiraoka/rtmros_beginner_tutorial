# Installing and Configuring Your OpenRTM-ROS Environment

## 1. ROSのインストール

[ROS installation instructions](http://wiki.ros.org/ROS/Installation)に従い、ROSをインストール・セットアップする

## 2. OpenRTM-ROS相互連携ソフトウェアのインストール

```bash
sudo apt install ros-${ROS_DISTRO}-openrtm-aist
sudo apt install ros-${ROS_DISTRO}-openrtm-tools
sudo apt install ros-${ROS_DISTRO}-rtmbuild
source /opt/ros/${ROS_DISTRO}/setup.bash
```

- [openrtm_aist](https://github.com/OpenRTM/OpenRTM-aist). OpenRTMのソフトウェアプラットフォーム. 使用するバージョンは1.1.2.
- [openrtm_tools](https://github.com/start-jsk/rtmros_common/tree/master/openrtm_tools). 各種シェルコマンドがインストールされる
- [rtmbuild](https://github.com/start-jsk/rtmros_common/tree/master/rtmbuild). ビルドツールがインストールされる

## 3. 環境設定

```bash
echo "export ORBgiopMaxMsgSize=2097152000" >> ~/.bashrc
```
1回に送信するデータサイズの最大サイズの制限を緩和する.