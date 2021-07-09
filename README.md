# scout_ros_driver
agilex scout-v2 robot ros driver

## 环境要求
- Ubuntu 18.04
- ROS melodic

## 安装与编译
更新软件
```shell
sudo apt-get update
```
安装依赖项
```shell
sudo apt-get install ros-melodic-joy
```
创建ROS工作空间
```shell
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
```
克隆代码并编译
```shell
git clone https://github.com/I-Quotient-Robotics/scout_ros_driver.git
git clone https://github.com/QuartzYan/iqr_teleop.git   # 手柄控制
cd ..
catkin_make
```
环境设置
```shell
echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
添加串口rules文件
```shell
roscd scout_base/config/
sudo cp ./58-scout.rules /etc/udev/rules.d/
sudo service udev reload
sudo service udev restart
sudo udevadm trigger
```

## 使用
- 首先，使用USB转232的串口线连接好机器人和计算机

- 启动驱动节点
```shell
roslaunch scout_base base.launch
```
- 发送速度消息
```shell
rostopic pub /scout_velocity_controller/cmd_vel geometry_msgs/Twist "linear:
  x: 0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -r 10
```