# ROS2 humble forklift_driver

# 解除安裝 ubuntu22.04 預設的 brltty 
> $ sudo apt remove brltty

P.S. BRLTTY is a background process (daemon) 

## Serial port setting
> $ sudo chmod 777 /dev/ttyUSBx

## Dependency
安裝與編譯非官方版本的 Serial 庫，因為官方尚未更新 ROS2 分支，未來需要關注此項任務進展。
> ~/path/to/workspace/src$ git clone https://github.com/abdullahdangac/serial.git -b ros2

## 下載
> ~/path/to/workspace/src$ git clone https://github.com/Yuntechec404/forklift_driver.git -b humble

## 編譯
> ~/path/to/workspace$ colcon build

## 環境變數
> ~/path/to/workspace$ source install/setup.bash

## 啟動底盤
> ~/path/to/workspace$ ros2 launch forklift_driver forklift_driver.launch

## 底盤設定
**src/forklift_driver/launch/forklift_driver.launch**
```xml
<launch>  
	<node pkg="forklift_driver" name="forklift_driver" type="forklift_driver" output="screen" >
	<!-- launch-prefix="gdb -ex  args" --><!-- Debug option -->
	    <!-- Parameter setting -->
		<param name="rate" value="25" /> <!-- 執行頻率 -->
		<param name="timeout" value="3" /> <!-- 距離最後一次收到cmd_vel or cmd_fork幾秒後停止 -->
		<param name="wheel_base" value="0.3" /> <!-- 前輪到後輪中心距離 -->
		<param name="theta_bias" value="0" /> <!-- 前輪偏移，需要修正的角度 -->
		<param name="use_imu_flag" value="True" /> <!-- 是(True)否(False)使用imu角速度計算里程計 -->
		<param name="odom_tf_flag" value="False" /> <!-- 是(True)否(False)發布odom tf -->
		<param name="init_fork_flag" value="True" /> <!-- 是(True)否(False)讓牙叉降到底 -->

		<!--Subscriber Topic setting/-->
		<param name="topic_cmd_vel" value="/cmd_vel" /> <!--速度命令-->
		<param name="topic_cmd_fork" value="/cmd_fork" /> <!--牙叉速度命令-->

		<!--Publisher Topic setting/-->
		<param name="topic_odom" value="/wheel_odom" /> <!--車輪里程計-->
		<param name="topic_forklift_pose" value="/forklift_pose" /> <!--牙叉位置-->
		<param name="topic_imu" value="/imu" /> <!--IMU數值-->
	</node>
</launch>
```

# 鍵盤控制

1. 安裝 teleop-twist-keyboard
> sudo apt install ros-humble-teleop-twist-keyboard

2. 啟動鍵盤控制
> ros2 run teleop_twist_keyboard teleop_twist_keyboard


```
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
anything else : stop

CTRL-C to quit
```
⚠️ 請按 'z'  decrease max speeds by 10%  將每秒的線速度與角速度降到 0.1~0.3，不然車子會暴衝
