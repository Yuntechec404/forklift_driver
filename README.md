# forklift_driver

## 啟動底盤
> $ roslaunch forklift_driver forklift_driver.launch

## 底盤設定
**src/forklift_driver/launch/forklift_driver.launch**
```yaml
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
src中的car_driver.cpp為以前的原始版本，現在在node資料夾中的是我看不下去重構過後的版本，並在launch檔中添加了幾個實用的選項。
