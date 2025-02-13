#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import tkinter as tk
from tkinter import ttk
from forklift_msg.msg import meteorcar

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)

        # 初始化速度參數
        self.robot_speed_factor = 1.0  # 機器人速度倍率 (0.1x ~ 1.0x)
        self.fork_speed = 1500         # 牙叉升降速度 (0 ~ 3000)

        self.fork_msg = meteorcar()
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_fork = rospy.Publisher('/cmd_fork', meteorcar, queue_size=1, latch=True)

        self.window = tk.Tk()
        self.window.title("Robot Control Panel")
        self.window.geometry("400x600")

        # 3x3 移動按鈕
        button_config = [
            ("\u2196",  1,  2), ("\u2191",  1,  0),  ("\u2197",  1, -2),
            ("\u21ba",  0,  2), ("Stop",    0,  0),  ("\u21bb",  0, -2),
            ("\u2199", -1,  -2), ("\u2193", -1,  0),  ("\u2198", -1, 2)
        ]

        for i, (label, x, z) in enumerate(button_config):
            btn = tk.Button(self.window, text=label, font=("Arial", 14), width=5, height=2,
                            command=lambda x=x, z=z: self.send_command(x, z))
            btn.grid(row=i // 3, column=i % 3, padx=5, pady=5)

        # 牙叉控制按鈕
        btn_up = tk.Button(self.window, text="Fork Up", font=("Arial", 14), width=10, height=2,
                           command=lambda: self.send_fork_command(1))
        btn_up.grid(row=4, column=0, columnspan=3, pady=5)

        btn_down = tk.Button(self.window, text="Fork Down", font=("Arial", 14), width=10, height=2,
                             command=lambda: self.send_fork_command(-1))
        btn_down.grid(row=5, column=0, columnspan=3, pady=5)

        # 機器人速度滑桿 (0.1x ~ 1.0x)
        tk.Label(self.window, text="Robot Speed", font=("Arial", 12)).grid(row=6, column=0, columnspan=2, sticky="w")
        self.robot_speed_label = tk.Label(self.window, text=f"{self.robot_speed_factor:.2f}x", font=("Arial", 12))
        self.robot_speed_label.grid(row=6, column=2, sticky="e")

        self.robot_speed_slider = ttk.Scale(self.window, from_=2.0, to=0.1, orient="horizontal",
                                            command=self.update_robot_speed)
        self.robot_speed_slider.set(self.robot_speed_factor)
        self.robot_speed_slider.grid(row=7, column=0, columnspan=3, pady=5, sticky="ew")

        # 牙叉速度滑桿 (0 ~ 3000)
        tk.Label(self.window, text="Fork Speed", font=("Arial", 12)).grid(row=8, column=0, columnspan=2, sticky="w")
        self.fork_speed_label = tk.Label(self.window, text=f"{self.fork_speed} mm/s", font=("Arial", 12))
        self.fork_speed_label.grid(row=8, column=2, sticky="e")

        self.fork_speed_slider = ttk.Scale(self.window, from_=3000, to=0, orient="horizontal",
                                           command=self.update_fork_speed)
        self.fork_speed_slider.set(self.fork_speed)
        self.fork_speed_slider.grid(row=9, column=0, columnspan=3, pady=5, sticky="ew")

        self.window.mainloop()

    def send_command(self, dir_x, dir_z):
        """ 發送機器人移動指令 """
        twist = Twist()
        twist.linear.x = 0.25 * self.robot_speed_factor * dir_x  # 速度倍率應用
        twist.angular.z = 0.25 * dir_z  # 固定轉彎速度
        self.pub_cmd_vel.publish(twist)

    def send_fork_command(self, direction):
        """ 發送牙叉升降指令 """
        self.fork_msg.fork_velocity = self.fork_speed * direction
        self.pub_fork.publish(self.fork_msg)

    def update_robot_speed(self, value):
        """ 更新機器人移動速度倍率，並顯示在 Label """
        self.robot_speed_factor = float(value)
        self.robot_speed_label.config(text=f"{self.robot_speed_factor:.2f}x")
        rospy.loginfo(f"Robot Speed Factor: {self.robot_speed_factor:.2f}x")

    def update_fork_speed(self, value):
        """ 更新牙叉升降速度，並顯示在 Label """
        self.fork_speed = int(float(value))
        self.fork_speed_label.config(text=f"{self.fork_speed} mm/s")
        rospy.loginfo(f"Fork Speed: {self.fork_speed}")

if __name__ == '__main__':
    try:
        RobotController()
    except rospy.ROSInterruptException:
        pass
