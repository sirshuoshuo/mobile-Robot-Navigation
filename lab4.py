#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class HuskyHighlevelController:
    def __init__(self, nh):
        self.p_ang = 0.5  # 控制角速度的比例增益
        self.warning_count = 0
        self.num = 0

        self.subscriber = rospy.Subscriber("/scan", LaserScan, self.LaserCallback)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist,queue_size=1)
        self.msg = Twist()

    def setVel(self, vel, dof):
        if dof == "forward":
            self.msg.linear.x = vel  # 设置线速度
        elif dof == "ang":
            self.msg.angular.z = vel  # 设置角速度

    def adjustSpeed(self, dist):
        # 调整机器人的前进速度，基于激光测得的距离使用比例控制（P 控制）
        vel = 1 * (dist - 0.15)  # 距离小于 0.16 米时停止
        if vel > 5.0:
            vel = 5.0  # 速度上限
        elif vel < 0.0:
            vel = 0.0  # 速度下限，停止前进

        self.setVel(vel, "forward")  # 设置调整后的线速度
        self.DriveHusky()  # 发布新的速度命令


    def adjustHeading(self, target_ang):
        angle_threshold = 0.05  # 设置阈值为 0.05 弧度（约 3度），你可以根据需要调整

        # 如果目标角度接近零，即机器人已经对准目标角度，停止旋转
        if abs(target_ang) < angle_threshold:
            self.setVel(0.0, "ang")  # 停止角速度
        else:
            # 否则，根据目标角度调整角速度（使用比例控制）
            self.setVel(self.p_ang * target_ang, "ang")

        self.DriveHusky()  # 发布新的速度命令

    def find_local_minimum_points(self, msg):
        # 设置距离的范围 1.0m 到 1.25m
        min_dist = 0.95
        max_dist = 1.35
        threshold = 2.5  # 周围的距离阈值，大于2.5m
        target_points = []  # 用于存储符合条件的目标点
        
        distancd = None
        ang = None
        i = 1
        while i < len(msg.ranges) - 1:  # 避免越界，检查前后点
            # 如果当前点的距离在 1.0 到 1.25 米之间，开始检查连续点
            if min_dist <= msg.ranges[i] <= max_dist:
                start = i  # 记录符合条件的连续段的开始位置
                # 检查后续点，确保这些点在 1.0m 到 1.25m 之间
                while i < len(msg.ranges) - 1 and min_dist <= msg.ranges[i] <= max_dist:
                    i += 1
                end = i  # 连续段的结束位置
                
                # 检查连续段前后的点是否大于阈值，确保这段是局部最小
                if start > 0 and end < len(msg.ranges) and msg.ranges[start - 1] > threshold and msg.ranges[end] > threshold:
                    # 如果符合条件，将连续段的角度和距离添加到目标点列表
                        distancd = msg.ranges[start+1]
                        ang = msg.angle_min + msg.angle_increment * (start+1)  # 计算每个点的角度
                        break
                        
            # 继续遍历下一个点
            i += 1
        if distancd is not None and ang is not None:
            self.warning_count = 0
        if distancd is None or ang is None:
            self.warning_count += 1
            rospy.logwarn("No valid local minimum point found.")
            distancd = 0.0  # 默认值
            ang = 0.0  # 默认值
        return distancd,ang
    def find_min_distance_above_threshold(self, msg, threshold=0.05):
        # 筛选出大于 0.05 的所有有效距离
        valid_distances = [d for d in msg.ranges if d > threshold]

        if valid_distances:
            # 找到最小距离
            min_distance = min(valid_distances)
            return min_distance

    def DriveHusky(self):
        # 发布速度命令，控制 Husky 机器人运动
        self.vel_pub.publish(self.msg)
    
    def LaserCallback(self, msg):
        dist,ang = self.find_local_minimum_points(msg)
        min_dis =self.find_min_distance_above_threshold(msg)
        rospy.loginfo("Object is %.2f meters away at %.2f degrees", dist, ang * 180.0 / math.pi)  # 打印物体的位置
        
        if self.warning_count >= 10:
            self.adjustHeading(0.0)
        else:
            self.msg.linear.x=0.0
            self.adjustHeading(ang)


        if self.warning_count >= 10 and self.num==1:
            rospy.loginfo("min distance is %.2f meter",min_dis)
            self.adjustSpeed(min_dis)

        if self.warning_count >= 10 and self.num==0:
            self.msg.linear.x = 1.0  # 设置线速
            self.DriveHusky()
            rospy.sleep(0.1)  # 延迟 2 秒，模拟机器人行驶 2 秒
            self.msg.linear.x = 0.0  # 设置前进速度为 0
            self.DriveHusky()  # 停止机器人
            self.num=1

if __name__ == "__main__":
    # 初始化 ROS 节点
    rospy.init_node("speed_controller")

    # 实例化控制器对象
    controller = HuskyHighlevelController(rospy)

    # 保持节点运行
    rospy.spin()

        
