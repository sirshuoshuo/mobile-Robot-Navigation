import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math


class HuskyHighlevelController:
    def __init__(self):
        # 初始化节点
        rospy.init_node('husky_highlevel_controller', anonymous=True)

        # 参数设置
        self.p_ang = 1.0                 # 角度比例控制参数
        self.p_vel = 0.5                 # 速度比例控制参数
        topic = "/scan"                  # 激光扫描的订阅话题
        queue_size = 10                  # 队列大小

        # 设置订阅和发布话题
        self.subscriber = rospy.Subscriber(topic, LaserScan, self.laser_callback)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # 初始化控制消息
        self.msg = Twist()
        self.is_turning = False  # 初始状态不在旋转
        self.target_angle = None  # 记录目标角度
        self.initial_scan_complete = False  # 标记是否完成初始扫描

        rospy.loginfo("Husky highlevel controller node launched!")

    def set_vel(self, vel, dof):
        """设置速度命令"""
        if dof == "forward":
            self.msg.linear.x = vel
        elif dof == "ang":
            self.msg.angular.z = vel

    def publish_velocity(self):
        """发布速度信息"""
        self.vel_pub.publish(self.msg)

    def adjust_speed(self, dist):
        """根据距离调整前进速度"""
        # 当距离小于等于0.15米时停止
        if dist <= 0.15:
            vel = 0.0  # 停止机器人
        else:
            vel = self.p_vel * (dist - 0.16)  # 保持在距离0.16m时停止
            if vel > 0.5:
                vel = 0.5  # 限制最大速度
        self.set_vel(vel, "forward")
        self.publish_velocity()

    def adjust_heading(self, ang):
        """根据角度调整朝向"""
        diff = -ang
        self.set_vel(self.p_ang * diff, "ang")
        self.publish_velocity()

    def laser_callback(self, msg):
        """激光回调函数"""
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        # 检查是否符合旋转条件：检测到距离突变
        for i in range(1, len(ranges) - 1):
            # 确保不会越界
            left = ranges[i-1] if i-2 >= 0 else float('inf')  # 如果i-2超出范围，设为无效
            right = ranges[i+1] if i+2 < len(ranges) else float('inf')  # 如果i+2超出范围，设为无效
            center = ranges[i]

            # 检测到突变范围大于1米（表示障碍物或急剧变化）
            if abs(left - right) > 1.0:
                target_angle = angle_min + i * angle_increment
                if self.target_angle is None:
                    self.target_angle = target_angle
                    rospy.loginfo(f"Detected sudden change, target angle: {math.degrees(self.target_angle)} degrees")
                    # 启动旋转到目标角度
                    self.is_turning = True
                    self.adjust_heading(self.target_angle)
                    return

        # 如果已经旋转到目标角度，停止旋转
        if self.is_turning and self.target_angle is not None:
            current_angle = msg.angle_min + msg.angle_increment * ranges.index(min(ranges))
            if abs(current_angle - self.target_angle) < 0.1:  # 当接近目标角度时停止
                rospy.loginfo("Reached target angle, stopping rotation.")
                self.set_vel(0.0, "ang")  # 停止旋转
                self.publish_velocity()
                self.is_turning = False  # 停止旋转
                self.target_angle = None  # 重置目标角度

        # 如果未旋转，开始直线行驶
        if not self.is_turning:
            dist = min(msg.ranges)
            if dist < 0.15:
                rospy.loginfo("Stopping as distance is below 15cm.")
                self.set_vel(0.0, "forward")  # 停止前进
                self.set_vel(0.0, "ang")      # 停止旋转
                self.publish_velocity()
            else:
                # 调整前进速度
                self.adjust_speed(dist)


if __name__ == '__main__':
    controller = HuskyHighlevelController()
    rospy.spin()
