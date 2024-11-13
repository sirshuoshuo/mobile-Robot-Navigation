#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math


class HuskyHighlevelController:
    def __init__(self):
        rospy.init_node('husky_highlevel_controller', anonymous=True)
        self.p_ang = 1.0  
        self.fixed_velocity = 0.3  

        self.subscriber = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.msg = Twist()

        # 目标位置和距离
        self.target_distance = None
        self.target_angle = None

        self.goal_reached = False  # 

    def set_vel(self, vel, dof):
        
        if dof == "forward":
            self.msg.linear.x = vel
        elif dof == "ang":
            self.msg.angular.z = vel

    def publish_velocity(self):
        
        self.vel_pub.publish(self.msg)

    def adjust_speed(self, dist):
        
        vel = 0.0 if dist <= 0.15 else self.fixed_velocity
        self.set_vel(vel, "forward")

    def adjust_heading(self, ang):
        
        self.set_vel(-ang, "ang")

    def laser_callback(self, msg):
        
       
        valid_ranges = [r for r in msg.ranges if r > 0]
        valid_indices = [i for i, r in enumerate(msg.ranges) if r > 0]

        # 突变点
        mutation_points = self.find_mutation_points(valid_ranges)

        if mutation_points and not self.goal_reached:
            # 转向角度和移动距离
            target_angle, target_distance = self.calculate_target_angle_and_distance(mutation_points, valid_indices, msg)
            self.target_angle = target_angle
            self.target_distance = target_distance
            self.adjust_heading(target_angle)
            self.adjust_speed(target_distance - 0.15)  
            self.publish_velocity()

           
            if self.target_distance <= 0.:
                self.set_vel(0.0, "forward")  
                self.set_vel(0.0, "ang")  
                self.publish_velocity()

                rospy.loginfo(f"已到达目标位置，目标距离：{self.target_distance}m")
                self.goal_reached = True  

        elif self.goal_reached:
            
            rospy.loginfo("已经到达目标位置，停止运动。")

    def find_mutation_points(self, ranges):
        
        mutation_points = []
        for i in range(1, len(ranges) - 1):
            if abs(ranges[i] - ranges[i - 1]) > 1:  # 差值是否大于1米
                mutation_points.append(i)
        return mutation_points

    def calculate_target_angle_and_distance(self, mutation_points, valid_indices, msg):
        
        angles = []
        distances = []

        # 突变点
        close_mutations = []

        for i in range(1, len(mutation_points)):
            idx1 = mutation_points[i - 1]
            idx2 = mutation_points[i]
            
            
            if abs(valid_indices[idx2] - valid_indices[idx1]) < 5:  
                
                avg_angle = (msg.angle_min + idx1 * msg.angle_increment + msg.angle_min + idx2 * msg.angle_increment) / 2
                avg_distance = (msg.ranges[valid_indices[idx1]] + msg.ranges[valid_indices[idx2]]) / 2
                close_mutations.append((avg_angle, avg_distance))
            else:
                angles.append(msg.angle_min + idx1 * msg.angle_increment)
                distances.append(msg.ranges[valid_indices[idx1]])

        # 选择平均值
        if close_mutations:
            avg_angle = sum([m[0] for m in close_mutations]) / len(close_mutations)
            avg_distance = sum([m[1] for m in close_mutations]) / len(close_mutations)
            return avg_angle, avg_distance
        else:
            return angles[0], distances[0]

if __name__ == '__main__':
    controller = HuskyHighlevelController()
    rate = rospy.Rate(10)  

    while not rospy.is_shutdown():
        rate.sleep()
