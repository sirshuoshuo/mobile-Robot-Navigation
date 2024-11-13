#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def laser_callback(msg):
    # 获取ranges数据并过滤掉为0的值
    ranges = [r for r in msg.ranges if r > 0]
    # 检查是否存在非零的距离值SSS
    if ranges:
        min_distance = min(ranges)
        # max_distance = max(ranges)
        rospy.loginfo("Minimum non-zero distance: %f meters" % min_distance)
        # rospy.loginfo("Maximum non-zero distance: %f meters" % max_distance)
    else:
        rospy.loginfo("No valid distance data available")

def laser_listener():
    # 初始化ROS节点
    rospy.init_node('laser_listener', anonymous=True)
    
    # 订阅/scan主题
    rospy.Subscriber("/scan", LaserScan, laser_callback)
    
    # 保持节点运行
    rospy.spin()
if __name__ == '__main__':
    try:
        laser_listener()
    except rospy.ROSInterruptException:
        pass