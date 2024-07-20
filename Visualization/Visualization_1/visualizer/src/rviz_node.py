#!/usr/bin/env python3

import os
import rospy
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker

from message_filters import Subscriber
from nav_msgs.msg import Odometry

class poseMarker:
    def __init__(self, name, id, color_r=0, color_g=0, color_b=0, color_a=255):
        self.marker = Marker()
        self.marker.header.frame_id = 'map'
        self.marker.ns = name
        self.marker.id = id
        self.marker.action = Marker.ADD
        self.marker.color = ColorRGBA(color_r/255.0, color_g/255.0, color_b/255.0, color_a/255.0)
        
        self.poselist = []
        self.f = open("/home/ysw/ws/PoseNet-Pytorch_AIR/data/dataset_train.txt")
        self.read_file()

        # GT_pose = Subscriber("/integrated_to_init", Odometry)
        # predict_pose = Subscriber("/integrated_to_init", Odometry)
        self.idx = 0

    def mark(self):
        pose = list(self.poselist[self.idx])
        
        self.marker.type = Marker.POINTS
        self.marker.scale = Vector3(0.1, 0.1, 0.1)
        self.marker.points.append(Point(pose[0], pose[1], pose[2]))
        # pos = pose[:3]
        # ori = pose[3:]
        # self.marker.pose.position = pos
        # self.marker.pose.orientation = ori

    def read_file(self):
        raw_lines = self.f.readlines()
        lines = raw_lines[3:]
        
        for line in lines:
            splits = line.split()
            # filename = splits[0]
            values = splits[1:]
            values = list(map(lambda x: float(x.replace(",", "")), values))

            self.poselist.append(values)

    # def callback(self, GT_pose, predict_pose):
    #     pass

def main():
    pub_train = rospy.Publisher('/train_marker', Marker, queue_size=1000)
    # pub_test = rospy.Publisher('/test_marker',Marker, queue_size=10)
    
    rospy.init_node('rviz_node', anonymous=True)
    
    m1 = poseMarker(name='GT_point', id=1, color_b=255)

    # m2 = poseMarker(name='pred_point', id=2, color_g=255)
    # m2.mark()
    rate = rospy.Rate(1000)    # 1 Hz
    while not rospy.is_shutdown():
        if m1.idx == len(m1.poselist):
            print("EOF")
            exit()
        m1.mark()
        info = f"{m1.poselist[m1.idx]} {rospy.get_time()}"
        print(m1.idx)
        rospy.loginfo(info)
        pub_train.publish(m1.marker)
        # pub_test.publish(m2.marker)
        rate.sleep()
        m1.idx += 1
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass