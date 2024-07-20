#!/usr/bin/env python3

import os
import rospy
from geometry_msgs.msg import Pose

class dataPublisher:
    def __init__(self):
        self.idx = 0
        self.poselist = []

    def read_file(self):
        self.f = open("/home/ysw/catkin_ws/src/visualizer/src/posenet_pytorch/data/dataset_test.txt")
        raw_lines = self.f.readlines()
        lines = raw_lines[3:]

        for line in lines:
            splits = line.split()
            values = splits[1:]
            values = list(map(lambda x: float(x.replace(",","")), values))
        
            self.poselist.append(values)

    def publish_pose(self):
        rospy.init_node('data_publish_node', anonymous=True)
        pose_pub = rospy.Publisher('/GT_pose', Pose, queue_size=0) #10
        rate = rospy.Rate(200)  # 200 Hz
        """consider delay by running posenet_pytorch"""
        if self.idx == 0:
            rospy.sleep(65) # 70 s
        while not rospy.is_shutdown():
            if self.idx > len(self.poselist):
                print("EOF")
                rospy.spin()
                # exit()
            pose  = Pose()
            pose.position.x = self.poselist[self.idx][0]
            pose.position.y = self.poselist[self.idx][1]
            pose.position.z = self.poselist[self.idx][2]
            pose.orientation.x = self.poselist[self.idx][3]
            pose.orientation.y = self.poselist[self.idx][4]
            pose.orientation.z = self.poselist[self.idx][5]
            pose.orientation.w = self.poselist[self.idx][6]
            
            # info = f"{self.poselist[self.idx]} {rospy.get_time()}"
            # print(self.idx)
            # rospy.loginfo(info)
            print("data_publish_node: ", self.idx)
            
            pose_pub.publish(pose)
            self.idx += 1
            rate.sleep()
            # rospy.spin()

if __name__ == "__main__":
    try:
        d = dataPublisher()
        d.read_file()
        d.publish_pose()
    except rospy.ROSInterruptException:
        pass