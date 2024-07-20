#!/usr/bin/env python3

import os
import rospy
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Vector3, Pose
from visualization_msgs.msg import Marker, MarkerArray

# from message_filters import Subscriber
# from nav_msgs.msg import Odometry

class poseMarker:
    def __init__(self, name, id, r=0, g=0, b=0, a=255):
        self.marker = Marker()
        self.marker.header.frame_id = 'map'
        self.marker.ns = name
        self.marker.id = id
        self.marker.action = Marker.ADD
        self.marker.color = ColorRGBA(r/255.0, g/255.0, b/255.0, a/255.0)
        self.marker.type = Marker.POINTS
        self.marker.scale = Vector3(0.1, 0.1, 0.1)
        # self.idx = 0

        self.GT_pose = rospy.Subscriber('/GT_pose', Pose, self.GT_callback)
        self.pred_pose = rospy.Subscriber('/pred_pose', Pose, self.pred_callback)
        
        """self.poselist = []
        self.f = open("/home/ysw/ws/PoseNet-Pytorch_AIR/data/dataset_test.txt")
        self.read_file()

    def mark(self):
        pose = list(self.poselist[self.idx])
        self.marker.points.append(Point(pose[0], pose[1], pose[2]))

    def read_file(self):
        raw_lines = self.f.readlines()
        lines = raw_lines[3:]
        
        for line in lines:
            splits = line.split()
            # filename = splits[0]
            values = splits[1:]
            values = list(map(lambda x: float(x.replace(",", "")), values))

            self.poselist.append(values)"""
        
    def GT_callback(self, GT_pose):
        x = GT_pose.position.x
        y = GT_pose.position.y
        z = GT_pose.position.z
        # mark()
        m1.marker.points.append(Point(x, y, z))
        # publish
        pub_train.publish(m1.marker)
        # rospy.sleep(0.1)

    def pred_callback(self, pred_pose):
        # print("HELP@@@@@@@@@@@@@@@@@@@@@")
        x = pred_pose.position.x
        y = pred_pose.position.y
        z = pred_pose.position.z
        # mark()
        m2.marker.points.append(Point(x, y, z))
        # publish
        pub_test.publish(m2.marker)
        # rospy.sleep(0.1)

def main():
    global m1           # GT_pose
    global pub_train
    m1 = poseMarker(name='GT_marker', id=1, b=255)
    pub_train = rospy.Publisher('/GT_marker', Marker, queue_size=10)    # 100

    global m2           # pred_pose
    global pub_test
    m2 = poseMarker(name='pred_marker', id=2, g=255)
    pub_test = rospy.Publisher('/pred_marker',Marker, queue_size=10)    # 100

    rospy.init_node('rviz_node', anonymous=False)

    rate = rospy.Rate(100)
    # rate.sleep()
    rospy.spin()
    

    """rate = rospy.Rate(100)    # 1 Hz
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
        m1.idx += 1"""

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass