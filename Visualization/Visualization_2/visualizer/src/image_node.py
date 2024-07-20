#!/usr/bin/env python3

import os
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class imagePublisher:
    def __init__(self):
        self.idx = 1
        # self.poselist = []
        self.dir = "/home/ysw/catkin_ws/src/visualizer/src/posenet_pytorch/data/test"
        self.file_list = os.listdir(self.dir)

    """def read_file(self):
        
        # raw_lines = self.f.readlines()
        # lines = raw_lines[3:]

        # for line in lines:
        #     splits = line.split()
        #     values = splits[1:]
        #     values = list(map(lambda x: float(x.replace(",","")), values))
        
        #     self.poselist.append(values)"""

    def publish_image(self):
        rospy.init_node('image_node', anonymous=True)
        img_pub = rospy.Publisher('/image', Image, queue_size=0)   #10
        rate = rospy.Rate(200)  # 200 Hz
        """consider delay by running posenet_pytorch"""
        if self.idx == 1:
            rospy.sleep(65)    # 70 s
        while not rospy.is_shutdown():
            if self.idx > len(self.file_list):
                print("OOB")
                rospy.spin()
                # exit()
            img_path = os.path.join(self.dir,"frame%05i.png"%self.idx)
            # print("HELP!!!!!!!!, img_path is ",img_path)
            if os.path.isfile(img_path):
                print("image_node: ", self.idx)
                img = cv2.imread(img_path)
                bridge = CvBridge()
                img_msg = bridge.cv2_to_imgmsg(img, encoding="passthrough")
                img_pub.publish(img_msg)
                
                self.idx += 1
                rate.sleep()
                # rospy.spin()


if __name__ == "__main__":
    try:
        i = imagePublisher()
        i.publish_image()
    except rospy.ROSInterruptException:
        pass