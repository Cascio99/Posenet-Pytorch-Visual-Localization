#!/usr/bin/env python3

from message_filters import ApproximateTimeSynchronizer, Subscriber
import os
import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

img_pose_cnt = 1
img_dir = "./data/train" # for trian.bag
# img_dir = "./data/test" # for test.bag
img_path = os.path.join(img_dir, "frame%05i.png"%img_pose_cnt)
pose_dir = "./data"
pose_path = os.path.join(pose_dir, "dataset_train.txt")
# pose_path = os.path.join(pose_dir, "dataset_test.txt")

if not os.path.exists(img_dir):
    os.makedirs(img_dir)
f = open(pose_path, "a")
intro = "\nImageFile, Camera Position [X Y Z W P Q R]\n\n"
f.write(intro)

bridge = CvBridge()

def callback(img_msg, odom_msg):
    global img_pose_cnt

    # extract .png from image_message
    cv_img = bridge.compressed_imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
    cv2.imwrite(img_path, cv_img)

    # extract pose from lidar_message
    pose = odom_msg.pose.pose
    x=pose.position.x, y=pose.position.y, z=pose.position.z
    qx=pose.orientation.x, qy=pose.orientation.y, qz=pose.orientation.z, qw=pose.orientation.w
    pose_info = f"{img_path} {x} {y} {z} {qx} {qy} {qz} {qw}\n"
    f.write(pose_info)

def listener():
    # initialize ROS node
    rospy.init_node("synchronizer", anonymous=True)
    
    # subscribe topic
    img_sub = Subscriber("/zed/left/image_rect_color/compressed", CompressedImage)

    # odom_sub = Subscriber("/odom", Odometry)
    odom_sub = Subscriber("/zed/odom", Odometry)    # has closest number of messages to the image_topic's
    # odom_sub = Subscriber("/imu/odom", Odometry)

    # image_pose time-synchronization
    ts = ApproximateTimeSynchronizer([img_sub,odom_sub], queue_size=10, slop=0.01)
    ts.registerCallback(callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == "__main__":
    listener()