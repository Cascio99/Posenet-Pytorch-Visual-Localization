#!/usr/bin/env python3

from message_filters import ApproximateTimeSynchronizer, Subscriber
import os
import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

img_pose_cnt = 1
# img_dir = "train" # for trian.bag
img_dir = "test" # for test.bag
pose_dir = ""

if not os.path.exists(img_dir):
    os.makedirs(img_dir)

bridge = CvBridge()

def callback(img_msg, odom_msg):
    global img_pose_cnt

    # extract .png from image_message
    cv_img = bridge.compressed_imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
    img_path = os.path.join(img_dir, "frame%05i.png"%img_pose_cnt)
    cv2.imwrite(img_path, cv_img)

    # extract pose from lidar_message
    p = odom_msg.pose.pose
    pos = p.position
    ori = p.orientation
    pose_info = f"{img_dir}/frame{img_pose_cnt:0>5}.png {pos.x} {pos.y} {pos.z} {ori.x} {ori.y} {ori.z} {ori.w}\n"
    
    pose_path = os.path.join(pose_dir, f"dataset_{img_dir}.txt")
    with open(pose_path, "a") as f:
        if img_pose_cnt  == 1:
            intro = "\nImageFile, Camera Position [X Y Z W P Q R]\n\n"
            f.write(intro)
        f.write(pose_info)

    img_pose_cnt += 1

def listener():
    # initialize ROS node
    rospy.init_node("synchronizer", anonymous=True)
    
    # subscribe topic
    img_sub = Subscriber("/zed/left/image_rect_color/compressed", CompressedImage)
    
    # odom_sub = Subscriber("/odom", Odometry)
    # odom_sub = Subscriber("/zed/odom", Odometry)
    odom_sub = Subscriber("/imu_odom", Odometry)

    # image_pose time-synchronization
    ts = ApproximateTimeSynchronizer([img_sub,odom_sub], queue_size=10, slop=0.1)
    ts.registerCallback(callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == "__main__":
    listener()
