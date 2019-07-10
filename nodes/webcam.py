#!/home/arnaud/opencv_py_env/bin/python2.7
"""

Publisher node in python which publishes images from
a webcam.

In a bash terminal:
    rosrun opencv_test webcam.py

"""

import rospy
import sys
print("Python interpreter: " + sys.executable)
import cv2
print("OpenCV version " + cv2.__version__)
from cv_bridge import CvBridge  # bridge between opencv and ROS : data conversions.
from sensor_msgs.msg import Image  # ROS image message.
print("--------------------------------------")

def main():
    rospy.init_node('image_pub', log_level=rospy.DEBUG, anonymous = True)  # the second option makes the debug info visible.
    # Publisher creation : image msg.
    topic = "initial_image"
    pub_img = rospy.Publisher(topic, Image)
    rospy.loginfo("I will publish to the topic %s", topic)
    rospy.loginfo('webcam node started')
    # Webcam stream (if there is not any other camera plugged to your computer)
    webcam = cv2.VideoCapture(0)
    # publishing loop the image.
    rate = rospy.Rate(30) # hz
    while not rospy.is_shutdown():

        # Webcam processing : retrieving the image.
        flag, img = webcam.read()
        if flag: # reading success.
            cv2.imshow("initial image", img)  # displays the image.
            cv2.waitKey(1) # waits 1ms
            # Image conversion to ROS standards.
            bdg = CvBridge()
            out_format = "bgr8"  # I can see it with image_view on rqt
            # out_format = "passthrough"  # passthough keeps the same encoding format.
            img_msg = bdg.cv2_to_imgmsg(img, out_format)

            # Publishing.
            str = "publishing at %s" % rospy.get_time()
            rospy.loginfo(str)
            img_msg.header.stamp = rospy.get_rostime() # timestamp actualization.
            rospy.loginfo("timestamp : % s.", img_msg.header.stamp)
            # image dimensions.
            dim = img.shape
            img_msg.height = dim[0]
            img_msg.width = dim[1]
            rospy.loginfo("Image size : w : %s x h : %s.", img_msg.width, img_msg.height)
            # publishing.
            pub_img.publish(img_msg)
            rate.sleep()
        else:
            rospy.logwarn("IMPOSSIBLE to read the image")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
