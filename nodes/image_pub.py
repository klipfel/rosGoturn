#!/home/arnaud/opencv_py_env/bin/python2.7
"""

Publisher node in python which is converting
the given image into a ros image message.
The initial image has to be in the folder where this code is.
( relative path )
ex :
    /home/arnaud/Pictures/Wallpapers/70557.jpg
    /home/arnaud/programs/ros/catkin_ws/src/opencv_test/nodes/ros_logo.jpg

In a bash terminal:
    rosrun opencv_test image_pub.py ros_logo.jpg

"""

import rospy
import sys
print("Python interpreter: " + sys.executable)
import cv2
print("OpenCV version " + cv2.__version__)
from cv_bridge import CvBridge  # bridge between opencv and ROS : data conversions.
from sensor_msgs.msg import Image  # ROS image message.
print("--------------------------------------")

def start_node(image_name):
    rospy.init_node('image_pub', log_level=rospy.DEBUG, anonymous = True)  # the second option makes the debug info visible.
    # Publisher creation : image msg.
    topic = "initial_image"
    pub_img = rospy.Publisher(topic, Image, queue_size=10)
    rospy.loginfo("I will publish to the topic %s", topic)
    rospy.loginfo('image_pub node started')
    rospy.loginfo("Reading the image : %s", image_name)
    # OpenCV : problem with the relative path ..
    #img = cv2.imread(image_name)
    #img = cv2.imread("/home/arnaud/Pictures/Wallpapers/70557.jpg")
    #img = cv2.imread("ros_logo.jpg")
    img = cv2.imread("/home/arnaud/programs/ros/catkin_ws/src/opencv_test/nodes/ros_logo.jpg")
    cv2.imshow("initial image", img)  # displays the image.
    cv2.waitKey(0) # waits the user to press any key before closing.
    # clock on the windows to then press any key on your keyboard.
    cv2.destroyAllWindows()
    # Image conversion to ROS standards.
    bdg = CvBridge()
    out_format = "bgr8"  # I can see it with image_view on rqt
    # out_format = "passthrough"  # passthough keeps the same encoding format.
    img_msg = bdg.cv2_to_imgmsg(img, out_format)
    #rospy.loginfo(img_msg)  # displays the message
    # publishing loop the image.
    rate = rospy.Rate(30) # hz
    while not rospy.is_shutdown():
        str = "publishing at %s" % rospy.get_time()
        rospy.loginfo(str)
        pub_img.publish(img_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        print("sys command args : " + str(sys.argv))
        cleared_args = rospy.myargv(argv=sys.argv)  # removes the arguments internally given
        # by ROS.
        print("Filtered args : " + str(cleared_args))
        image_name = cleared_args[1]
        print("Loaded image : " + image_name)
        # node process.
        start_node(image_name)
    except rospy.ROSInterruptException:
        pass
