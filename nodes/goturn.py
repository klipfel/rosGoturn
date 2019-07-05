#!/home/arnaud/opencv_py_env/bin/python2.7
"""

Node that reads the image published by a camera
converts it to an openCV format and
processes it.

"""
# ROS libraries
import rospy
from cv_bridge import CvBridge
# Other libraries
import cv2
# message
from sensor_msgs.msg import Image

def callback(msg):
    try:
        # Image processing.
        encoding = msg.encoding
        rospy.loginfo("Image processed at time %s, with encoding type %s.", rospy.get_time(), encoding)
        # convert sensor_msgs/Image to OpenCV Image
        bdg = CvBridge()
        # convertion of the image into openCV format.
        # conservation of the same encoding type.
        img_cv2 = bdg.imgmsg_to_cv2(msg, encoding)
        cv2.imshow("Recovered image", img_cv2)
        cv2.waitKey(1)  # waits.
    except Exception as err:
        rospy.loginfo(err)

def main():
    # Node initialization.
    rospy.init_node('goturn', log_level=rospy.DEBUG, anonymous = True)
    rospy.loginfo("Subscriber starts.")
    # Subscriptions.
    sub = rospy.Subscriber("initial_image",Image,callback)
    # Process.
    rospy.spin()
    # Destroyes all windows after the end of the node.
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
