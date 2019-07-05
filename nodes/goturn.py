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
import cv2, sys, os
# message
from sensor_msgs.msg import Image

def callback(msg):
    try:
        img_cv2 = convertion(msg)
        #display(img_cv2)
        # goturn processing
        goturn(img_cv2)
    except Exception as err:
        rospy.loginfo(err)

def convertion(msg):
    """
    Convertion to OpenCV standards.
    """
    # Image processing.
    encoding = msg.encoding
    rospy.loginfo("Image processed at time %s, with encoding type %s.", rospy.get_time(), encoding)
    # convert sensor_msgs/Image to OpenCV Image
    bdg = CvBridge()
    # convertion of the image into openCV format.
    # conservation of the same encoding type.
    img_cv2 = bdg.imgmsg_to_cv2(msg, encoding)
    return img_cv2

def display(img):
    cv2.imshow("Recovered image", img)
    cv2.waitKey(1)  # waits.

def goturn(img):
    rospy.loginfo("Goturn processing.")
    tracker = cv2.TrackerGOTURN_create()
    rospy.loginfo(tracker)

def main():
    # Node initialization.
    rospy.init_node('goturn', log_level=rospy.DEBUG, anonymous = True)
    rospy.loginfo("Subscriber starts.")
    # Subscriptions.
    sub = rospy.Subscriber("initial_image",Image,callback)
    # Process.
    rospy.spin() # calls the callback each time a message is received.
    # Destroyes all windows after the end of the node.
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
