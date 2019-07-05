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

################################################################################################

class GoturnNode:

    def __init__(self):
        self.tracker = cv2.TrackerGOTURN_create()
        self.init_flag = False  # False, the tracker was not initialized.
        self.check_model()

    def check_model(self):
        # root dir.
        ROOT_DIR = os.path.dirname(os.path.abspath(__file__)) # since isfile depends on the terminal
        # location.
        rospy.loginfo("Root directory : %s", ROOT_DIR)
        if not (os.path.isfile(ROOT_DIR + '/goturn.caffemodel') and os.path.isfile(ROOT_DIR + '/goturn.prototxt')):
            errorMsg = " Could not find GOTURN model in current directory.\n Please ensure goturn.caffemodel and goturn.prototxt are in the current directory"
            rospy.loginfo(errorMsg)
            sys.exit()

    def track(self, img):
        """
        Tracking process, outputs the bounding box.
        The initialization requires to launch the node (terminal)
        in the derictory where the models are.
        """
        if self.init_flag == False:
            # initialization.
            bbox = (276, 23, 86, 320)
            #bbox = cv2.selectROI(img, False)  # some issues, it freezes after.
            flag = self.tracker.init(img, bbox)
            if not(flag):
                rospy.loginfo("Cannot initialize the tracker .... ")
                sys.exit()
            else:
                rospy.loginfo("initialization successfull.")
            self.init_flag = True
        # tracking process.
        else:
            rospy.loginfo("Goturn processing at time %s.", rospy.get_time())
            # TODO

################################################################################################

def callback(msg):
    try:
        img_cv2 = convertion(msg)
        #display(img_cv2)
        # goturn processing
        node.track(img_cv2)
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
    rospy.loginfo("Goturn processing at time %s.", rospy.get_time())

################################################################################################

def main():
    # Node initialization.
    rospy.init_node('goturn', log_level=rospy.DEBUG, anonymous = True)
    rospy.loginfo("Subscriber starts.")
    # Subscriptions.
    sub = rospy.Subscriber("initial_image",Image,callback)
    # tracker initialization.
    global node  # makes the node structure available for each method.
    node = GoturnNode()
    # Process.
    rospy.spin() # calls the callback each time a message is received.
    # Destroyes all windows after the end of the node.
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
