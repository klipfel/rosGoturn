#!/home/arnaud/opencv_py_env/bin/python2.7
"""

Node that reads the image published by a camera
converts it to an openCV format and
processes it.

/!\ Launch this node with a terminal open in the folder
where this file is located. (The implementation of
Goturn needs it)

"""
# ROS libraries
import rospy
from cv_bridge import CvBridge
# Other libraries
import cv2, sys, os
OPENCV_VERSION = cv2.__version__
# message
from sensor_msgs.msg import Image
import threading

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
            rospy.logfatal(errorMsg)
            sys.exit()
        # Checks if the terminal launching the tracker is launched in the folder where
        # the models are located.
        if not (os.path.isfile('goturn.caffemodel') and os.path.isfile('goturn.prototxt')):
            rospy.logfatal(" The terminal has to be launched where the models are located.")
            sys.exit()

    def track(self, img):
        """
        Tracking process, outputs the bounding box.
        The initialization requires to launch the node (terminal)
        in the derictory where the models are.
        """
        #rospy.loginfo("FRAME NUMBER %s", self.identifier)
        if self.init_flag == False:
            # initialization.
            #bbox = (276, 23, 86, 320)
            bbox = cv2.selectROI(img, False)  # some issues, it freezes after.
            flag = self.tracker.init(img, bbox)
            if not(flag):
                rospy.logfatal("Cannot initialize the tracker .... ")
                sys.exit()
            else:
                rospy.loginfo("initialization SUCCESS.")
            self.init_flag = True  # initialization done.
        # tracking process : initialization already done.
        else:
            rospy.loginfo("Goturn processing at time %s.", rospy.get_time())
            # TODO

    def process(self,msg):
        # conversion to opencv.
        img_cv2 = conversion(msg)
        #display(img_cv2)
        # goturn processing
        self.track(img_cv2)

################################################################################################

def conversion(msg):
    """
    Convertion to OpenCV standards.
    """
    rospy.loginfo("Conversion at %s.. ", rospy.get_time())
    # Image processing.
    encoding = msg.encoding
    #rospy.loginfo("NEW CALLBACK : Image processed at time %s, with encoding type %s.", rospy.get_time(), encoding)
    # convert sensor_msgs/Image to OpenCV Image
    bdg = CvBridge()
    # convertion of the image into openCV format.
    # conservation of the same encoding type.
    img_cv2 = bdg.imgmsg_to_cv2(msg, encoding)
    return img_cv2

def display(img):
    cv2.imshow("Recovered image", img)
    cv2.waitKey(1)  # waits.

def check_dependencies():
    # TODO : forcing the opencv version.
    pass

################################################################################################

def main():
    # Node initialization.
    rospy.init_node('goturn', log_level=rospy.DEBUG, anonymous = True)
    rospy.loginfo("Subscriber starts.")
    # Subscriptions.
    #buffer_size = 2**30 # maximum possible buffer size
    #rospy.loginfo("Subscriber's buffer sizer : %s", buffer_size)
    #sub = rospy.Subscriber("initial_image",Image,callback, queue_size = 1, buff_size = buffer_size)
    # tracker initialization.
    global node  # makes the node structure available for each method.
    node = GoturnNode()
    # spinning loop for the frame.
    while not rospy.is_shutdown():
        msg = rospy.wait_for_message("initial_image",Image)  # gets a new message / frame.
        rospy.loginfo("Frame timestamp :  %s.", msg.header.stamp)
        rospy.loginfo("Frame processing starts at %s.", rospy.get_rostime())
        node.process(msg)  # frame processing.

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
