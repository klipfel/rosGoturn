#!/home/arnaud/opencv_py_env/bin/python2.7
import rospy
from std_msgs.msg import Float64

def callback_bearing(msg):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s at %s", msg.data, rospy.get_rostime())
    # command computation.
    # TODO

def main():
    # Node initialization.
    rospy.init_node('tracker_controller', log_level=rospy.DEBUG, anonymous = True)
    rospy.loginfo("tracker_controller starts.")
    # Subscription.
    rospy.Subscriber("bearing_error", Float64, callback_bearing)
    # Publishing.
    # TODO : commands
    # Spinning.
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
