#!/home/arnaud/opencv_py_env/bin/python2.7
import rospy
from std_msgs.msg import Float64

# parameters of the servomotor control.
# u is equivalent to an angle (bearing).
u0 = 50    # straight.
umin = 0    # right
umax = 100    # left

# PID parameters
Kp = 100

def callback_bearing(msg):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s at %s", msg.data, rospy.get_rostime())
    # command computation.
    e = msg.data  # error regarding the bearing.
    u = u0 + Kp*e
    # Saturation.
    if u > umax:
        u = umax
    elif u < umin:
        u = umin
    # Publishing.
    pub.publish(u)
    rospy.loginfo(rospy.get_caller_id() + " Command : %s , at %s", u, rospy.get_rostime())

def main():
    # Node initialization.
    rospy.init_node('tracker_controller', log_level=rospy.DEBUG, anonymous = True)
    rospy.loginfo("tracker_controller starts.")
    # Subscription.
    rospy.Subscriber("bearing_error", Float64, callback_bearing)
    # Publisher
    global pub
    pub = rospy.Publisher("bearing/command", Float64, queue_size = 1) # bearing command.
    # Publishing.
    # TODO : commands
    # Spinning.
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
