import rospy
from std_msgs.msg import String

stop_left = False
slow_down_left = False
stop_right = False
slow_down_right = False
stop_fcws = False
slow_down_fcws = False

def left_callback(msg):
    global stop_left, slow_down_left
    if msg.data == "STOP":
        rospy.loginfo("Obstacles found on left: STOP")
        stop_left = True
        slow_down_left = False
    elif msg.data == "SLOW":
        rospy.loginfo("Obstacles found on left: SLOW DOWN")
        slow_down_left = True
        stop_left = False
    else:
        stop_left = False
        slow_down_left = False
    decide_stop_or_go()

def right_callback(msg):
    global stop_right, slow_down_right
    if msg.data == "STOP":
        rospy.loginfo("Obstacles found on right: STOP")
        stop_right = True
        slow_down_right = False
    elif msg.data == "SLOW":
        rospy.loginfo("Obstacles found on right: SLOW DOWN")
        slow_down_right = True
        stop_right = False
    else:
        stop_right = False
        slow_down_right = False
    decide_stop_or_go()

def fcws_callback(msg):
    global stop_fcws, slow_down_fcws
    if msg.data == "STOP":
        rospy.loginfo("Forward collision warning: STOP")
        stop_fcws = True
        slow_down_fcws = False
    elif msg.data == "SLOW":
        rospy.loginfo("Forward collision warning: SLOW DOWN")
        slow_down_fcws = True
        stop_fcws = False
    else:
        slow_down_fcws = False
        stop_fcws = False
    decide_stop_or_go()

def decide_stop_or_go():
    global stop_left, stop_right, stop_fcws, slow_down_left, slow_down_right, slow_down_fcws
    rospy.loginfo(f"Deciding... stop_left: {stop_left}, stop_right: {stop_right}, stop_fcws: {stop_fcws}, slow_down_left: {slow_down_left}, slow_down_right: {slow_down_right}, slow_down_fcws: {slow_down_fcws}")
    if stop_left or stop_right or stop_fcws:
        rospy.loginfo("Publishing STOP")
        stop_pub.publish("STOP")
    elif slow_down_left or slow_down_right or slow_down_fcws:
        rospy.loginfo("Publishing SLOW")
        stop_pub.publish("SLOW")
    else:
        rospy.loginfo("Publishing GO")
        stop_pub.publish("GO")

def main():
    rospy.init_node('stop_go_controller')

    rospy.Subscriber('/srr_left_commands', String, left_callback)
    rospy.Subscriber('/srr_right_commands', String, right_callback)
    rospy.Subscriber('/fcws_commands', String, fcws_callback)

    global stop_pub
    stop_pub = rospy.Publisher('/vehicle_commands', String, queue_size=100)

    rospy.spin()

if __name__ == '__main__':
    main()
