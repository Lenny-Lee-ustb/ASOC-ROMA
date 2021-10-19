import rospy
from std_msgs.msg import String
from rt500.msg import RTK


def callback(data):
    global x
    global y
    global z
    global roll
    global pitch
    global yaw
    
    str = data.split(',')
    while str[0] != '':
        if str[0] != '#INSPVAXA':
            continue
        roll = float(str[18])
        pitch = float(str[19])
        yaw = float(str[20])
        x = float(str[11])
        y = float(str[12])
        z = float(str[13])




if __name__ == '__main__':
    try:
        rospy.init_node('rtk_publisher', anonymous=True)
        rospy.Subscriber('nmea_sentence', String, callback)
        pub = rospy.Publisher('rtk', RTK, queue_size=1, latch=True)
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            rtk_msg = RTK()
            rtk_msg.lat = x
            rtk_msg.log = y
            rtk_msg.alt = z
            rtk_msg.roll = roll
            rtk_msg.pitch = pitch
            rtk_msg.yaw = yaw
            
            pub.publish(rtk_msg)
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.logerr('Could not start rtk node.')



