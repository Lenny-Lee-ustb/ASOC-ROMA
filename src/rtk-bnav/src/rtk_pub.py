#!/usr/bin/env python3

# a script for GNSS-INS's INSPVAXA header. Convert to odometry format and pub. An example is shown following:
# "#INSPVAXA,COM3,0,93.9,FINESTEERING,2179,25487.000,00000000,03e2,757;
#  INS_ALIGNMENT_COMPLETE,INS_RTKFIXED,
# 22.60481305778,113.99030218678,44.5478,-3.3822, #lon lat height undulation
# 0.4616,0.2494,0.0314, #north east up -velocity
# 97.733607103,32.939981889,133.649272773, #roll pitch yaw
# 0.0254,0.0258,0.0342, #var-lon lat height
# 0.0108,0.0130,0.0105, #var-north east up -velocity
# 0.0400,0.0476,1.1325, #var-roll pitch yaw
# 00000000,0*93c5919d"


from math import pi
import time
import rospy
import utm
from nmea_msgs.msg import Sentence
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Quaternion
from tf.transformations import quaternion_from_euler

_angle2rad = pi/180.0
# _rad2angle = 180.0/pi

tic = time.time()

def callback(data):
    global tic 
    datahead = data.header
    datastr = data.sentence
    str = datastr.split(',')
    if str[0] == '#INSPVAXA':
        # toc = time.time() 
        # rospy.loginfo("Hz:%.2f"%(1/(toc-tic))) # get frequency
        # tic = toc
        pose_rtk = Pose()
        # add position
        pos_lat = float(str[11])
        pos_lon = float(str[12])
        pos = utm.from_latlon(pos_lat, pos_lon)
        pose_rtk.position.x = pos[0] # North
        pose_rtk.position.y = pos[1] # East
        pose_rtk.position.z = float(str[13]) # Height
        # add orientation
        roll = float(str[18]) * _angle2rad
        pitch = float(str[19]) * _angle2rad
        yaw = float(str[20]) * _angle2rad
        quat = quaternion_from_euler(roll, pitch, yaw, axes='rxyz') # r p y
        pose_rtk.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3]) 
        # variance of position, angles
        # Var_lat = float(str[21]) + pos_lat
        # Var_Long = float(str[22]) +pos_lon
        # var_pos = utm.from_latlon(Var_lat, Var_Long)
        Var_Lat = float(str[21])
        Var_Long = float(str[22])
        Var_Height = float(str[23])
        Var_Roll = float(str[27]) * _angle2rad
        Var_Pitch = float(str[28]) * _angle2rad
        Var_Yaw = float(str[29]) * _angle2rad
        # covariance of pose
        pose_cov_rtk = [Var_Lat*Var_Lat, 0, 0, 0, 0, 0,
                        0, Var_Long*Var_Long, 0, 0, 0, 0,
                        0, 0, Var_Height*Var_Height, 0, 0, 0,
                        0, 0, 0, Var_Roll*Var_Roll, 0, 0,
                        0, 0, 0, 0, Var_Pitch*Var_Pitch, 0, 
                        0, 0, 0, 0, 0, Var_Yaw*Var_Yaw] # 6x6 x, y, z, roll, pitch, yaw

        twist_rtk = Twist()
        # add linear velocity
        twist_rtk.linear.x = float(str[15]) # north velocity
        twist_rtk.linear.y = float(str[16]) # east velocity
        twist_rtk.linear.z = float(str[17]) # up velocity
        # variance of linear velocity
        Var_vx = float(str[24])
        Var_vy = float(str[25])
        Var_vz = float(str[26])
        # covariance of twist
        twist_cov_rtk =[Var_vx*Var_vx, 0, 0, 0, 0, 0,
                        0, Var_vy*Var_vy, 0, 0, 0, 0,
                        0, 0, Var_vz*Var_vz, 0, 0, 0,
                        0, 0, 0, 999, 0, 0,
                        0, 0, 0, 0, 999, 0, 
                        0, 0, 0, 0, 0, 999] # 6x6 vx(not sure), vy(not sure), vz, v_roll, v_pitch, v_yaw

        if str[10] == 'INS_RTKFIXED':
            pub_data(pose_rtk, pose_cov_rtk,twist_rtk, twist_cov_rtk, datahead)
            # toc = time.time() 
            # rospy.loginfo("Hz:%.2f"%(1/(toc-tic))) # get frequency
            # tic = toc
            # print(data)
        else:
            rospy.logwarn("No RTK Signal !!")

def pub_data(pose, pose_cov, twist, twist_cov, dataheader):
    pub_rtk = Odometry()
    pub_rtk.header.stamp = dataheader.stamp
    pub_rtk.header.frame_id = 'odom'
    pub_rtk.child_frame_id = 'gps'

    pub_rtk.pose.pose.position = pose.position
    pub_rtk.pose.pose.orientation = pose.orientation
    pub_rtk.pose.covariance = pose_cov
    
    pub_rtk.twist.twist = twist
    pub_rtk.twist.covariance = twist_cov
    pub.publish(pub_rtk)

if __name__ == '__main__':
    try:
        rospy.init_node('rtk_publisher', anonymous=True)
        rospy.Subscriber('nmea_sentence', Sentence, callback)
        pub = rospy.Publisher('GPS_odom', Odometry, queue_size=5, latch=True)
        rospy.loginfo("start RTK node!")
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr('Could not start rtk node.')
