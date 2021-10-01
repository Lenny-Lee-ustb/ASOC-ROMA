#!/usr/bin/env python3

import rospy
import serial
import math
from geometry_msgs.msg import Pose
 
QuarData=[0.0]*4
PosData=[0.0]*3        
FrameState = 0          
Bytenum = 0           
CheckSum = 0           


def DueData(inputdata):  
    global  FrameState  
    global  Bytenum
    global  CheckSum
    global  PosData
    global  QuarData
    

    for data in inputdata:
        if inputdata[data] == 0x23 & inputdata[data+1] == 0x42:
            data1 = data
        if inputdata[data] == 0x23 & inputdata[data+1] == 0x48:
            data2 = data
        if inputdata[data] == 0x24 & inputdata[data+1] == 0x47 & inputdata[data+2] == 0x50 & inputdata[data+3] == 0x47:
            data3 = data

        c1 = [0]*29
        c2 = [0]*25
        m = 0
        n = 0

        for i in range(data1,data2-1):
            if inputdata[i] == 0x2C:
                c1[m] = i
                m = m + 1
        
        for j in range(data2,data3-1):
            if inputdata[j] == 0x2c:
                c2[n] = j
                n = n + 1

        lat = inputdata[c1[11]+1:c1[12]-1]




 
def get_pos(datahex):                                 
    rxl = datahex[0]                                        
    rxh = datahex[1]
    ryl = datahex[2]                                        
    ryh = datahex[3]
    rzl = datahex[4]                                        
    rzh = datahex[5]
    k_angle = 180.0
 
    angle_x = (rxh << 8 | rxl) / 32768.0 * k_angle
    angle_y = (ryh << 8 | ryl) / 32768.0 * k_angle
    angle_z = (rzh << 8 | rzl) / 32768.0 * k_angle
    if angle_x >= k_angle:
        angle_x -= 2 * k_angle
    if angle_y >= k_angle:
        angle_y -= 2 * k_angle
    if angle_z >=k_angle:
        angle_z-= 2 * k_angle
    angle = [0.0]*3
    return angle_x,angle_y,angle_z


def get_quar(datahex):
    roll = angle[0]
    pitch = angle[1]
    yaw = angle[2]
    
    q= Pose()
    q.orientation.w= cy * cp * cr + sy * sp * sr
    q.orientation.x = cy * cp * sr - sy * sp * cr
    q.orientation.y = sy * cp * sr + cy * sp * cr
    q.orientation.z = sy * cp * cr - cy * sp * sr
    return q


def pub_data(a,w,angle):
    pub_imu = Imu()
    pub_imu.header.frame_id = '/IMU_leg_'+ID
    pub_imu.header.stamp = rospy.Time.now()
    pub_imu.linear_acceleration.x = a[0]
    pub_imu.linear_acceleration.y = a[1]
    pub_imu.linear_acceleration.z = a[2]
    pub_imu.angular_velocity.x = w[0]
    pub_imu.angular_velocity.y = w[1]
    pub_imu.angular_velocity.z = w[2]
    pub_imu.orientation = RPY2Quar(angle).orientation
    pub.publish(pub_imu)

 
if __name__=='__main__': 
    try:
        print(rospy.get_param_names())
        ID = str(rospy.get_param('ID'))
        port =rospy.get_param('imu_'+ID+'/serialport_name')
        baud =rospy.get_param('imu_'+ID+'/baudrate')
        # change into parameter later

        node_name = 'imu_leg_'+ID
        rospy.init_node(node_name, log_level=rospy.DEBUG)
        
        ser = serial.Serial(port, baud, timeout=0.5)
        pub = rospy.Publisher('/imu_data_leg_'+ID, Imu, queue_size=1, latch=True)
        
        print(port,ID)
        print(str(ser.is_open)+'_'+ID)

        while not rospy.is_shutdown():
            datahex = ser.read(33)
            DueData(datahex)
    
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start Imu node.')
