#!/usr/bin/env python
import csv
import os

import rospy
import tf
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path

CSV_HEADER = ['xr', 'yr', 'thetar', 'kappar'] # data format

class Path_loader(object):
	def __init__(self):
		rospy.init_node('path_publisher', log_level=rospy.DEBUG)
		self.rate = rospy.Rate(3) # 3hz
		self.pub = rospy.Publisher('/fix_path', Path, queue_size=1, latch=True)
		self.new_path_loader('src/path_load/src/data.csv')
		
		rospy.spin()

	def new_path_loader(self, file_path):
		if os.path.isfile(file_path):
			paths = self.load_paths(file_path)
			rospy.loginfo('Path Loaded')

			while not rospy.is_shutdown():
				self.publish(paths)
				self.rate.sleep()
			
		else:
			rospy.logerr('%s is not a file', file_path)

	def quaternion_from_yaw(self, yaw):
		return tf.transformations.quaternion_from_euler(0., 0., yaw)

	def load_paths(self, fname):
		paths = Path()
		index =0
		with open(fname) as wfile:
			reader = csv.DictReader(wfile, CSV_HEADER)
			for wp in reader:
				p = PoseStamped()
				p.header.seq = index
				index =index +1
				p.header.stamp = rospy.Time.now()
				p.header.frame_id = '/world'
				p.pose.position.x = float(wp['xr'])
				p.pose.position.y = float(wp['yr'])
				p.pose.position.z = float(wp['kappar'])
				q = self.quaternion_from_yaw(float(wp['thetar']))
				p.pose.orientation = Quaternion(*q)
				paths.poses.append(p)
		return paths


	def publish(self,paths):
		pub_path = Path()
		pub_path.header.frame_id = '/world'
		pub_path.header.stamp = rospy.Time.now()
		# pub_path.poses.header.frame_id='/world'
		# pub_path.poses.header.stamp=rospy.Time.now()
		pub_path.poses = paths.poses
		self.pub.publish(pub_path)


if __name__ == '__main__':
    try:
        Path_loader()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint node.')
