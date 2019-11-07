#!/usr/bin/env python 
import rospy
import math
import numpy as np
# from geometry_msgs.msg import Pose, PoseArray
from visualization_msgs.msg import MarkerArray,Marker
# from std_msgs.msg import ColorRGBA
# import tf.transformations

class Tag:
    def __init__(self):
        self.tag_pub = rospy.Publisher('/cctag_rviz',Marker, queue_size = 2)
    
    def print_points(self):
        rate = rospy.Rate(10)
        
        # iterations = 0
        while not rospy.is_shutdown():
            marker = Marker()
            marker.header.frame_id = "/base"
            marker.header.stamp = rospy.Time()
            marker.ns = "primitive_surfaces"
            marker.id = 10
            marker.mesh_use_embedded_materials = True
            marker.type = marker.MESH_RESOURCE
            marker.action = marker.ADD
            marker.pose.position.x = 0
            marker.pose.position.y = 0
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 1
            marker.scale.y = 1
            marker.scale.z = 1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.mesh_resource = "file:///home/pratique/drone_course_data/CC_tag_detection/Submission/cctag.dae";
            
            self.tag_pub.publish(marker)
            rate.sleep()

def main():
    obj = Tag()
    rospy.init_node('my_node_name',anonymous=True)
    obj.print_points()

if __name__ == '__main__':
    main()