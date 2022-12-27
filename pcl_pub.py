#!/usr/bin/env python3
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from PIL import Image 
import cv2
import numpy as np
import matplotlib.pyplot as plt
from mapping import img_to_pcl

def Pcl_pub():
   pcl_pub = rospy.Publisher('/pcl', PointCloud2, queue_size=10)
   points=img_to_pcl()
   header = Header()
   header.frame_id = "map"
   header.stamp = rospy.Time.now()
   fields = [PointField('x', 0, PointField.FLOAT32, 1),
      PointField('y', 4, PointField.FLOAT32, 1),
      PointField('z', 8, PointField.FLOAT32, 1),
      PointField('rgba', 12, PointField.UINT32, 1),
      ]
      
   pc2 = point_cloud2.create_cloud(header, fields, points)
   pcl_pub.publish(pc2)

   print("sub")

if __name__ == '__main__':      
   rospy.init_node('pcl_pub',anonymous=True)
        
   try:
      Pcl_pub()
      
   except rospy.ROSInterruptException:
      pass
