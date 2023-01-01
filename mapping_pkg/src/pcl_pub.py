#!/usr/bin/env python3
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
from mapping import img_to_pcl
from image import dataload
import open3d as o3d

cnt=0

def Pcl_pub():
   global cnt
   pcl_pub = rospy.Publisher('/pcl', PointCloud2, queue_size=10)
   color_raw,depth_raw,pose,Q=dataload(cnt)
   points=img_to_pcl(color_raw,depth_raw,pose,Q)
   header = Header()
   header.seq = cnt
   print(cnt)
   header.frame_id = "map"
   header.stamp = rospy.Time.now()
   
   fields = [PointField('x', 0, PointField.FLOAT32, 1),
      PointField('y', 4, PointField.FLOAT32, 1),
      PointField('z', 8, PointField.FLOAT32, 1),
      PointField('rgba', 12, PointField.UINT32, 1),
      ]
   pc2 = point_cloud2.create_cloud(header, fields, points)
   rospy.sleep(0.5)
   pcl_pub.publish(pc2)
   cnt+=1
   print("sub")

if __name__ == '__main__':      
   rospy.init_node('pcl_pub',anonymous=True)
        
   try:
      while True:
         Pcl_pub()
      
   except rospy.ROSInterruptException:
      pass