#!/usr/bin/env python3
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import numpy as np
from mapping2 import img_to_pcl
from image2 import dataload
import open3d as o3d
from cv_bridge import CvBridge
import cv2

cnt=0
cv_bridge = CvBridge()

def callback(data):
   global color
   color=cv_bridge.imgmsg_to_cv2(data)
   
def callback_depth(data):
   global depth
   global cnt
   depth_img=cv_bridge.imgmsg_to_cv2(data)
   color_img = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
   
   depth_img_=depth_img.astype(np.uint16)*255
   color_raw=o3d.geometry.Image(color_img)
   depth_raw=o3d.geometry.Image(depth_img_)

   pcl_pub = rospy.Publisher('/pcl', PointCloud2, queue_size=10)
   depth_,pose,Q=dataload(cnt)
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
   flag1=False
   flag2=False
   print("sub")
  
def Img_sub():
   rospy.Subscriber('/seg',Image, callback)
   rospy.Subscriber('/depth',Image, callback_depth)
   rospy.spin()

if __name__ == '__main__':      
   rospy.init_node('pcl_pub2',anonymous=True)
        
   try:
      Img_sub() 
      
   except rospy.ROSInterruptException:
      pass