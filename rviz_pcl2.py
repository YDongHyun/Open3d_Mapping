#!/usr/bin/env python3
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from PIL import Image 
import cv2
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d

def Pcl_pub():
   pcl_pub = rospy.Publisher('/pcl', PointCloud2, queue_size=10)
   color_raw = o3d.io.read_image("image480/00020.jpg")
   depth_raw = o3d.io.read_image("depth/00020.png")
   rgbd_image = o3d.geometry.RGBDImage.create_from_tum_format(
      color_raw, depth_raw, convert_rgb_to_intensity=False)

   color1 = np.array([[0.12, 0.8, 0.8],[0.13, 0.9, 0.9]])
   color2 = np.array([[0., 0.5, 0.2],[0.1,0.6,0.3]])
   color3 = np.array([[0.6,0.7,0.2],[0.9,0.8,0.4]])

   pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
      rgbd_image,
      o3d.camera.PinholeCameraIntrinsic(
         o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
   pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
 
   xyz_1=[]
   color_1=[]
   color= np.asarray(pcd.colors)
   xyz = np.asarray(pcd.points)

   for i in range(len(color)):
      if ((color[i]>=color1[0]) & (color[i]<=color1[1])).min():
         xyz_1.append(xyz[i])
         color_1.append(color[i])
      elif ((color[i]>=color2[0]) & (color[i]<=color2[1])).min():
         xyz_1.append(xyz[i])
         color_1.append(color[i])
      elif ((color[i]>=color3[0]) & (color[i]<=color3[1])).min():
         xyz_1.append(xyz[i])
         color_1.append(color[i])

   points=[]
   ret=np.array(xyz_1,dtype=np.float32)
   header = Header()
   header.frame_id = "map"
   header.stamp = rospy.Time.now()
   fields = [PointField('x', 0, PointField.FLOAT32, 1),
      PointField('y', 4, PointField.FLOAT32, 1),
      PointField('z', 8, PointField.FLOAT32, 1),
      #PointField('rgb', 12, PointField.UINT32, 1),
      #PointField('rgba', 12, PointField.UINT32, 1),
      ]
   print(xyz[0])
   points=xyz_1
   pc2 = point_cloud2.create_cloud(header, fields, points)

   pcl_pub.publish(pc2)

   print("sub")

if __name__ == '__main__':      
   rospy.init_node('pcl_pub',anonymous=True)
        
   try:
      Pcl_pub()
      
   except rospy.ROSInterruptException:
      pass
