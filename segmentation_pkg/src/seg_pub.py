#!/usr/bin/env python3
import rospy
import numpy as np
from segmentation import seg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from image import dataload

cnt=0

def Seg_pub():
   global cnt
   cv_bridge = CvBridge()
   seg_pub = rospy.Publisher('/seg', Image, queue_size=10)
   depth_pub = rospy.Publisher('/depth', Image, queue_size=10)
   color_raw,depth_raw=dataload(cnt)
   seg_img=seg(color_raw)
   msg_color=cv_bridge.cv2_to_imgmsg(seg_img)
   msg_depth=cv_bridge.cv2_to_imgmsg(depth_raw)

   seg_pub.publish(msg_color)
   depth_pub.publish(msg_depth)
   #rospy.sleep(0.)
   cnt+=1
   print("sub")

if __name__ == '__main__':      
   rospy.init_node('seg_pub',anonymous=True)
        
   try:
      while True:
         Seg_pub()
      
   except rospy.ROSInterruptException:
      pass
