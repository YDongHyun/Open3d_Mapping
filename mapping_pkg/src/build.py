#!/usr/bin/env python3
import rospy
import struct

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

def Map_pub():
    pub = rospy.Publisher('/map', PointCloud2, queue_size=10)
    points = []
    with open("/home/ydh/catkin_ws/src/mapping_pkg/src/test.txt", "r") as f:
        lines = f.readlines()
        for i,line in enumerate(lines):
            if (i>=142) & (i<=15669):
                cc = line.split()
                x = float(cc[0])
                y = float(cc[1])
                z = float(cc[2])
                r = int(cc[3])
                g = int(cc[4])
                b = int(cc[5])
                a = 255
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                pt = [x, y, z, rgb]
                points.append(pt)
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgba', 12, PointField.UINT32, 1),
            ]

    header = Header()
    header.frame_id = "map"
    header.stamp = rospy.Time.now()

    pc = point_cloud2.create_cloud(header, fields, points)
    rospy.sleep(0.5)
    pub.publish(pc)
    print("sub")


if __name__ == '__main__':      
   rospy.init_node("Map_pub",anonymous=True)
        
   try:
      Map_pub()
      
   except rospy.ROSInterruptException:
      pass