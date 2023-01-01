import struct
import open3d as o3d
import numpy as np
import time

def quaternion_rotation_matrix(Q):
   # Extract the values from Q
   q0 = Q[0]
   q1 = Q[1]
   q2 = Q[2]
   q3 = Q[3]
   # First row of the rotation matrix
   r00 = 2 * (q0 * q0 + q1 * q1) - 1
   r01 = 2 * (q1 * q2 - q0 * q3)
   r02 = 2 * (q1 * q3 + q0 * q2)
   # Second row of the rotation matrix
   r10 = 2 * (q1 * q2 + q0 * q3)
   r11 = 2 * (q0 * q0 + q2 * q2) - 1
   r12 = 2 * (q2 * q3 - q0 * q1)

   # Third row of the rotation matrix
   r20 = 2 * (q1 * q3 - q0 * q2)
   r21 = 2 * (q2 * q3 + q0 * q1)
   r22 = 2 * (q0 * q0 + q3 * q3) - 1

   # 3x3 rotation matrix
   rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
   return rot_matrix

def img_to_pcl(color_raw,depth_raw,pose,Q):
   qu=quaternion_rotation_matrix(Q)
   rgbd_image = o3d.geometry.RGBDImage.create_from_tum_format(
      color_raw, depth_raw, convert_rgb_to_intensity=False)
   #pcd =class_extract(color_raw,depth_raw)
   color1 = np.array([[0.19, 0.8, 0.9],[0.2, 0.9, 1.1]])
   color2 = np.array([[0., 0.5, 0.2],[0.1,0.6,0.3]])
   color3 = np.array([[0.6,0.7,0.2],[0.9,0.8,0.4]])

   pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
      rgbd_image,
      o3d.camera.PinholeCameraIntrinsic(
         o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
   pcd = pcd.voxel_down_sample(voxel_size=0.03)
   pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
   print(pcd)
   xyz_1=[]
   color= np.asarray(pcd.colors)
   xyz = np.asarray(pcd.points)
   start = time.time()
   
   transpose=np.array([[qu[0][0],qu[0][1],qu[0][2],pose[0]],
               [qu[1][0],qu[1][1], qu[1][2],pose[1]],
               [qu[2][0], qu[2][1],qu[2][2],pose[2]],[0,0,0,1]])             
               
   for i in range(len(color)):
      if ((color[i]>=color1[0]) & (color[i]<=color1[1])).min():
         r, g, b = int(color[i][0]*255), int(color[i][1]*255), int(color[i][2]*255)
         rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]
         
         before_point=np.array([[xyz[i][0]],[xyz[i][1]],[xyz[i][2]],[1]])
         point=transpose@before_point
         xyz_1.append([point[0],point[1],point[2],rgb])

      elif ((color[i]>=color2[0]) & (color[i]<=color2[1])).min():
         r, g, b = int(color[i][0]*255), int(color[i][1]*255), int(color[i][2]*255)
         rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]
         before_point=np.array([[xyz[i][0]],[xyz[i][1]],[xyz[i][2]],[1]])
         point=transpose@before_point
         xyz_1.append([point[0],point[1],point[2],rgb])

      elif ((color[i]>=color3[0]) & (color[i]<=color3[1])).min():
         r, g, b = int(color[i][0]*255), int(color[i][1]*255), int(color[i][2]*255)
         rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]
         before_point=np.array([[xyz[i][0]],[xyz[i][1]],[xyz[i][2]],[1]])
         point=transpose@before_point
         xyz_1.append([point[0],point[1],point[2],rgb])
      
         
   print("time :", time.time() - start)
   return xyz_1
