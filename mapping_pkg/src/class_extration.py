import struct
import open3d as o3d
import numpy as np
import cv2
import matplotlib.pyplot as plt

def class_mask(img):
    #img=cv2.resize(img,(848,480))
    class_mask = np.copy(img)
    thresholds = (img[:,:,0] >250) & (img[:,:,1] >250) &(img[:,:,1]>250) 
    class_mask[thresholds] = [0]
    return class_mask

def depth_mask(img,dep):
    depth_mask = np.copy(dep)
    thresholds = (img[:,:,0] ==0) & (img[:,:,1] ==0) &(img[:,:,1]==0)
    depth_mask[thresholds] = [0]
    return depth_mask

def draw_poly(img):
    gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    contours, hierarchy= cv2.findContours(gray, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    for i in range(len(contours)):

        img = cv2.drawContours(img, [contours[i]], 0, (0,255,0), 2)
        cv2.imshow("tset",img)
    return img
    
def class_extract(img,dep):
    img=cv2.resize(img,(848,480))
    dep=cv2.resize(dep,(848,480))

    class_img=class_mask(img)

    depth_img=depth_mask(class_img,dep)
    class_img = cv2.cvtColor(class_img, cv2.COLOR_BGR2RGB)
    depth_img = cv2.cvtColor(depth_img, cv2.COLOR_BGR2RGB)
    #class_img=cv2.cvtColor(class_img,cv2.COLOR_BGR2HSV)
    #class_img=class_img*100/255
    #img=draw_poly(class_img)

    class_img=o3d.geometry.Image(class_img)
    depth_img = o3d.geometry.Image(depth_img)

    rgbd_image = o3d.geometry.RGBDImage.create_from_tum_format(
         class_img, depth_img, convert_rgb_to_intensity=False)
    print(rgbd_image)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
       rgbd_image,
       o3d.camera.PinholeCameraIntrinsic(
          o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    #o3d.visualization.draw_geometries([pcd])
    return pcd
