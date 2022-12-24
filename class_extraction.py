import struct
import open3d as o3d
import numpy as np
import cv2
import matplotlib.pyplot as plt

img= cv2.imread("image/00005.png")
dep = cv2.imread("depth/00005.png")

def class_mask(img):
    img=cv2.resize(img,(848,480))
    class_mask = np.copy(img)
    thresholds = (img[:,:,0] >250) & (img[:,:,1] >250) &(img[:,:,1]>250) # 기준 색상 필터링
    class_mask[thresholds] = [0]
    return class_mask

def depth_mask(img,dep):
    depth_mask = np.copy(dep)
    thresholds = (img[:,:,0] ==0) & (img[:,:,1] ==0) &(img[:,:,1]==0)
    depth_mask[thresholds] = [0]
    return depth_mask

class_img=class_mask(img)
depth_img=depth_mask(class_img,dep)

cv2.imshow("test",class_img)
cv2.waitKey()

class_img=o3d.geometry.Image(class_img.astype(np.uint8))
depth_img = o3d.geometry.Image(depth_img.astype(np.uint8))

rgbd_image = o3d.geometry.RGBDImage.create_from_tum_format(
    class_img, depth_img, convert_rgb_to_intensity=False)

plt.subplot(1, 2, 1)
plt.title('SUN grayscale image')
plt.imshow(class_img)
plt.subplot(1, 2, 2)
plt.title('SUN depth image')
plt.imshow(class_img)
plt.show()

pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
      rgbd_image,
      o3d.camera.PinholeCameraIntrinsic(
         o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
o3d.visualization.draw_geometries([pcd])
