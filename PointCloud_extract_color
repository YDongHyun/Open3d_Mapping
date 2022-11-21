import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import laspy as lp

color_raw = o3d.io.read_image("image480/00020.jpg")
depth_raw = o3d.io.read_image("depth/00020.png")
rgbd_image = o3d.geometry.RGBDImage.create_from_tum_format(
    color_raw, depth_raw, convert_rgb_to_intensity=False)
print(rgbd_image)

plt.subplot(1, 2, 1)
plt.title('Redwood grayscale image')
plt.imshow(rgbd_image.color)
plt.subplot(1, 2, 2)
plt.title('Redwood depth image')
plt.imshow(rgbd_image.depth)
#plt.show()
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
print(color[18100:20100])
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

pcd1= o3d.geometry.PointCloud()
pcd1.points = o3d.utility.Vector3dVector(xyz_1)
pcd1.colors = o3d.utility.Vector3dVector(color_1)
 
o3d.visualization.draw_geometries([pcd1])
