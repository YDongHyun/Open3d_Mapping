import os
import cv2 

def dataload(cnt):
    color=sorted(os.listdir('./image'))
    path1 = './image/'+color[cnt]
    img_color = cv2.imread(path1)
    #img_color = o3d.io.read_image(path1)

    depth=sorted(os.listdir('./depth'))
    path2 = './depth/'+depth[cnt]
    #img_depth = o3d.io.read_image(path2)
    img_depth = cv2.imread(path2)

    return img_color,img_depth
