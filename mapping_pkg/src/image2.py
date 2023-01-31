import os

import cv2 
import open3d as o3d

def dataload(cnt):
    depth=sorted(os.listdir('./depth'))
    path2 = './depth/'+depth[cnt]
    img_depth = o3d.io.read_image(path2)
    #img_depth = cv2.imread(path2)
    #img_depth=o3d.geometry.Image(img_depth)


    with open("image.txt",'r') as f:
        lines=f.readlines()
        out=lines[cnt].split()
        pose=[float(out[1]),float(out[2]),float(out[3])]
        Q=[float(out[7]),float(out[4]),float(out[5]),float(out[6])]

    return img_depth, pose, Q
