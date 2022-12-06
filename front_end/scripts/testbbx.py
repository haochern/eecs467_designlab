import BoundingBox
import iou
import utils
import numpy as np

bbx1_cor = np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]])
bbx2_cor= np.array([[0,0,0],[0.5,0,0],[0.5,0.5,0],[0,0.5,0],[0,0,1],[-0.5,0,1],[-0.5,-0.5,1],[0,-0.5,1]])

bbx1=BoundingBox.BoundingBox(bbx1_cor,1)
bbx2=BoundingBox.BoundingBox(bbx2_cor,2)

IOUU= iou.IoU(bbx1,bbx2)
iouu= IOUU.iou()
print(iou)