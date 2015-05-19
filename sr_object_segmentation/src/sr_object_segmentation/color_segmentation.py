#!/usr/bin/env python
import cv2
import numpy as np
#import SimpleCV as sCV

from sr_object_segmentation import *

class ColorSegmentation(SrObjectSegmentation):
    """
    Segmentation based upo a color segmentation
    """

    def __init__(self,color):
        """
        Initialize the color segmentation object with the color chosen to segmente as parameter
        @param color - name of the color (string) chosen to segmente the image
        """
        self.color=color
        sr_object_segmentation.__init__(self)


    def segmentation(self):
        """
        Segmente the image according to the color attribute
        @return - dictionnary of segments found with points coordinates
        """

        img=cv2.imread(self.img)

        # define the list of boundaries with this order: red,blue,yellow,gray
        boundaries = {
            'red':([17, 15, 100], [50, 56, 200]),
            'blue':([86, 31, 4], [220, 88, 50]),
            'yellow':([25, 146, 190], [62, 174, 250]),
            'gray':([103, 86, 65], [145, 133, 128])
        }

        pts=[]
        dic={}
        k=0

        # create NumPy arrays from the boundaries
        lower = np.array(boundaries[color][0], dtype = "uint8")
        upper = np.array(boundaries[color][1], dtype = "uint8")

        # find the colors within the specified boundaries and apply
        # the mask
        mask = cv2.inRange(img, lower, upper)
        #output = cv2.bitwise_and(img,img, mask = mask)

        # return a dictionnary with segment id and the coordinates of the points corresponding
        for y in range(len(mask)):
            x_seq=np.nonzero(mask[y])[0]
            while len(x_seq)!=0:
                for x in x_seq:
                    pts.append((x,y))
                break
            dic[k]=pts
            k+=1

        #Sort by descending size of segments
        seg_by_length=sorted(dic.values(),key=len,reverse=True)
        for i in range(len(dic)):
            dic[i]=seg_by_length[i][0]

        return dic


