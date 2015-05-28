#!/usr/bin/env python

import SimpleCV as sCV
from sr_object_segmentation import SrObjectSegmentation


class FeatureRecognition(SrObjectSegmentation):
    """
    Segmentation based upon the search of a specified feature, with a SimpleCV algorithm
    """

    def __init__(self, image, feature):
        """
        Initialize the feature recognition object
        @param image - the image to search
        @param feature - the feature to search the image for
        """
        SrObjectSegmentation.__init__(self, image, {})
        self.name = 'Finding feature algorithm'
        self.feature = sCV.Image('/home/glassbot/Desktop/strawberry.jpg')
        self.points = self.segmentation()
        self.nb_segments = len(self.points)

    def segmentation(self):
        """
        Segment the image searching the feature given as parameter
        @return - dictionary of the feature points coordinates
        """

        methods = ["SQR_DIFF", "SQR_DIFF_NORM", "CCOEFF", "CCOEFF_NORM", "CCORR",
                   "CCORR_NORM"]  # the various types of template matching available
        t = 5  # Threshold
        found = False
        matches = []
        while not found and t > 0:
            for m in methods:
                print "current method:", m  # print the method being used
                result = sCV.Image('/home/glassbot/Desktop/strawberry_table.jpg', sample=True)
                dl = sCV.DrawingLayer((source.width, source.height))
                temp = source.findTemplate(template, threshold=t, method=m)
                if len(temp) != 0:
                    matches.append((temp, m))
                    found = True
            t -= 1

        if matches:
            dic = {}
            match = matches[0][0]
            points = []
            xmin, ymin = match.x() - match.width() / 2, match.y() - match.height() / 2
            xmax, ymax = match.x() + match.width() / 2, match.y() + match.height() / 2
            for x in range(xmin, xmax):
                for y in range(ymin, ymax):
                    points.append((x, y))
            dic[0] = points

            # Sort by descending size of segments
            seg_by_length = sorted(dic.values(), key=len, reverse=True)

            for i in range(len(dic)):
                dic[i] = seg_by_length[i]

            return dic
        else:
            print 'Feature not found'


source = sCV.Image('/home/glassbot/Desktop/strawberry_table.jpg')
template = sCV.Image('/home/glassbot/Desktop/strawberry.jpg')

fr = FeatureRecognition(source, template)
