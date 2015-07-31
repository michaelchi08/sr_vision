#!/usr/bin/env python

import os
import unittest
import rospkg
import rosbag
import numpy as np

from sr_object_segmentation.shape_color_segmentation import \
    ShapeColorSegmentation
from sr_object_tracking.utils import Utils

PKG = 'sr_object_segmentation'


class TestSegmentation(unittest.TestCase):
    """ Test class for color/shape segmentation algorithm """

    def setUp(self):
        """
        Initialization
        """
        image = self.bag_read_all('segmentation_test.bag')[0]
        color = 'red'
        shape = np.load(os.path.dirname(os.path.dirname(
            os.path.realpath(__file__))) + '/shapes/dataset/strawberry.npy')
        size = 2500
        utils = Utils(color, image)

        self.algo = ShapeColorSegmentation(color, shape, size, utils)
        self.algo.segmentation(image)

        self.nb_segments = self.algo.nb_segments
        self.segments = self.algo.segmented_box

    # Util methods
    ###############

    def bag_read_all(self, f):
        """Read all messages from the given bag file. Path is relative to
        test dir."""
        rp = rospkg.RosPack()
        bag_file = os.path.join(rp.get_path(PKG), 'test', f)
        bag = rosbag.Bag(bag_file)
        msgs = []
        for topic, msg, t in bag.read_messages():
            msgs.append(msg)
        return msgs

    def wellLocated(self, seg, theo):
        t = 50
        x, y, w, h = seg
        c_x, c_y = int(x + w / 2), int(y + h / 2)
        well_located = c_x in xrange(theo[0] - t,
                                     theo[0] + t) and c_y in xrange(
            theo[1] - t, theo[1] + t)
        self.assertTrue(well_located)
        return well_located

    def wellSized(self, seg, theo):
        t = 50
        _, _, w, h = seg
        well_sized = w in xrange(theo[0] - t,
                                 theo[0] + t) and h in xrange(
            theo[1] - t, theo[1] + t)
        return well_sized

    # Tests
    ########

    def test_not_empty(self):
        """
        Verify that the number of segmented objects is not null
        """
        self.assertNotEqual(self.nb_segments, 0)

    def test_centroid(self):
        """
        Verify that the segmented objects are well-located
        """
        self.wellLocated(self.segments[0], (525, 335))
        self.wellLocated(self.segments[1], (225, 245))

    def test_size(self):
        """
        Verify that the segmented objects size is correct
        """
        self.wellSized(self.segments[0], (95, 125))
        self.wellSized(self.segments[1], (120, 145))


if __name__ == '__main__':
    import rosunit

    rosunit.unitrun(PKG, 'test_segmentation', TestSegmentation)
