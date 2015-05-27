#!/usr/bin/env python

import urllib
from PIL import Image
import tarfile
import os
import shutil
import numpy as np

from sr_benchmarking.drawing import ImagesTest


class Berkeley(ImagesTest):
    def __init__(self):
        """
        Initialize the Berkeley dataset object, downloading files if absent
        """
        ImagesTest.__init__(self)
        self.name = 'Berkeley'
        self.files_id = ['3096', '86000', '167062', '227092']
        try:
            os.listdir('sr_benchmarking/BSDS300/')
        except:
            self.download_images()
        for file_id in self.files_id:
            self.ref_seg.append(read_seg_file(file_id))
        self.pil_img = [Image.open('sr_benchmarking/BSDS300/' + img + '.jpg') for img in self.files_id]
        self.np_img = [np.array(self.pil_img[i]) for i in range(len(self.pil_img))]

    def download_images(self):
        """
        Download the dataset from the berkeley website
        """
        print "Downloading Berkeley's dataset..."

        # Download the archives if absent
        if not 'BSDS300-images.tgz' in os.listdir('.'):
            images_archive = urllib.URLopener()
            images_archive.retrieve("http://www.eecs.berkeley.edu/Research/Projects/CS/vision/bsds/BSDS300-images.tgz",
                                    "BSDS300-images.tgz")
        if not "BSDS300-human.tgz" in os.listdir('.'):
            hum_seg_archive = urllib.URLopener()
            hum_seg_archive.retrieve("http://www.eecs.berkeley.edu/Research/Projects/CS/vision/bsds/BSDS300-human.tgz",
                                     "BSDS300-human.tgz")

        try:
            os.listdir('sr_benchmarking/BSDS300/images/test/')
        except:
            tar_images = tarfile.open('BSDS300-images.tgz', 'r')
            for name in self.files_id:
                path = 'BSDS300/images/test/' + name + '.jpg'
                tar_images.extractall('sr_benchmarking/', members=[tar_images.getmember(path)])
                shutil.copy2('sr_benchmarking/BSDS300/images/test/' + name + '.jpg', 'sr_benchmarking/BSDS300/')
            shutil.rmtree('sr_benchmarking/BSDS300/images')

        try:
            os.listdir('sr_benchmarking/BSDS300/human/')
        except:
            tar_hum_seg = tarfile.open('BSDS300-human.tgz', 'r')
            tar_hum_seg.extractall('sr_benchmarking/', members=None)
            shutil.rmtree('sr_benchmarking/BSDS300/human/gray')

        hum_seg_id = 0
        hum_list = os.listdir('sr_benchmarking/BSDS300/human/color')
        for hum in hum_list:
            seg_list = os.listdir('sr_benchmarking/BSDS300/human/color/' + hum)
            for seg in seg_list:
                seg = seg[:-4]
                if seg in self.files_id:
                    shutil.copy2('sr_benchmarking/BSDS300/human/color/' + hum + '/' + seg + '.seg',
                                 'sr_benchmarking/BSDS300/' + seg + '_' + str(hum_seg_id) + '.seg')
                    hum_seg_id += 1

        shutil.rmtree('sr_benchmarking/BSDS300/human/')
        os.remove('BSDS300-images.tgz')
        os.remove('BSDS300-human.tgz')


def read_seg_file(img_id):
    """
    Read a segmentation .seg file to get the segments and points corresponding
    @param img_id - id of the image from the Berkeley dataset
    """
    files = open_seg_files(img_id)

    # Reading file (with the 11 lines header)
    lines = []
    for i, f in enumerate(files):
        lines.append([])
        for header in range(11):
            f.readline()
        for line in f:
            lines[i].append([int(x) for x in line.split()])
        f.close()

    dic_list = []
    for i in range(len(lines)):
        dic_list.append(make_dictionary(lines[i]))
    return dic_list


def make_dictionary(lines):
    """
    Translate the segmentation files into a dictionary with segments and points coordinates
    @param lines - Lines of the seg files, as a list for each human
    @return - Dictionary with segments id as keys and coordinates as values
    """
    dic = {}
    for line in lines:
        coordinates = []
        for y in range(line[2], line[3]):
            coordinates.append((line[1], y))
        dic[line[0]] = coordinates

    # Sort by descending size of segments
    seg_by_length = sorted(dic.values(), key=len, reverse=True)
    dic = {}
    for i in range(len(seg_by_length)):
        dic[i] = seg_by_length[i]
    return dic


def open_seg_files(img_id):
    """
    Open the segmentation files
    @param img_id - ID of the image, from the Berkeley's dataset
    @return - List of the files, opened for reading
    """
    files = []
    for i in range(25):
        path = 'sr_benchmarking/BSDS300/' + img_id + '_' + str(i) + '.seg'
        try:
            f = open(path, 'r')
        except IOError:
            pass
        else:
            files.append(f)
    return files
