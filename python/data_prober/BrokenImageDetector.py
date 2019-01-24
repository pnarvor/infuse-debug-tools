import os
import itertools
import numpy as np
from scipy.misc import imread
from scipy.signal import medfilt
from scipy.ndimage.filters import convolve
import matplotlib.pyplot as plt

class BrokenImageDetector:
   
    def __init__(self):
         self.kernel = np.array([[-1.], \
                                 [ 1.]])
         self.median_kernel_size = 3

    def detect(self, path):
    
         files = os.listdir(path)
         filenames = []
         for filename in files:
             if filename.endswith(".pgm"):
                 filenames.append(filename)
         filenames.sort()
        
         score_list = []
         print("computing scores :")
         for filename in filenames:
         # for filename in itertools.islice(filenames, 500, 1000):
         # for filename in itertools.islice(filenames, 491, 492):
             print("    " + filename)
             img = imread(path + filename, mode='F')
             score_list.append(self.compute_score(img))
   
         scores = np.array(score_list)
         # return scores - medfilt(scores, kernel_size = self.median_kernel_size)
         return scores

    def compute_score(self, img):

        img_left = img[:,0:img.shape[1] / 2]
        edge_left = self.edge_detector(img_left)
        score_left = self.thresholding(edge_left)

        img_right = img[:,img.shape[1] / 2:]
        edge_right = self.edge_detector(img_right)
        score_right = self.thresholding(edge_right)

        # fig, axes = plt.subplots(2,1, sharex=False, sharey=False)
        # axes[0].plot(edge_left)
        # axes[0].grid()
        # axes[1].plot(edge_right)
        # axes[1].grid()

        # fig, axes = plt.subplots(1,2, sharex=False, sharey=False)
        # axes[0].imshow(img_left)
        # axes[1].imshow(img_right)

        return max([score_left, score_right])

    def edge_detector(self, img):

        gradient_img = convolve(img, self.kernel)
        gradient = np.sum(np.absolute(gradient_img), axis=1)
        score = gradient - medfilt(gradient, kernel_size = self.median_kernel_size)

        return score

    def thresholding(self, edge_detection):
        max_value = np.amax(edge_detection) - np.mean(edge_detection)

        return max_value
