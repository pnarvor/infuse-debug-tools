#! /usr/bin/python3

import sys
import os
sys.path.append('../')
from multiprocessing import Process
import argparse

from data_prober import BrokenImageDetector

def compute(source_directory):

    
    if not os.path.isdir(source_directory):
        print("Source directory \"", source_directory,
              "\" does not exists. Aborting.")
        return 

    print("Computing scores for images in \"", source_directory, "\"")
    
    bid = BrokenImageDetector()
    bid.detect(os.path.join(source_directory,"data/"))
    bid.write_score_file(source_directory)
    

parser = argparse.ArgumentParser()
parser.add_argument("source_directory", type=str,
                    help="Where to look for raw images \"*_cam\" folders")
parser.add_argument("-a", "--all-cams", action='store_true',
                    help="Compute score fro all cameras")
parser.add_argument("-n", "--navcam", action='store_true',
                    help="Compute score for navcam")
parser.add_argument("-f", "--frontcam", action='store_true',
                    help="Compute score for frontcam")
parser.add_argument("-r", "--rearcam", action='store_true',
                    help="Compute score for rearcam")
args = parser.parse_args()

pNavLeft = Process(target=compute,
                   args=(os.path.join(args.source_directory, "nav_cam/left/"),))
pNavRight = Process(target=compute,
                    args=(os.path.join(args.source_directory, "nav_cam/right/"),))
pFrontLeft = Process(target=compute,
                     args=(os.path.join(args.source_directory, "front_cam/left/"),))
pFrontRight = Process(target=compute,
                      args=(os.path.join(args.source_directory, "front_cam/right/"),))
pRearLeft = Process(target=compute,
                    args=(os.path.join(args.source_directory, "rear_cam/left/"),))
pRearRight = Process(target=compute,
                     args=(os.path.join(args.source_directory, "rear_cam/right/"),))

if args.navcam or args.all_cams:
    pNavLeft.start()
    pNavRight.start()

if args.frontcam or args.all_cams:
    pFrontLeft.start()
    pFrontRight.start()

if args.rearcam or args.all_cams:
    pRearLeft.start()
    pRearRight.start()

if args.navcam or args.all_cams:
    pNavLeft.join()
    pNavRight.join()

if args.frontcam or args.all_cams:
    pFrontLeft.join()
    pFrontRight.join()

if args.rearcam or args.all_cams:
    pRearLeft.join()
    pRearRight.join()




