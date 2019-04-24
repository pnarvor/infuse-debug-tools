#! /usr/bin/python3

import os
import argparse
import subprocess
from shutil import copyfile
import sh
from io import StringIO
import signal

def list_data_files(path):

    config_file_paths = []
    rosbag_paths = []

    files = os.listdir(path)
    for f in os.listdir(path):
        if not os.path.isfile(os.path.join(path,f)):
            continue
        if os.path.splitext(f)[1] == ".bag":
            rosbag_paths.append(f)
        else:
            config_file_paths.append(f)
    
    return [config_file_paths, rosbag_paths]

def do_extract(source_dir, output_dir, rawdata_subdir="raw_data", suffix=""):

    if source_dir[-1] == '/':
        source_dir = source_dir[:-1]
    folder_name = os.path.split(source_dir)[1] + suffix
    output_dir = os.path.join(output_dir, folder_name)

    files = list_data_files(source_dir)
    bags = [os.path.join(source_dir, f) for f in files[1]]
    bags.sort()

    print("Creating output directory...", end=" ")
    if not os.path.isdir(output_dir):
        os.mkdir(output_dir)
    print("Done")

    print("Copying configuration files...", end=" ")
    for f in files[0]:
        copyfile(os.path.join(source_dir, f),
                 os.path.join(output_dir, f))
    print("Done")

    try:

        print("Executing infuse_data_extractor...")
        command = sh.Command("infuse_data_extractor")
        command = command.bake(_out="infuse_data_extractor_stdout.txt",
                               _err="infuse_data_extractor_stderr.txt")
        command("-a", "--velodyne-png", os.path.join(output_dir, rawdata_subdir), bags)
        print("Done")

        # print("Executing infuse_stereo_matching...")
        # command = sh.Command("infuse_stereo_matching")
        # command = command.bake(_out="infuse_stereo_matching_stdout.txt",
        #                        _err="infuse_stereo_matching_stderr.txt")
        # command("-n", "--nav_calibration_file_path", os.path.join(source_dir, "navcam-calibration.yaml"),
        #         os.path.join(output_dir, rawdata_subdir), bags)
        # print("Done")

        # print("Executing infuse_stereo_matching...")
        # command = sh.Command("infuse_stereo_matching")
        # command = command.bake(_out="infuse_stereo_matching_stdout.txt",
        #                        _err="infuse_stereo_matching_stderr.txt")
        # # command("-f", "--front_calibration_file_path", os.path.join(source_dir, "frontcam-calibration.yaml"),
        # #         "-r", "--rear_calibration_file_path",  os.path.join(source_dir, "rearcam-calibration.yaml"),
        # #         "-n", "--nav_calibration_file_path",  os.path.join(source_dir, "navcam-calibration.yaml"),
        # #         os.path.join(output_dir, rawdata_subdir), bags)
        # command("-f", "--front_calibration_file_path", os.path.join(source_dir, "frontcam-calibration.yaml"),
        #         "-r", "--rear_calibration_file_path",  os.path.join(source_dir, "rearcam-calibration.yaml"),
        #         os.path.join(output_dir, rawdata_subdir), bags)
        # print("Done")

    except Exception as e:
        print("Bag extraction failed : ", e, "\nSkipping.")

parser = argparse.ArgumentParser()
parser.add_argument("destination_directory", type=str,
                    help="Directory where raw_data will be written")
parser.add_argument("source_directories", type=str, nargs='+',
                    help="list of log_data_acquisition* folders")
parser.add_argument("-r","--rawdata-folder-name", type=str,
                    default="raw_data",
                    help="Name of subdirectory in which raw data will be saved")
args = parser.parse_args()

# folder_names = []
# for path in args.source_directories:
#     if path[-1] == '/':
#         path = path[:-1]
#     folder_names.append(os.path.split(path)[1])

print("Will now extract data from these folders :")
for d in args.source_directories:
    print(" - ", d)
input("Press enter to continue, or Ctrl-c to abord")

for d,i in zip(args.source_directories, range(len(args.source_directories))):
    print("Extracting ", str(i + 1), "/", str(len(args.source_directories)), " : ", d)
    do_extract(d, args.destination_directory,
               rawdata_subdir=args.rawdata_folder_name,
               suffix="_raw")

