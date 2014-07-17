#!/usr/bin/python

import argparse
import dslam_ecto_bridge.dslam_image_bridge as image_bridge
import dslam_ecto_vision.dslam_vision_utils as vision_utils
#import dslam_ecto_bridge.beachhead as beachhead
import ecto
from ecto.opts import scheduler_options, run_plasm
from ecto_opencv.highgui import imshow, FPSDrawer, ImageSaver, ImageReader
import ecto_ros, ecto_ros.ecto_sensor_msgs as ecto_sensor_msgs
from ecto_ros import Mat2Image
import os
import rospy
import sys
import time
import subprocess

##############################################################################
# Arguments
##############################################################################

parser = argparse.ArgumentParser(description='Testies.')

# our local command line args
parser.add_argument('-a', '--average', default=False, action='store_true', help='do averaging on stereo pairs.')
parser.add_argument('-b', '--bag', default=False, action='store_true', help='Write to a ROS Bag.')
parser.add_argument('-d', '--dump', default=False, action='store_true', help='dump a continuous stream of images (use with -o).')
parser.add_argument('-f', '--fps', default=False, action='store_true', help='show fps etched on the image windows.')
parser.add_argument('-o', '--output-dir', action='store', default='./', help='default image storage location.')
parser.add_argument('-r', '--read', default=False, action='store_true', help='read from image store instead of stereo pairs.')
# add ecto scheduler args
scheduler_options(parser)
options = parser.parse_args()

##############################################################################
# Cells
##############################################################################

if not options.read:
    image_source = image_bridge.ImageClient('image_source', ip='192.168.1.3', port=5555)
    bit_bucket = vision_utils.BitBucket('bit_bucket', verbose=False)
if options.fps:
    fps_left = FPSDrawer()
    fps_right = FPSDrawer()

if options.average:
    average = vision_utils.AveragePair('average', window_size = 100)
    # keys are shared across all windows...i.e. hitting 'l' here in any opencv window will trigger this imshow_left_average event
    # ....assign unique keys for each window!
    imshow_left_average = imshow('show_average_left', name='average_left', triggers=dict(save=ord('s')))
    imshow_right_average = imshow('show_average_right', name='average_right')
    left_image_filename = os.path.join(options.output_dir, 'snapshots', 'left', 'image_%04d.bmp')
    right_image_filename = os.path.join(options.output_dir, 'snapshots', 'right', 'image_%04d.bmp')
    save_image_left = ecto.If('save_image_left', cell=ImageSaver('save_image_left', filename_format=left_image_filename, start=1))
    save_image_right = ecto.If('save_image_right', cell=ImageSaver('save_image_right', filename_format=right_image_filename, start=1))

# Raw image display
imshow_left = imshow('show_raw_left', name='raw_left')
imshow_right = imshow('show_raw_right', name='raw_right')

# Slowed image display for reading from file system instead of board
imshow_sleft = imshow(name='13ms wait raw left', waitKey=13)
imshow_sright = imshow(name='13ms wait raw right', waitKey=13)

# Dumping
if options.dump:
    save_left = ImageSaver('dump_left', filename_format=os.path.join(options.output_dir, 'dump', 'left', "image_%04d.bmp"))
    save_right = ImageSaver('dump_right', filename_format=os.path.join(options.output_dir, 'dump', 'right', "image_%04d.bmp"))
if options.dump and options.average:
    dump_average_left = ecto.If('dump_average_left', cell=ImageSaver('dump_average_left', filename_format=os.path.join(options.output_dir, 'dump', 'average_left', "image_%04d.bmp")))
    dump_average_right = ecto.If('dump_average_right', cell=ImageSaver('dump_average_right', filename_format=os.path.join(options.output_dir, 'dump', 'average_right', "image_%04d.bmp")))

# Reading from filesystem (Can be modified to loop)
if options.read:
    load_left = ImageReader(path=options.output_dir, match="^.*left.*\\.bmp$", loop=False)
    load_right = ImageReader(path=options.output_dir, match="^.*right.*\\.bmp$", loop=False)

# Converters
mat2ImgMsgLeft = Mat2Image()
mat2ImgMsgRight = Mat2Image()

# Publisher and Subscriber
ImagePub = ecto_sensor_msgs.Publisher_Image
ImageSub = ecto_sensor_msgs.Subscriber_Image
pub_imageLeft = ImagePub("image_publisher", topic_name='/image_left')
pub_imageRight = ImagePub("image_publisher", topic_name='/image_right')

##############################################################################
# Graph
##############################################################################

graph = []  # graph cannot be empty!

if options.fps:
    graph += [
        image_source['left']  >> fps_left['image'],
        image_source['right'] >> fps_right['image'],
        fps_left['image']  >> imshow_left['image'],
        fps_right['image'] >> imshow_right['image'],
    ]

if options.average:
    graph += [
        image_source['pair'] >> average['pair'],
        average['left']  >> imshow_left_average['image'],
        average['right'] >> imshow_right_average['image'],
        imshow_left_average['save'] >> save_image_left['__test__'],
        imshow_left_average['save'] >> save_image_right['__test__'],  # can only have one imshow source of triggers for 's' and then it works worldwide.
        average['left'] >> save_image_left['image'],
        average['right'] >> save_image_right['image'],
    ]

if options.dump:
    graph += [
        image_source['left']  >> save_left['image'],
        image_source['right'] >> save_right['image'],
    ]

if options.dump and options.average:
     graph += [
         average['updated'] >> (dump_average_left['__test__'], dump_average_right['__test__']),
         average['left']  >> dump_average_left['image'],
         average['right'] >> dump_average_right['image'],
     ]

if options.read:
    graph += [
        load_left['image']  >> imshow_sleft['image'],
        load_right['image'] >> imshow_sright['image'],
    ]

if options.bag:
    graph += [
	image_source['left']  >> imshow_left['image'],
        image_source['right'] >> imshow_right['image'],
        image_source['left'] >> mat2ImgMsgLeft['image'],
        image_source['right'] >> mat2ImgMsgRight['image'],
        mat2ImgMsgLeft['image']  >> pub_imageLeft['input'],
        mat2ImgMsgRight['image'] >> pub_imageRight['input'],
	subprocess.Popen("rosbag record -q /odom /image_left /image_right", shell=True)
    ]
#else:
#    graph += [
#        image_source['counter'] >> bit_bucket['counter'],
#        image_source['left']  >> imshow_left['image'],
#        image_source['right'] >> imshow_right['image'],
#    ]


plasm = ecto.Plasm()
plasm.connect(graph)


##############################################################################
# View Plasm
##############################################################################

# This blocks execution and same as run_plasm's --gui
#ecto.view_plasm(plasm)

##############################################################################
# Ros
##############################################################################

ecto_ros.init(sys.argv, "dslam_ecto_bridge")
#rospy.init_node("dslam_ecto_bridge")

##############################################################################
# Run plasm
##############################################################################

# some custom hacking

try:
    run_plasm(options, plasm, locals=vars())
except KeyboardInterrupt:
    pass

##############################################################################
# Manual Executors
##############################################################################

#sched = ecto.Scheduler(plasm)

#try:
    #sched.execute_async(niter=15)
    #sched.run()
    #sched.execute(niter=15)
    #stats = sched.stats()
    #print stats

#while(True):
#     time.sleep(0.2);
#     sched.execute(niter=1)

#while not rospy.is_shutdown():
#    try:
#        time.sleep(0.2);
#    except Exception as e:
#    except KeyboardInterrupt as e:
#        print "Caught keyboard interrupt " 
#        break
#    print("Dude")
    #sched.execute(niter=1)

#except KeyboardInterrupt:
#    pass
