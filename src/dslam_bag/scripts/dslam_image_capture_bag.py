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
parser.add_argument('-4', '--ip4', default=False, action='store_true', help='Use camera on IP 4')
parser.add_argument('-3', '--ip3', default=False, action='store_true', help='Use camera on IP 3')
# add ecto scheduler args
scheduler_options(parser)
#myargs = rospy.myargv(argv=sys.argv)
options = parser.parse_args()

##############################################################################
# Cells
##############################################################################

if not options.read:
    image_source4 = image_bridge.ImageClient('image_source', ip='192.168.1.4', port=5555)
    image_source3 = image_bridge.ImageClient('image_source', ip='192.168.1.3', port=5555)
    bit_bucket4 = vision_utils.BitBucket('bit_bucket', verbose=False)
    bit_bucket3 = vision_utils.BitBucket('bit_bucket', verbose=False)
if options.fps:
    if options.ip3:
        fps_left3 = FPSDrawer()
        fps_right3 = FPSDrawer()
    if options.ip4:
        fps_left4 = FPSDrawer()
        fps_right4 = FPSDrawer()


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
imshow_left4 = imshow('show_raw_left', name='raw_left4')
imshow_right4 = imshow('show_raw_right', name='raw_right4')
imshow_left3 = imshow('show_raw_left', name='raw_left3')
imshow_right3 = imshow('show_raw_right', name='raw_right3')

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
mat2ImgMsgLeft4 = Mat2Image()
mat2ImgMsgRight4 = Mat2Image()
mat2ImgMsgLeft3 = Mat2Image()
mat2ImgMsgRight3 = Mat2Image()

# Publisher
ImagePub = ecto_sensor_msgs.Publisher_Image
pub_imageLeft3 = ImagePub("image_publisher", topic_name='/image_left3')
pub_imageRight3 = ImagePub("image_publisher", topic_name='/image_right3')
pub_imageLeft4 = ImagePub("image_publisher", topic_name='/image_left4')
pub_imageRight4 = ImagePub("image_publisher", topic_name='/image_right4')

##############################################################################
# Graph
##############################################################################

graph = []  # graph cannot be empty!

if options.fps:
    if options.ip3:    
        graph += [
            image_source3['left']  >> fps_left3['image'],
            image_source3['right'] >> fps_right3['image'],
            fps_left3['image']  >> imshow_left3['image'],
            fps_right3['image'] >> imshow_right3['image'],
        ]
    if options.ip4:
        graph += [
            image_source4['left']  >> fps_left4['image'],
            image_source4['right'] >> fps_right4['image'],
            fps_left4['image']  >> imshow_left4['image'],
            fps_right4['image'] >> imshow_right4['image'],
         ]

if options.average:
    if options.ip3:
        graph += [
            image_source3['pair'] >> average['pair'],
            average['left']  >> imshow_left_average['image'],
            average['right'] >> imshow_right_average['image'],
            imshow_left_average['save'] >> save_image_left['__test__'],
            imshow_left_average['save'] >> save_image_right['__test__'],  # can only have one imshow source of triggers for 's' and then it works worldwide.
            average['left'] >> save_image_left['image'],
            average['right'] >> save_image_right['image'],
        ]
    if options.ip4:
        graph += [
             image_source4['pair'] >> average['pair'],
             average['left']  >> imshow_left_average['image'],
             average['right'] >> imshow_right_average['image'],
             imshow_left_average['save'] >> save_image_left['__test__'],
             imshow_left_average['save'] >> save_image_right['__test__'],  # can only have one imshow source of triggers for 's' and then it works worldwide.
             average['left'] >> save_image_left['image'],
             average['right'] >> save_image_right['image'],
         ]


if options.dump:
    if options.ip3:
        graph += [
            image_source3['left']  >> save_left['image'],
            image_source3['right'] >> save_right['image'],
        ]
    if options.ip4:
        graph += [
             image_source4['left']  >> save_left['image'],
             image_source4['right'] >> save_right['image'],
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
    if options.ip4:
        graph += [
            image_source4['left'] >> mat2ImgMsgLeft4['image'],
            image_source4['right'] >> mat2ImgMsgRight4['image'],
            mat2ImgMsgLeft4['image']  >> pub_imageLeft4['input'],
            mat2ImgMsgRight4['image'] >> pub_imageRight4['input'],
        ]
    if options.ip3:
        graph += [
            image_source3['left'] >> mat2ImgMsgLeft3['image'],
            image_source3['right'] >> mat2ImgMsgRight3['image'],
            mat2ImgMsgLeft3['image']  >> pub_imageLeft3['input'],
            mat2ImgMsgRight3['image'] >> pub_imageRight3['input'],
        ]
    subprocess.Popen("rosbag record -q /odom /image_left3 /image_right3 /image_left4 /image_right4 /tf", shell=True)
if not options.read and not options.bag and not options.fps:
    if options.ip3:
        graph += [
            image_source3['counter'] >> bit_bucket3['counter'],
            image_source3['left']  >> imshow_left3['image'],
            image_source3['right'] >> imshow_right3['image'],
        ]
    if options.ip4:
        graph += [
            image_source4['counter'] >> bit_bucket4['counter'],
            image_source4['left']  >> imshow_left4['image'],
            image_source4['right'] >> imshow_right4['image'],
         ]



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

if options.ip3 and options.ip4:
    ecto_ros.init(sys.argv, "dslam_ecto_bridge")
if options.ip3 and not options.ip4:
    ecto_ros.init(sys.argv, "dslam_ecto_bridge3")
if options.ip4 and not options.ip3:
    ecto_ros.init(sys.argv, "dslam_ecto_bridge4")
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
