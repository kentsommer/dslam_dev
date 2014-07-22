#!/usr/bin/python

import argparse
import dslam_ecto_bridge.dslam_image_bridge as image_bridge
import dslam_bag.dslam_bag_vision_utils as vision_utils
#import dslam_ecto_bridge.beachhead as beachhead
import ecto
from ecto.opts import scheduler_options, run_plasm
from ecto_opencv.highgui import imshow, FPSDrawer, ImageSaver, ImageReader
import ecto_ros, ecto_ros.ecto_sensor_msgs as ecto_sensor_msgs
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
parser.add_argument('-front', '--frontcam', default=False, action='store_true', help='Use front camera')
parser.add_argument('-back', '--backcam', default=False, action='store_true', help='Use back camera')
# add ecto scheduler args
scheduler_options(parser)
#myargs = rospy.myargv(argv=sys.argv)
options = parser.parse_args()

##############################################################################
# Cells
##############################################################################

if not options.read and (options.frontcam or options.backcam):
    image_sourceBack = image_bridge.DSlamSource('image_source', ip='192.168.1.4', port=5555)
    image_sourceFront = image_bridge.DSlamSource('image_source', ip='192.168.1.3', port=5555)
    #bit_bucketBack = vision_utils.BitBucket('bit_bucket', verbose=False)
    #bit_bucketFront = vision_utils.BitBucket('bit_bucket', verbose=False)
if options.fps:
    if options.frontcam:
        fps_leftFront = FPSDrawer()
        fps_rightFront = FPSDrawer()
    if options.backcam:
        fps_leftBack = FPSDrawer()
        fps_rightBack = FPSDrawer()


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
imshow_leftBack = imshow('show_raw_left', name='raw_leftBack')
imshow_rightBack = imshow('show_raw_right', name='raw_rightBack')
imshow_leftFront = imshow('show_raw_left', name='raw_leftFront')
imshow_rightFront = imshow('show_raw_right', name='raw_rightFront')

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
mat2ImgMsgLeftBack = vision_utils.Mat2ImageStamped()
mat2ImgMsgRightBack = vision_utils.Mat2ImageStamped()
mat2ImgMsgLeftFront = vision_utils.Mat2ImageStamped()
mat2ImgMsgRightFront = vision_utils.Mat2ImageStamped()
# Publisher
ImagePub = ecto_sensor_msgs.Publisher_Image
pub_imageLeftFront = ImagePub("image_publisher", topic_name='/image/front/left')
pub_imageRightFront = ImagePub("image_publisher", topic_name='/image/front/right')
pub_imageLeftBack = ImagePub("image_publisher", topic_name='/image/back/left')
pub_imageRightBack = ImagePub("image_publisher", topic_name='/image/back/right')

##############################################################################
# Graph
##############################################################################

graph = []  # graph cannot be empty!

if options.fps:
    if options.frontcam:    
        graph += [
            image_sourceFront['left']  >> fps_leftFront['image'],
            image_sourceFront['right'] >> fps_rightFront['image'],
            fps_leftFront['image']  >> imshow_leftFront['image'],
            fps_rightFront['image'] >> imshow_rightFront['image'],
        ]
    if options.backcam:
        graph += [
            image_sourceBack['left']  >> fps_leftBack['image'],
            image_sourceBack['right'] >> fps_rightBack['image'],
            fps_leftBack['image']  >> imshow_leftBack['image'],
            fps_rightBack['image'] >> imshow_rightBack['image'],
         ]

if options.average:
    if options.frontcam:
        graph += [
            image_sourceFront['pair'] >> average['pair'],
            average['left']  >> imshow_left_average['image'],
            average['right'] >> imshow_right_average['image'],
            imshow_left_average['save'] >> save_image_left['__test__'],
            imshow_left_average['save'] >> save_image_right['__test__'],  # can only have one imshow source of triggers for 's' and then it works worldwide.
            average['left'] >> save_image_left['image'],
            average['right'] >> save_image_right['image'],
        ]
    if options.backcam:
        graph += [
             image_sourceBack['pair'] >> average['pair'],
             average['left']  >> imshow_left_average['image'],
             average['right'] >> imshow_right_average['image'],
             imshow_left_average['save'] >> save_image_left['__test__'],
             imshow_left_average['save'] >> save_image_right['__test__'],  # can only have one imshow source of triggers for 's' and then it works worldwide.
             average['left'] >> save_image_left['image'],
             average['right'] >> save_image_right['image'],
         ]


if options.dump:
    if options.frontcam:
        graph += [
            image_sourceFront['left']  >> save_left['image'],
            image_sourceFront['right'] >> save_right['image'],
        ]
    if options.backcam:
        graph += [
             image_sourceBack['left']  >> save_left['image'],
             image_sourceBack['right'] >> save_right['image'],
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
    if options.backcam:
        graph += [
            image_sourceBack['left'] >> mat2ImgMsgLeftBack['image'],
            image_sourceBack['time'] >> mat2ImgMsgLeftBack['time'],
            image_sourceBack['right'] >> mat2ImgMsgRightBack['image'],
            image_sourceBack['time'] >> mat2ImgMsgRightBack['time'], 
            mat2ImgMsgLeftBack['image']  >> pub_imageLeftBack['input'],
            mat2ImgMsgRightBack['image'] >> pub_imageRightBack['input'],
        ]
    if options.frontcam:
        graph += [
            image_sourceFront['left'] >> mat2ImgMsgLeftFront['image'],
            image_sourceFront['time'] >> mat2ImgMsgLeftFront['time'],
            image_sourceFront['right'] >> mat2ImgMsgRightFront['image'],
            image_sourceFront['time'] >> mat2ImgMsgRightFront['time'],
            mat2ImgMsgLeftFront['image']  >> pub_imageLeftFront['input'],
            mat2ImgMsgRightFront['image'] >> pub_imageRightFront['input'],
        ]
    if options.frontcam and not options.backcam:
        subprocess.Popen("rosbag record -q /odom /image/front/left /image/front/right /tf", shell=True)
    if options.backcam and not options.frontcam:
        subprocess.Popen("rosbag record -q /odom /image/back/left /image/back/right /tf", shell=True)
    if options.frontcam and options.backcam:
        subprocess.Popen("rosbag record -q /odom /image/front/left /image/front/right /image/back/left /image/back/right /tf", shell=True)
if not options.read and not options.bag and not options.fps:
    if options.frontcam:
        graph += [
            #image_sourceFront['counter'] >> bit_bucketFront['counter'],
            image_sourceFront['left']  >> imshow_leftFront['image'],
            image_sourceFront['right'] >> imshow_rightFront['image'],
        ]
    if options.backcam:
        graph += [
            #image_sourceBack['counter'] >> bit_bucketBack['counter'],
            image_sourceBack['left']  >> imshow_leftBack['image'],
            image_sourceBack['right'] >> imshow_rightBack['image'],
         ]
    else:
        sys.exit("\n Dude select something, you have to either read from file system or select a combination of cameras: -front, -back, or -r are all options \n ...pick one! Oh and then re-run me \n")

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

if options.frontcam and options.backcam:
    ecto_ros.init(sys.argv, "dslam_ecto_bridge")
if options.frontcam and not options.backcam:
    ecto_ros.init(sys.argv, "dslam_ecto_bridgeFront")
if options.backcam and not options.frontcam:
    ecto_ros.init(sys.argv, "dslam_ecto_bridgeBack")
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
