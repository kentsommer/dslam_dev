#!/usr/bin/python

import argparse
import dslam_ecto_bridge.dslam_image_bridge as image_bridge
import dslam_bag.dslam_bag_vision_utils as vision_utils
#import dslam_ecto_bridge.beachhead as beachhead
import ecto
from ecto.opts import scheduler_options, run_plasm
from ecto_opencv.highgui import imshow, FPSDrawer
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
parser.add_argument('-b', '--bag', default=False, action='store_true', help='Write to a ROS Bag.')
parser.add_argument('-front', '--frontcam', default=False, action='store_true', help='Use front camera')
parser.add_argument('-back', '--backcam', default=False, action='store_true', help='Use back camera')
# add ecto scheduler args
scheduler_options(parser)
#myargs = rospy.myargv(argv=sys.argv)
options = parser.parse_args()

##############################################################################
# Cells
##############################################################################

if options.frontcam or options.backcam:
    image_sourceBack = image_bridge.DSlamSource('image_source', ip='192.168.1.4', port=5555)
    image_sourceFront = image_bridge.DSlamSource('image_source', ip='192.168.1.3', port=5555)

# Raw image display
imshow_leftBack = imshow('show_raw_left', name='raw_leftBack')
imshow_rightBack = imshow('show_raw_right', name='raw_rightBack')
imshow_leftFront = imshow('show_raw_left', name='raw_leftFront')
imshow_rightFront = imshow('show_raw_right', name='raw_rightFront')
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
if not options.bag:
    if options.frontcam:
        graph += [
            image_sourceFront['left']  >> imshow_leftFront['image'],
            image_sourceFront['right'] >> imshow_rightFront['image'],
        ]
    if options.backcam:
        graph += [
            image_sourceBack['left']  >> imshow_leftBack['image'],
            image_sourceBack['right'] >> imshow_rightBack['image'],
         ]
    if not options.backcam and not options.frontcam:
        sys.exit("\n Dude select something as an imput -> specifically a combination of cameras: -front and -back are options \n ...pick one! Oh and then re-run me \n")

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
