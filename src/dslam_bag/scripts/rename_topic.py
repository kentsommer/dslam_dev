#!/usr/bin/env python

from rosbag import Bag

with Bag('correctodom_wide_flat.bag', 'w') as bag:
    for topic, msg, t in Bag('original.bag'):
        bag.write('/image_left' if topic == '/image_left4' else topic, msg, t)
        bag.write('/image_right' if topic == '/image_right4' else topic, msg, t)
