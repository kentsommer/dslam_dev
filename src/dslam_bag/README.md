D(damn)SLAM with ROSBag support
=========

Quickly and easily ROSBAG incoming DSLAM image and /odom feeds. 

**Usage:**

* Optional arguments:
    * -h : show help message
    * -a : do averaging on stereo pairs
    * -b : bag incoming images and /odom
    * -d : dump a continuous stream of images (Use with -o)
    * -f : Show fps etched on the image windows
    * -o : default image storage location
    * -r : read from filesystem instead of DSLAM board
    * -front : use the front camera pair (left, right). Can be used with or without -back
    * -back : use the back camera pair (left, right). Can be used with or without -front

* To run:
    * ./dslam_image_capture_bag.py
    
