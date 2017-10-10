#!/usr/bin/python
"""
This program is demonstration for face and object detection using haar-like features.
The program finds faces in a camera image or video stream and displays a red box around them.

Original C implementation by:  ?
Python implementation by: Roman Stanchak, James Bowman
Updated: Copyright (c) 2016, Tal Regev.
"""

import sys
import os
from optparse import OptionParser

import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge
import cv2
import numpy

# Parameters for haar detection
# From the API:
# The default parameters (scale_factor=2, min_neighbors=3, flags=0) are tuned
# for accurate yet slow object detection. For a faster operation on real video
# images the settings are:
# scale_factor=1.2, min_neighbors=2, flags=CV_HAAR_DO_CANNY_PRUNING,
# min_size=<minimum possible face size

min_size      = (10, 10)
image_scale   = 2
haar_scale    = 1.2
min_neighbors = 2
haar_flags    = 0

if __name__ == '__main__':

    # TODO add this file in the repository and make it relative to this python script. (not all people will run this from linux)
    haarfile = '/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml'

    parser = OptionParser(usage = "usage: %prog [options]")
    parser.add_option("-c", "--cascade", action="store", dest="cascade", type="str", help="Haar cascade file, default %default", default = haarfile)
    parser.add_option("-t", "--topic", action="store", dest="topic", type="str", help="Topic to find a face on, default %default", default = '/camera/rgb/image_raw')
    parser.add_option("-ct", "--ctopic", action="store", dest="ctopic", type="str", help="Compressed topic to find a face on, default %default", default = '/camera/rgb/image/compressed')
    (options, args) = parser.parse_args()

    cascade = cv2.CascadeClassifier()
    cascade.load(options.cascade)
    br = CvBridge()

    def detect_and_draw(imgmsg):
        img = br.imgmsg_to_cv2(imgmsg, "bgr8")
        # allocate temporary images
        new_size = (int(img.shape[1] / image_scale), int(img.shape[0] / image_scale))

        # convert color input image to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # scale input image for faster processing
        small_img = cv2.resize(gray, new_size, interpolation = cv2.INTER_LINEAR)

        small_img = cv2.equalizeHist(small_img)

        if(cascade):
            faces = cascade.detectMultiScale(small_img, haar_scale, min_neighbors, haar_flags, min_size)
            if faces is not None:
                for (x, y, w, h) in faces:
                    # the input to detectMultiScale was resized, so scale the
                    # bounding box of each face and convert it to two CvPoints
                    pt1 = (int(x * image_scale), int(y * image_scale))
                    pt2 = (int((x + w) * image_scale), int((y + h) * image_scale))
                    cv2.rectangle(img, pt1, pt2, (255, 0, 0), 3, 8, 0)

        cv2.imshow("result", img)
        cv2.waitKey(6)

    def compressed_detect_and_draw(compressed_imgmsg):
        img = br.compressed_imgmsg_to_cv2(compressed_imgmsg, "bgr8")
        # allocate temporary images
        new_size = (int(img.shape[1] / image_scale), int(img.shape[0] / image_scale))

        # convert color input image to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # scale input image for faster processing
        small_img = cv2.resize(gray, new_size, interpolation = cv2.INTER_LINEAR)

        small_img = cv2.equalizeHist(small_img)

        if(cascade):
            faces = cascade.detectMultiScale(small_img, haar_scale, min_neighbors, haar_flags, min_size)
            if faces is not None:
                for (x, y, w, h) in faces:
                    # the input to detectMultiScale was resized, so scale the
                    # bounding box of each face and convert it to two CvPoints
                    pt1 = (int(x * image_scale), int(y * image_scale))
                    pt2 = (int((x + w) * image_scale), int((y + h) * image_scale))
                    cv2.rectangle(img, pt1, pt2, (255, 0, 0), 3, 8, 0)

        cv2.imshow("compressed_result", img)
        cv2.waitKey(6)

    rospy.init_node('rosfacedetect')
    rospy.Subscriber(options.topic, sensor_msgs.msg.Image, detect_and_draw)
    rospy.Subscriber(options.ctopic, sensor_msgs.msg.CompressedImage, compressed_detect_and_draw)
    rospy.spin()
