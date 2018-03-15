# Copyright UCL Business plc 2017. Patent Pending. All rights reserved.
#
# The MonoDepth Software is licensed under the terms of the UCLB ACP-A licence
# which allows for non-commercial use only, the full terms of which are made
# available in the LICENSE file.
#
# For any other use of the software not covered by the UCLB ACP-A Licence,
# please contact info@uclb.com

from __future__ import absolute_import, division, print_function

# only keep warnings and errors
import os
os.environ['TF_CPP_MIN_LOG_LEVEL']='0'

import numpy as np
import argparse
import re
import time
import tensorflow as tf
import tensorflow.contrib.slim as slim
import scipy.misc
import matplotlib.pyplot as plt

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from monodepth_model import *
from monodepth_dataloader import *
from average_gradients import *

parser = argparse.ArgumentParser(description='Monodepth TensorFlow implementation.')

parser.add_argument('--encoder',          type=str,   help='type of encoder, vgg or resnet50', default='vgg')
parser.add_argument('--image_path',       type=str,   help='path to the image', required=True)
parser.add_argument('--checkpoint_path',  type=str,   help='path to a specific checkpoint to load', required=True)
parser.add_argument('--input_height',     type=int,   help='input height', default=256)
parser.add_argument('--input_width',      type=int,   help='input width', default=512)

args = parser.parse_args()

def post_process_disparity(disp):
    _, h, w = disp.shape
    l_disp = disp[0,:,:]
    r_disp = np.fliplr(disp[1,:,:])
    m_disp = 0.5 * (l_disp + r_disp)
    l, _ = np.meshgrid(np.linspace(0, 1, w), np.linspace(0, 1, h))
    l_mask = 1.0 - np.clip(20 * (l - 0.05), 0, 1)
    r_mask = np.fliplr(l_mask)
    return r_mask * l_disp + l_mask * r_disp + (1.0 - l_mask - r_mask) * m_disp

class monodepth:
    def __init__(self):

        '''Initialize ros publisher, ros subscriber'''
        self.image_pub = rospy.Publisher("/monodepth/image",Image)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/kitti/left_color_image",Image,self.callback)

        '''Initialize network for the depth estimation'''
        params = monodepth_parameters(
            encoder=args.encoder,
            height=args.input_height,
            width=args.input_width,
            batch_size=2,
            num_threads=1,
            num_epochs=1,
            do_stereo=False,
            wrap_mode="border",
            use_deconv=False,
            alpha_image_loss=0,
            disp_gradient_loss_weight=0,
            lr_loss_weight=0,
            full_summary=False)

        self.left  = tf.placeholder(tf.float32, [2, args.input_height, args.input_width, 3])
        self.model = MonodepthModel(params, "test", self.left, None)

        # SESSION
        config = tf.ConfigProto(allow_soft_placement=True)
        self.sess = tf.Session(config=config)

        # SAVER
        train_saver = tf.train.Saver()

        # INIT
        self.sess.run(tf.global_variables_initializer())
        self.sess.run(tf.local_variables_initializer())
        coordinator = tf.train.Coordinator()
        threads = tf.train.start_queue_runners(sess=self.sess, coord=coordinator)

        # RESTORE
        restore_path = args.checkpoint_path.split(".")[0]
        train_saver.restore(self.sess, restore_path)

    def callback(self,data):
        input_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.imshow("Image window", input_image)
        cv2.waitKey(3)

        original_height, original_width, num_channels = input_image.shape
        input_image = scipy.misc.imresize(input_image, [args.input_height, args.input_width], interp='lanczos')
        input_image = input_image.astype(np.float32) / 255
        input_images = np.stack((input_image, np.fliplr(input_image)), 0)

        self.test_simple(input_images,original_height,original_width)


    def test_simple(self,input_images,original_height,original_width):
        """Test function."""

        disp = self.sess.run(self.model.disp_left_est[0], feed_dict={self.left: input_images})
        disp_pp = post_process_disparity(disp.squeeze()).astype(np.float32)
#        disp_pp = disp[0,:,:].astype(np.float32)

        disp_to_img = scipy.misc.imresize(disp_pp.squeeze(), [original_height, original_width])
        cv2.imshow("disp_to_img", disp_to_img)
        cv2.waitKey(3)
        
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(disp_to_img, "mono8"))


def main(_):

    ic = monodepth()

    #init rospy
    rospy.init_node('monodepth', anonymous=True)

    rospy.spin()

if __name__ == '__main__':
    tf.app.run()
