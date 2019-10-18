#!/usr/bin/env python
import cv2
import rospy
import tensorflow as tf
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TwistStamped

from ddr_learner.models.base_learner import TrajectoryLearner

bridge = CvBridge()


class Network(object):
    def __init__(self, config):

        self.config = config
        self.pub = rospy.Publisher("cnn_predictions", TwistStamped, queue_size=1)
        self.feedthrough_sub = rospy.Subscriber("/hummingbird/state_change", Bool,
                                                self.callback_feedthrough, queue_size=1)
        self.checkpoint_sub = rospy.Subscriber("/checkpoint", String,
                                               self.callback_checkpoint, queue_size=1)

        self.gamma_sub = rospy.Subscriber("/gamma", String,
                                          self.callback_gamma, queue_size=1)

        self.learner = TrajectoryLearner()
        self.learner.setup_inference(config, mode='prediction')

        self.saver = tf.train.Saver([var for var in tf.trainable_variables()])
        self.use_network_out = False
        self.smoothed_pred = np.zeros((3,))
        self.alpha = 1.0
        self.checkpoint = ""
        self.gamma = ""

    def callback_feedthrough(self, data):
        self.use_network_out = data.data
        if self.use_network_out:

            if self.config.ckpt_file:
                checkpoint = self.config.ckpt_file
            else:
                checkpoint = tf.train.latest_checkpoint(self.config.checkpoint_dir)
            self.saver.restore(self.sess, checkpoint)
            print("--------------------------------------------------")
            print("Restored checkpoint file {}".format(checkpoint))
            print("--------------------------------------------------")

    def callback_gamma(self, data):
        self.gamma = data.data

    def callback_checkpoint(self, data):
        self.config.ckpt_file = self.config.checkpoint_dir + self.gamma + "/" + data.data

    def run(self, sess):
        self.sess = sess
        while not rospy.is_shutdown():
            data_camera = None

            while data_camera is None:
                try:
                    data_camera = rospy.wait_for_message("camera",
                                                         Image)

                except:
                    print("could not aquire image data")
                    break

            if self.use_network_out:
                print("Using network prediction!")
            else:
                print("Not using prediction!")
                continue

            # Reading image and processing it through the network
            try:
                cv_input_image = bridge.imgmsg_to_cv2(data_camera)

            except CvBridgeError as e:
                print(e)
                continue

            inputs = {}

            cv_input_image = cv2.resize(cv_input_image, (300, 200),
                                 interpolation=cv2.INTER_LINEAR)

            inputs['images'] = cv_input_image[None]
            results = self.learner.inference(inputs, sess)
            predictions = np.squeeze(results['predictions'])
            msg = TwistStamped()
            msg.header.stamp = rospy.Time.now()

            self.smoothed_pred = (1 - self.alpha) * self.smoothed_pred + \
                                 self.alpha * predictions

            msg.twist.linear.x = self.smoothed_pred[0]
            msg.twist.linear.y = self.smoothed_pred[1]
            msg.twist.linear.z = self.smoothed_pred[2]
            self.pub.publish(msg)
