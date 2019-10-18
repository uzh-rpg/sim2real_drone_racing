#!/usr/bin/env python

import rospy
from Network import Network
import os, datetime
import tensorflow as tf
import sys
import gflags
from ddr_learner.common_flags import FLAGS

def run_network():

    rospy.init_node('deep_drone_racing_learned_traj', anonymous=True)

    # RUN NETWORK
    with tf.Session() as sess:
        network = Network.Network(FLAGS)
        sess.run(tf.global_variables_initializer())
        network.run(sess)

def parse_flags(argv):
    # Utility main to load flags
    try:
      argv = FLAGS(argv)  # parse flags
    except gflags.FlagsError:
      print ('Usage: %s ARGS\\n%s' % (sys.argv[0], FLAGS))
      sys.exit(1)

if __name__ == "__main__":
    parse_flags(sys.argv)
    run_network()
