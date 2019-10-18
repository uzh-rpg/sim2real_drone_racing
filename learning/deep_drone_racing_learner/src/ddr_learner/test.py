import os
import sys
import gflags
import math
import tensorflow as tf
import json
import numpy as np

from models.base_learner import TrajectoryLearner
from utils import compute_loss
from models.data_utils import DirectoryIterator

from common_flags import FLAGS

def _main():
    learner = TrajectoryLearner()
    learner.setup_inference(FLAGS, mode='loss')

    saver = tf.train.Saver([var for var in tf.trainable_variables()])
    init = tf.initialize_all_variables()

    test_generator = DirectoryIterator(FLAGS.test_dir,
                                       shuffle=False,
                                       target_size=(FLAGS.img_width, FLAGS.img_height),
                                       batch_size = FLAGS.batch_size)

    steps = int(math.ceil(test_generator.samples / FLAGS.batch_size))

    with tf.Session() as sess:
        saver.restore(sess, FLAGS.ckpt_file)
        print("--------------------------------------------------")
        print("Restored checkpoint file {}".format(FLAGS.ckpt_file))
        print("--------------------------------------------------")
        outs = compute_loss(sess, learner, test_generator, steps, verbose=1)

    # Logging
    print("Average Vel Std: {:.3f}".format(outs['vel_std']))
    print("Average Point Std: {:.3f}".format(outs['pnt_std']))
    print("Average Vel MSE: {:.3f}".format(outs['vel_mse']))
    print("Average Point MSE: {:.3f}".format(outs['pnt_mse']))

def main(argv):
    # Utility main to load flags
    try:
      argv = FLAGS(argv)  # parse flags
    except gflags.FlagsError:
      print ('Usage: %s ARGS\\n%s' % (sys.argv[0], FLAGS))
      sys.exit(1)
    _main()

if __name__ == "__main__":
    main(sys.argv)
