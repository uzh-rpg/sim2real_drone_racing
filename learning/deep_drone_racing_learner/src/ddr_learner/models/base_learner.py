#Base model for learning

import os
import time
import math
from itertools import count
import random
import tensorflow as tf
import numpy as np
from keras.utils.generic_utils import Progbar
from .nets import resnet8 as prediction_network
from .data_utils import DirectoryIterator

TEST_PHASE = 0
TRAIN_PHASE = 1

class TrajectoryLearner(object):
    def __init__(self):
        pass

    def read_from_disk(self, inputs_queue):
        """Consumes the inputs queue.
        Args:
            filename_and_label_tensor: A scalar string tensor.
        Returns:
            Two tensors: the decoded images, and the labels.
        """
        pnt_seq = tf.cast(inputs_queue[1], dtype=tf.float32)
        file_content = tf.read_file(inputs_queue[0])
        image_seq = tf.image.decode_jpeg(file_content, channels=3)

        return image_seq, pnt_seq

    def preprocess_image(self, image):
        """ Preprocess an input image
        Args:
            Image: A uint8 tensor
        Returns:
            image: A preprocessed float32 tensor.
        """
        image = tf.image.resize_images(image,
                [self.config.img_height, self.config.img_width])
        image = tf.cast(image, dtype=tf.float32)
        image = tf.divide(image, 255.0)
        return image

    def get_filenames_list(self, directory):
        # Load labels, velocities and image filenames. The shuffling will be done after
        iterator = DirectoryIterator(directory, shuffle=False)
        return iterator.filenames, iterator.ground_truth

    def build_train_graph(self):
        is_training_ph = tf.placeholder(tf.bool, shape=(), name="is_training")
        with tf.name_scope("data_loading"):
            # generate training and validation batches ( we do not need labels)
            train_batch, n_samples_train = self.generate_batches(
                           self.config.train_dir)
            val_batch, n_samples_test = self.generate_batches(
                 self.config.val_dir, validation=True)

            #image_batch, pnt_batch, n_samples = \
            #self.generate_batches(self.config.train_dir)
            current_batch = tf.cond(is_training_ph,lambda: train_batch,
                                    lambda: val_batch)

            image_batch, pnt_batch = current_batch[0], current_batch[1]

        with tf.name_scope("trajectory_prediction"):
            pred_pnt = prediction_network(image_batch,
                                          output_dim=self.config.output_dim,
                                          f=self.config.f)


        with tf.name_scope("compute_loss"):
            point_loss = tf.losses.mean_squared_error(labels=pnt_batch[:,:2],
                                                      predictions=pred_pnt[:,:2])

            vel_loss = tf.losses.mean_squared_error(labels=pnt_batch[:, 2],
                                                    predictions=pred_pnt[:,2])

            train_loss = point_loss + 0.1 * vel_loss

        with tf.name_scope("metrics"):
            _, var = tf.nn.moments(pred_pnt, axes=-1)
            std = tf.sqrt(var)
            point_rmse = tf.sqrt(point_loss)
            vel_rmse = tf.sqrt(vel_loss)


        with tf.name_scope("train_op"):
            train_vars = [var for var in tf.trainable_variables()]
            optimizer  = tf.train.AdamOptimizer(self.config.learning_rate,
                                             self.config.beta1)
            self.grads_and_vars = optimizer.compute_gradients(train_loss,
                                                          var_list=train_vars)
            self.train_op = optimizer.apply_gradients(self.grads_and_vars)
            self.global_step = tf.Variable(0,
                                           name='global_step',
                                           trainable=False)
            self.incr_global_step = tf.assign(self.global_step,
                                              self.global_step+1)

        # Collect tensors that are useful later (e.g. tf summary), maybe add
        # images

        self.train_steps_per_epoch = \
            int(math.ceil(n_samples_train/self.config.batch_size))
        self.val_steps_per_epoch = \
            int(math.ceil(n_samples_test/self.config.batch_size))
        self.pred_pnt = pred_pnt
        self.gt_pnt = pnt_batch
        self.point_rmse = point_rmse
        self.vel_rmse = vel_rmse
        self.pred_stds = std
        self.image_batch = image_batch
        self.is_training = is_training_ph
        self.total_loss = train_loss
        self.val_loss_eval = point_loss


    def generate_batches(self, data_dir, validation=False):
        seed = random.randint(0, 2**31 - 1)
        # Load the list of training files into queues
        file_list, pnt_list= self.get_filenames_list(data_dir)
        # Convert to tensors before passing
        inputs_queue = tf.train.slice_input_producer([file_list,
          pnt_list],
          seed=seed,
          shuffle=not validation)

        image_seq, pnt_seq = self.read_from_disk(inputs_queue)
        # Resize images to target size and preprocess them
        image_seq = self.preprocess_image(image_seq)
        # Form training batches
        image_batch, pnt_batch = tf.train.batch([image_seq,
             pnt_seq],
             batch_size=self.config.batch_size,
             # This should be 1 for validation, but makes training significantly slower.
             # Since we are anyway not interested in the absolute value of the metrics, we keep it > 1.
             num_threads=self.config.num_threads,
             capacity=self.config.capacity_queue,
             allow_smaller_final_batch=validation)
        return [image_batch, pnt_batch], len(file_list)

    def collect_summaries(self):

        pnt_error_sum = tf.summary.scalar("point_rmse", self.point_rmse)
        vel_error_sum = tf.summary.scalar("vel_rmse", self.vel_rmse)
        image_sum = tf.summary.image("image", self.image_batch)
        self.step_sum_op = tf.summary.merge([pnt_error_sum, vel_error_sum, image_sum])
        self.validation_error = tf.placeholder(tf.float32, [])
        self.val_error_log = tf.summary.scalar("Validation_Error",
                                               self.validation_error)

    def save(self, sess, checkpoint_dir, step):
        model_name = 'model'
        print(" [*] Saving checkpoint to %s..." % checkpoint_dir)
        if step == 'latest':
            self.saver.save(sess,
                        os.path.join(checkpoint_dir, model_name + '.latest'))
        else:
            self.saver.save(sess,
                        os.path.join(checkpoint_dir, model_name),
                        global_step=step)

    def train(self, config):
        """High level train function.
        Args:
            config: Configuration dictionary
        Returns:
            None
        """
        self.config = config
        self.build_train_graph()
        self.collect_summaries()
        with tf.name_scope("parameter_count"):
            parameter_count = tf.reduce_sum([tf.reduce_prod(tf.shape(v)) \
                                        for v in tf.trainable_variables()])
        self.saver = tf.train.Saver([var for var in \
            tf.trainable_variables()] +  [self.global_step], max_to_keep=100)
        sv = tf.train.Supervisor(logdir=config.checkpoint_dir,
                                 save_summaries_secs=0,
                                 saver=None)

        gpu_config = tf.ConfigProto()
        gpu_config.gpu_options.allow_growth=True
        with sv.managed_session(config=gpu_config) as sess:
            print("Number of params: {}".format(sess.run(parameter_count)))
            if config.resume_train:
                print("Resume training from previous checkpoint")
                checkpoint = tf.train.latest_checkpoint(
                                                config.checkpoint_dir)
                self.saver.restore(sess, checkpoint)

            progbar = Progbar(target=self.train_steps_per_epoch)

            n_epochs = 0

            for step in count(start=1):
                if sv.should_stop():
                    break
                start_time = time.time()
                fetches = { "train" : self.train_op,
                              "global_step" : self.global_step,
                              "incr_global_step": self.incr_global_step
                             }
                if step % config.summary_freq == 0:
                    fetches["vel_rmse"] = self.vel_rmse
                    fetches["pnt_rmse"] = self.point_rmse
                    fetches["stds"] = self.pred_stds
                    fetches["summary"] = self.step_sum_op

                # Runs a series of operations
                results = sess.run(fetches,
                                   feed_dict={self.is_training: True})

                progbar.update(step % self.train_steps_per_epoch)


                gs = results["global_step"]

                if step % config.summary_freq == 0:
                    sv.summary_writer.add_summary(results["summary"], gs)
                    train_epoch = math.ceil( gs /self.train_steps_per_epoch)
                    train_step = gs - (train_epoch - 1) * self.train_steps_per_epoch
                    print("Epoch: [%2d] [%5d/%5d] time: %4.4f/it point_rmse: %.3f " \
                          "vel_rmse: %.6f, point_std: %.6f, vel_std: %.6f"
                       % (train_epoch, train_step, self.train_steps_per_epoch, \
                          time.time() - start_time, results["pnt_rmse"],
                          results["vel_rmse"],
                          np.mean(results["stds"][:2]),
                          results["stds"][2] ))

                if step % self.train_steps_per_epoch == 0:
                    n_epochs += 1
                    actual_epoch = int(gs / self.train_steps_per_epoch)
                    self.save(sess, config.checkpoint_dir, actual_epoch)
                    progbar = Progbar(target=self.train_steps_per_epoch)
                    # Evaluate val accuracy
                    val_error = 0
                    for i in range(self.val_steps_per_epoch):
                        loss = sess.run(self.val_loss_eval, feed_dict={
                            self.is_training: False})
                        val_error += loss
                    val_error = val_error / self.val_steps_per_epoch
                    # Log to Tensorflow board
                    val_sum = sess.run(self.val_error_log, feed_dict ={
                        self.validation_error: val_error})
                    sv.summary_writer.add_summary(val_sum, n_epochs)
                    print("Epoch [{}] Validation Loss: {}".format(
                        actual_epoch, val_error))
                    if (n_epochs == self.config.max_epochs):
                        break


    def build_test_graph(self):
        """This graph will be used for testing. In particular, it will
           compute the loss on a testing set, or for prediction of trajectories.
        """
        image_height, image_width = self.config.test_img_height, \
                                    self.config.test_img_width

        self.num_channels = 3
        input_uint8 = tf.placeholder(tf.uint8, [None, image_height,
                                    image_width, self.num_channels],
                                    name='raw_input')


        input_mc = self.preprocess_image(input_uint8)

        pnt_batch = tf.placeholder(tf.float32, [None, self.config.output_dim],
                                          name='gt_labels')


        with tf.name_scope("trajectory_prediction"):
            pred_pnt = prediction_network(input_mc,
                    output_dim=self.config.output_dim, f=self.config.f)

        with tf.name_scope("compute_loss"):
            point_loss = tf.losses.mean_squared_error(labels=pnt_batch[:,:2],
                                                      predictions=pred_pnt[:,:2])

            vel_loss = tf.losses.mean_squared_error(labels=pnt_batch[:, 2],
                                                    predictions=pred_pnt[:, 2])
            total_loss = point_loss + vel_loss


        with tf.name_scope("metrics"):
            _, var = tf.nn.moments(pred_pnt, axes=-1)
            std = tf.sqrt(var)

        self.inputs_img = input_uint8
        self.pred_pnt = pred_pnt
        self.gt_pnt = pnt_batch
        self.pred_stds = std
        self.point_loss = point_loss
        self.total_loss = total_loss
        self.vel_loss = vel_loss

    def setup_inference(self, config, mode):
        """Sets up the inference graph.
        Args:
            mode: either 'loss' or 'prediction'. When 'loss', it will be used for
            computing a loss (gt trajectories should be provided). When
            'prediction', it will just make predictions (to be used in simulator)
            config: config dictionary. it should have target size and trajectories
        """
        self.mode = mode
        self.config = config
        self.build_test_graph()

    def inference(self, inputs, sess):
        results = {}
        fetches = {}
        if self.mode == 'loss':
            fetches["vel_loss"] = self.vel_loss
            fetches["pnt_loss"] = self.point_loss
            fetches["stds"] = self.pred_stds

            results = sess.run(fetches,
                               feed_dict= {self.inputs_img: inputs['images'],
                                           self.gt_pnt: inputs['gt_labels']})
        if self.mode == 'prediction':
            results['predictions'] = sess.run(self.pred_pnt, feed_dict = {
                self.inputs_img: inputs['images']})

        return results
