import numpy as np
import re
import os
import cv2

from keras.preprocessing.image import Iterator

class DirectoryIterator(Iterator):
    def __init__(self, directory,
            target_size=(224,224), num_channels=3, # target size is width, height  (compatible with opencv)
            batch_size=32, shuffle=False, seed=None, follow_links=False):
        self.directory = directory
        self.target_size = tuple(target_size)
        self.follow_links = follow_links

        self.num_channels = 3
        self.image_shape = (self.target_size[1], self.target_size[0], self.num_channels)

        self.samples = 0

        experiments = []
        for subdir in sorted(os.listdir(directory)):
            if os.path.isdir(os.path.join(directory, subdir)):
                experiments.append(subdir)
        self.num_experiments = len(experiments)
        self.formats = {'jpg', 'png'}

        # Associate each filename with a corresponding label
        self.filenames = []
        self.ground_truth = []

        for subdir in experiments:
            subpath = os.path.join(directory, subdir)
            self._decode_experiment_dir(subpath)

        # Conversion of list into array
        if len(self.ground_truth) > 0:
            self.ground_truth = np.array(self.ground_truth, dtype = np.float32)
            self.with_gt = True
        else:
            self.with_gt = False

        print('Found {} images belonging to {} experiments.'.format(
                self.samples, self.num_experiments))
        super(DirectoryIterator, self).__init__(self.samples,
                batch_size, shuffle, seed)

    def _recursive_list(self, subpath):
        return sorted(os.walk(subpath, followlinks=self.follow_links),
                key=lambda tpl: tpl[0])

    def _decode_experiment_dir(self, dir_subpath):
        labels_filename = os.path.join(dir_subpath, "labels.txt")
        with_gt = False

        if os.path.isfile(labels_filename):
            # Try load labels
            ground_truth = np.loadtxt(labels_filename, usecols=(0,1,2), delimiter=';')
            with_gt=True
        else:
            print("No GT found in {}".format(dir_subpath))

        # Now fetch all images in the image subdir
        image_dir_path = os.path.join(dir_subpath, "images")
        for root, _, files in self._recursive_list(image_dir_path):
            sorted_files = sorted(files,
                    key = lambda fname: int(re.search(r'\d+',fname).group()))
            for frame_number, fname in enumerate(sorted_files):
                is_valid = False
                for extension in self.formats:
                    if fname.lower().endswith('.' + extension):
                        is_valid = True
                        break
                if is_valid:
                    absolute_path = os.path.join(root, fname)
                    self.filenames.append(absolute_path)
                    if with_gt:
                        self.ground_truth.append(ground_truth[frame_number])
                    self.samples += 1

    def _load_img(self, path):
        """
        Load an image. Ans reshapes it to target size

        # Arguments
            path: Path to image file.
            target_size: Either `None` (default to original size)
                or tuple of ints `(img_width, img_height)`.

        # Returns
            Image as numpy array.
        """
        img = cv2.imread(path)
        if self.target_size:
            if (img.shape[1], img.shape[0]) != self.target_size:
                img = cv2.resize(img, self.target_size,
                                 interpolation=cv2.INTER_LINEAR)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return img

    def next(self):
        """
        Public function to fetch next batch. Note that this function
        will only be used for evaluation and testing, but not for training.

        # Returns
            The next batch of images and labels.
        """
        with self.lock:
            #index_array, start, current_batch_size = next(
            index_array = next(
                    self.index_generator)
            current_batch_size = index_array.shape[0]

        # Image transformation is not under thread lock, so it can be done in
        # parallel
        batch_x = np.zeros((current_batch_size,) + self.image_shape,
                dtype=np.uint8)

        # Build batch of image data
        for i, j in enumerate(index_array):
            fname = self.filenames[j]
            x = self._load_img(os.path.join(fname))
            batch_x[i] = x
        if self.with_gt:
            batch_y = self.ground_truth[index_array]
        else:
            batch_y = None

        return batch_x, batch_y
