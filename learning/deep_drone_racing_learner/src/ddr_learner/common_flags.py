import gflags

FLAGS = gflags.FLAGS

# Train parameters
gflags.DEFINE_integer('img_width', 300, 'Target Image Width')
gflags.DEFINE_integer('img_height', 200, 'Target Image Height')
gflags.DEFINE_integer('batch_size', 32, 'Batch size in training and evaluation')
gflags.DEFINE_float("learning_rate", 0.001, "Learning rate of for adam")
gflags.DEFINE_float("beta1", 0.9, "Momentum term of adam")
gflags.DEFINE_float("f", 1.0, "Model Width, float in [0,1]")
gflags.DEFINE_integer('output_dim', 3, "Number of output dimensionality")

gflags.DEFINE_string('train_dir', "../../data/Training", 'Folder containing'
                     ' training experiments')
gflags.DEFINE_string('val_dir', "../../data/Testing", 'Folder containing'
                     ' validation experiments')
gflags.DEFINE_string('checkpoint_dir', "/tmp/debug_learning", "Directory name to"
                     "save checkpoints and logs.")

# Input Queues reading
gflags.DEFINE_integer('num_threads', 8, 'Number of threads reading and '
                      '(optionally) preprocessing input files into queues')
gflags.DEFINE_integer('capacity_queue', 100, 'Capacity of input queue. A high '
                      'number speeds up computation but requires more RAM')

# Log parameters
gflags.DEFINE_integer("max_epochs", 100, "Maximum number of training epochs")

gflags.DEFINE_bool('resume_train', False, 'Whether to restore a trained'
                   ' model for training')
gflags.DEFINE_integer("summary_freq", 100, "Logging every log_freq iterations")
gflags.DEFINE_integer("save_latest_freq", 100, \
    "Save the latest model every save_latest_freq iterations"
                     "(overwrites the previous latest model)")

# Testing parameters
gflags.DEFINE_string('test_dir', "../../data/validation_sim2real/beauty", 'Folder containing'
                     ' testing experiments')
gflags.DEFINE_string('output_dir', "./tests/test_0", 'Folder containing'
                     ' testing experiments')
gflags.DEFINE_string("ckpt_file", "/tmp/logs/model-2", "Checkpoint file")
gflags.DEFINE_integer('test_img_width', 300, 'Target Image Width')
gflags.DEFINE_integer('test_img_height', 200, 'Target Image Height')
