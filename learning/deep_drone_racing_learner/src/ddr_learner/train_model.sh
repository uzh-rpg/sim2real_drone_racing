#!/bin/bash

train_data=../../data/Training
val_data=../../data/Training

python2.7 train.py --checkpoint_dir=/tmp/logs --f=0.5 --train_dir=$train_data --val_dir=$val_data --summary_freq=100 --batch_size=200 --max_epochs=100 --num_threads=6
