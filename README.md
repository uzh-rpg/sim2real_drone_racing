# Deep Drone Racing: From Simulation to Reality with Domain Randomization

This repo contains the implementation of a zero-shot sim2real method for drone racing.

<p align="center">
  <img src="./docs/sim2real.gif" alt="ddr">
</p>

For more information visit the project page:[http://rpg.ifi.uzh.ch/research\_drone\_racing.html](http://rpg.ifi.uzh.ch/research_drone_racing.html).

#### Citing

If you use this code in an academic context, please cite the following publication:

Paper: [Deep Drone Racing: From Simulation to Reality with Domain Randomization](http://rpg.ifi.uzh.ch/docs/TRO19_Loquercio.pdf)

Video: [YouTube](https://youtu.be/vdxB89lgZhQ)

```
@article{loquercio2019deep,
  title={Deep Drone Racing: From Simulation to Reality with Domain Randomization},
  doi={10.1109/TRO.2019.2942989},
  author={Loquercio, Antonio and Kaufmann, Elia and Ranftl, Ren{\'e} and Dosovitskiy, Alexey and Koltun, Vladlen and Scaramuzza, Davide},
  journal={IEEE Transactions on Robotics},
  year={2019}
}

```
## Installation

### Requirements

The code was tested with Ubuntu 18.04 and ROS Melodic.
Different OS and ROS versions are possible but not supported.

### Step-by-Step Procedure

Use the following commands to create a new catkin workspace and a virtual environment with all the required dependencies.

```bash
export ROS_VERSION=melodic
mkdir drone_racing_ws
cd drone_racing_ws
export CATKIN_WS=./catkin_ddr
mkdir -p $CATKIN_WS/src
cd $CATKIN_WS
catkin init
catkin config --extend /opt/ros/$ROS_VERSION
catkin config --merge-devel
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-fdiagnostics-color
cd src

git clone https://github.com/uzh-rpg/sim2real_drone_racing.git
cd sim2real_drone_racing
cd ..
vcs-import < sim2real_drone_racing/dependencies.yaml
touch octomap/octovis/CATKIN_IGNORE

# Build and re-source the workspace
catkin build
. $CATKIN_WS/devel/setup.bash

# Create your learning environment
virtualenv -p python2.7 ./droneflow
source ./droneflow/bin/activate

# If you have a GPU, use the following command. You will need to have CUDA 10.0 installed for it to work.
#pip install tensorflow-gpu==1.13.1
# If you don't have a GPU, uncomment the previous line and comment the next
pip install tensorflow==1.13.1

# Install Required Python dependecies
cd $CATKIN_WS/src/sim2real_drone_racing
pip install -r python_dependencies.txt

```


## Let's Race

Once you have installed the dependencies, you will be able to fly in simulation with our pre-trained checkpoint. You don't need GPU for execution. Note that if the network can't run at least at 10Hz, you won't be able to fly successfully.

Open a terminal and type:
```bash
cd drone_racing_ws
. ./catkin_ddr/devel/setup.bash
. ./droneflow/bin/activate
export CUDA_VISIBLE_DEVICES=''
roslaunch deep_drone_racing_learning  net_controller_launch.launch

```

Open an other terminal and type:
```bash
cd drone_racing_ws
. ./catkin_ddr/devel/setup.bash
. ./droneflow/bin/activate
roslaunch test_racing test_racing.launch

```

## Train your own Sim2Real model

You can use the following commands to generate data in simulation and train your model on it. The trained checkpoint can then be used to control a physical platform on a race track.

### Generate data

```bash
cd drone_racing_ws
. ./catkin_ddr/devel/setup.bash
. ./droneflow/bin/activate
roscd drone_racing/resources/scripts
python collect_data.py

```

It is possible to change parameters (number of iteration per background/ gate texture/ etc. ) in the above script.
Defaults should be good. Optionally, you can use the data we have already collected, available at [this link](http://rpg.ifi.uzh.ch/datasets/sim2real_ddr/simulation_training_data.zip).


### Train the Network

```bash
roscd deep_drone_racing_learner/src/ddr_learner

```

Modify the file [train\_model.sh](./learning/deep_drone_racing_learner/src/ddr_learner/train_model.sh) to add the path of validation data collected in the real world, which you can download from [this link](http://rpg.ifi.uzh.ch/datasets/sim2real_ddr/validation_real_data.zip).
Then, run the following command to train the model.

```bash
./train_model.sh

```

### Test the Network

Edit the following file to use the checkpoint you just trained

```bash
rosed deep_drone_racing_learning net_controller_launch.launch

```

The trained network can now be tested in an environment which was never observed at training time.

Open a terminal and run:

```bash
cd drone_racing_ws
. ./catkin_ddr/devel/setup.sh
. ./droneflow/bin/activate
export CUDA_VISIBLE_DEVICES=''
roslaunch deep_drone_racing_learning  net_controller_launch.launch

```

Open another terminal and run:

```bash
cd drone_racing_ws
. ./catkin_ddr/devel/setup.sh
. ./droneflow/bin/activate
roslaunch test_racing test_racing.launch

```
