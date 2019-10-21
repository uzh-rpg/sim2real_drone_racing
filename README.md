# sim2real_drone_racing

Make some cool intro.

### Installation
Ideally (but optionally) create new catkin workspace.

Install ros melodic. This works on Ubuntu 18.04, other OS are possible but not supported.

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

git clone git@github.com:uzh-rpg/sim2real_drone_racing.git
cd sim2real_drone_racing
cd ..
vcs-import < sim2real_drone_racing/dependencies.yaml

# Build and re-source the workspace
catkin build
. $CATKIN_WS/devel/setup.bash

# Create your learning environment
virtualenv -p python2.7 ./droneflow
source ./droneflow/bin/activate

# If you have a GPU, use the following command. You will need to have CUDA 10.0 installed for it to work.
pip install tensorflow-gpu==1.13.1
# If you don't have a GPU, comment the previous line and uncomment the next
#pip install tensorflow==1.13.1

# Install Required Python dependecies
cd $CATKIN_WS/src/sim2real_drone_racing
pip install -r python_dependencies.txt

```


## Let's Race

Open a terminal and paste the commands below. You shouldn't need a GPU for execution. However, note that if the network can't run at least at 10Hz, you won't be able to fly successfully.

```bash
cd drone_racing_ws
. ./catkin_ddr/devel/setup.bash
. ./droneflow/bin/activate
export CUDA_VISIBLE_DEVICES=''
roslaunch deep_drone_racing_learning  net_controller_launch.launch
```

Open an other terminal and type

```bash
cd drone_racing_ws
. ./catkin_ddr/devel/setup.bash
roslaunch test_racing test_racing.launch

```



## Train your own Sim2Real model

### Generate data

```bash
cd drone_racing_ws
. ./catkin_ddr/devel/setup.bash
roscd drone_racing/resources/scripts
python collect_data.py

```

It is possible to change parameters (number of iteration per background/ gate texture/ etc. ) in the above script.
Defaults should be good.
Once this step is completed, you can train your own network. Optionally, you can use the data we have already collected, available at [this link](train_data.zip)


### Train the Network

```bash
roscd deep_drone_racing_learner/src/ddr_learner

```

Modify the file [train\_model.sh](./learning/deep_drone_racing_learner/src/ddr_learner/train_model.sh) to add the path of validation data collected in the real world, which you can download from [this link](path_to_data.zip).
Then, you can run the following to start data collection.

```bash
./train_model.zsh

```

### Test the Network

Edit the following file to use the checkpoint you just trained
```bash
rosed deep_drone_racing_learning net_controller_launch.launch

```

Now test the network in an environment which was never observed at training time. Instructions are equivalent to the one used above to train the network.

Open the following network and run:

```bash
cd drone_racing_ws
. ./catkin_ddr/devel/setup.zsh
. ./droneflow/bin/activate
export CUDA_VISIBLE_DEVICES=''
roslaunch deep_drone_racing_learning  net_controller_launch.launch

```

Open an other terminal and run

```bash
cd drone_racing_ws
. ./catkin_ddr/devel/setup.zsh
roslaunch test_racing test_racing.launch

```
