import os
import numpy as np
import time
import subprocess
import shutil
from imutils import paths

# set up simulation
TRAIN_DIR="../../../../learning/deep_drone_racing_learner/data/Training"

def collect_data_in_fixed_env(num_iterations):
    runs_dir = os.listdir(TRAIN_DIR)
    runs_dir = [ d for d in runs_dir if (os.path.isdir(os.path.join(TRAIN_DIR,d)) and 'Run' in d)]
    runs_dir = sorted(runs_dir)
    if len(runs_dir) == 0:
        last_index = 0
        first_iteration = True
    else:
        last_index = int(runs_dir[-1].split("_")[-1]) + 1
    print("Setting run index to {} ".format(last_index))

    for _ in range(num_iterations):
        print("=======================")
        print("  Iteration number {}  ".format(last_index))
        print("=======================")
        # set current run index
        os.system("timeout 1s rostopic pub /hummingbird/run_idx std_msgs/Int16 {}".format(last_index))

        # set up simulation scenario
        print('Replacing quad for new run...')
        os.system('timeout 1s rostopic pub /hummingbird/autopilot/off std_msgs/Empty')
        # reset quad to initial position
        os.system("rosservice call /gazebo/set_model_state '{model_state: { model_name: hummingbird, pose: { position: { x: -0.5, y: 22.0 ,z: 0.2 }, orientation: {x: 0, y: 0, z: -0.707, w: 0.707 } }, twist:{ linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'")


        # start quadrotor
        print("Start quadrotor")
        os.system("timeout 1s rostopic pub /hummingbird/bridge/arm std_msgs/Bool 'True'")
        os.system("timeout 1s rostopic pub /hummingbird/autopilot/start std_msgs/Empty")
        time.sleep(3)

        # setup environment and start
        os.system("timeout 1s rostopic pub /hummingbird/setup_environment std_msgs/Empty")

        print("Collecting data...")
        time.sleep(40)

        # stop algorithm
        print("Stop collecting data")
        os.system("timeout 2s rostopic pub /hummingbird/hard_stop std_msgs/Empty")

        # increase run index by one
        last_index += 1

def main():
    bkg_folder='../race_track/iros_materials/materials/textures/train_bkgs'
    texture_goal_fname='../race_track/iros_materials/materials/textures/sky.jpg'
    asphalt_goal_fname='../race_track/iros_materials/materials/textures/asphalt.jpg'
    gate_material_folder='../race_track/iros_materials/materials/textures/gate_bkgs'
    bkg_goal_fname='../race_track/real_world/gate/meshes/images.jpeg'

    # Gate shapes
    gates_shapes_dir='../race_track/real_world/gate/meshes/gate_shapes'
    all_shapes = [os.path.join(gates_shapes_dir, f.split('.')[0]) \
                  for f in os.listdir(gates_shapes_dir) if f.endswith('.stl')]
    num_gates = len(all_shapes) - 1 # Last is used for testing
    gate_dae='../race_track/real_world/gate/meshes/gate.dae'
    gate_stl='../race_track/real_world/gate/meshes/gate.stl'
    light_changer='../race_track/real_world/gate/meshes/set_gate_properties.py'

    num_iterations_per_bkg = 1
    num_loops = 2

    # Read all the backgrounds and order them
    all_images = paths.list_images(bkg_folder)
    all_images = sorted(all_images)

    # Read all possible gate materials
    all_gates_materials = paths.list_images(gate_material_folder)
    all_gates_materials = sorted(all_gates_materials)

    if not os.path.isdir(TRAIN_DIR):
        os.mkdir(TRAIN_DIR)

    for _ in range(num_loops):
        for i, bkg_img_fname in enumerate(all_images):
            # Copy new background
            os.system("cp {} {}".format(bkg_img_fname, texture_goal_fname))
            # Copy new asphalt
            os.system("cp {} {}".format(all_images[-(i+1)], asphalt_goal_fname))
            # Copy new gate background
            os.system("cp {} {}".format(all_gates_materials[i%9], bkg_goal_fname)) # Use the first 9 for training and the last for testing
            # Copy new gate shape
            gate_number = np.random.choice(num_gates)
            shutil.copy(all_shapes[gate_number]+ '.stl', gate_stl)
            shutil.copy(all_shapes[gate_number]+ '.dae', gate_dae)
            # Make random illumination
            os.system("python {} -xml_file {} -emission {} -ambient {}".format(
                light_changer,
                gate_dae,
                0.1*np.random.rand(), # Gates have little emission, 0.3
                np.random.rand())) # 0.5


            print("Processing Background {}".format(i))
            time.sleep(2)
            # set environment
            subprocess.call("roslaunch drone_racing simulation_no_quad_gui.launch &", shell=True)
            time.sleep(10)
            collect_data_in_fixed_env(num_iterations_per_bkg)
            os.system("pkill -9 rviz; pkill -9 gzserver")
            time.sleep(5)

if __name__ == '__main__':
    main()
