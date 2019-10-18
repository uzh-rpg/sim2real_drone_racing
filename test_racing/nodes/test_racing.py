#!/usr/bin/env python

import rospy
from TestRacing import TestRacing
import os, datetime
import sys
import time
import subprocess
from imutils import paths
import gflags
import json

def run_testing():
    full_param_name = rospy.search_param('ddr_dir')
    ddr_dir = rospy.get_param(full_param_name)
    ddr_dir = ddr_dir['test_racing']['ddr_dir']
    ddr_dir = os.path.join(ddr_dir, '..')
    ddr_dir = os.path.abspath(ddr_dir)
    print(ddr_dir)


    rospy.init_node('test_racing', anonymous=True)

    test_racing = TestRacing.TestRacing()

    bkg_folder= os.path.join(ddr_dir, 'drone_racing/drone_racing/resources/race_track/iros_materials/materials/textures/test_bkgs')
    assert os.path.isdir(bkg_folder), "Not found bkg folder"
    floor_folder= os.path.join(ddr_dir, 'drone_racing/drone_racing/resources/race_track/iros_materials/materials/textures/test_floor_bkgs')
    assert os.path.isdir(floor_folder), "Not found floor bkg folder"
    texture_goal_fname= os.path.join(ddr_dir, 'drone_racing/drone_racing/resources/race_track/iros_materials/materials/textures/sky.jpg')
    texture_floor_fname= os.path.join(ddr_dir, 'drone_racing/drone_racing/resources/race_track/iros_materials/materials/textures/asphalt.jpg')
    gate_material_folder=os.path.join(ddr_dir, 'drone_racing/drone_racing/resources/race_track/iros_materials/materials/textures/gate_bkgs')
    bkg_goal_fname=os.path.join(ddr_dir, 'drone_racing/drone_racing/resources/race_track/real_world/gate/meshes/images.jpeg')

    assert os.path.isfile(texture_goal_fname), "Not found bkg_file: %s" % texture_goal_fname
    assert os.path.isfile(texture_floor_fname), "Not found floor_bkg_file: %s" % texture_floor_fname

    num_iterations_per_bkg = 10

    # Read all the backgrounds and order them
    all_images = paths.list_images(bkg_folder)
    all_images = sorted(all_images)

    result_dict = {'gate_pass': {}, 'exec_time': {}}

    # Write final result dictionary
    timestr = time.strftime("%Y%m%d-%H%M%S")
    result_dir = os.path.join(ddr_dir, 'test_racing/results/',
                              timestr)
    if not os.path.isdir(result_dir):
        os.makedirs(result_dir)
    results_fname = os.path.join(result_dir, "evaluation.json")

    # Read all possible gate materials
    all_gates_materials = paths.list_images(gate_material_folder)
    all_gates_materials = sorted(all_gates_materials)

    # Read all possible floor materials
    all_floor_materials = paths.list_images(floor_folder)
    all_floor_materials = sorted(all_floor_materials)

    for i, bkg_img_fname in enumerate(all_images):
        # Copy new background
        os.system("cp {} {}".format(bkg_img_fname, texture_goal_fname))
        # Copy new gate background
        os.system("cp {} {}".format(all_gates_materials[-1], bkg_goal_fname)) # Use the first 9 for training and the last for testing
        # Copy new floor texture
        os.system("cp {} {}".format(all_floor_materials[-1], texture_floor_fname)) # Use the first 9 for training and the last for testing

        print("Processing Background {}".format(i))
        time.sleep(2)
        # set environment
        subprocess.call("roslaunch drone_racing simulation_no_quad_gui.launch &", shell=True)
        time.sleep(10)
        passed_gates, exec_times = test_racing.run(num_iterations_per_bkg)
        assert len(passed_gates) == num_iterations_per_bkg, "wrong output size for gate"
        assert len(exec_times) == num_iterations_per_bkg, "wrong output size for time"
        result_dict['gate_pass'][bkg_img_fname] = passed_gates
        result_dict['exec_time'][bkg_img_fname] = exec_times

        # Killing is kinda sensitive
        time.sleep(5)
        os.system("pkill -9 rviz; pkill -9 gzserver")

        # Overloads every time in case of breakdowns
        time.sleep(1)
        with open(results_fname, 'w') as outfile:
            json.dump(result_dict, outfile)


if __name__ == "__main__":
    run_testing()
