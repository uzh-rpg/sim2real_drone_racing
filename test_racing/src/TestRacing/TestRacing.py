#/usr/bin/env python
import rospy
import os
import time

from std_msgs.msg import Empty

class TestRacing(object):
    def __init__(self):

        self.crash_sub = rospy.Subscriber("/crashed", Empty,
                                                self.callback_crashed, queue_size=1)
        self.crash_sub = rospy.Subscriber("/passed_gate", Empty,
                                                self.callback_passed_gate, queue_size=1)
        self.crashed = False
        self.passed_gates = 0
        # Where to log test data
        self.folder_idx = 5000

    def callback_crashed(self, data):
        self.crashed = True

    def callback_passed_gate(self, data):
        self.passed_gates = self.passed_gates + 1
        #print(self.passed_gates)

    def run(self, num_iterations):
        # This function is only for testing, no data generation!
        self.crashed = False

        num_passed_gates = []
        time_before_crash = []

        for i in range(1, num_iterations + 1, 1):
            self.crashed = False
            self.passed_gates = 0
            print("===========================================================")
            print("                       Model-%d                            " % i)
            print("===========================================================")

            print("Replacing quad for new run...")
            os.system("timeout 1s rostopic pub /hummingbird/autopilot/off std_msgs/Empty")

            # reset quad to initial position
            os.system(
                "rosservice call /gazebo/set_model_state '{model_state: { model_name: hummingbird, pose: { position: { x: 0.0, y: 22.0 ,z: 0.2 }, orientation: {x: 0, y: 0, z: -0.707, w: 0.707 } }, twist:{ linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'")

            # start quadrotor
            os.system("timeout 1s rostopic pub /hummingbird/bridge/arm std_msgs/Bool 'True'")
            print("Start quadrotor")
            os.system("timeout 1s rostopic pub /hummingbird/autopilot/start std_msgs/Empty")
            os.system("timeout 1s rostopic pub /hummingbird/run_idx std_msgs/Int16 " + str(self.folder_idx))
            # Network only
            os.system("timeout 1s rostopic pub /hummingbird/only_network std_msgs/Bool 'True'")
            # Network enabled
            os.system("timeout 1s rostopic pub /hummingbird/state_change std_msgs/Bool 'True'")
            # start the navigation
            os.system("timeout 1s rostopic pub /hummingbird/setup_environment std_msgs/Empty")

            start_time = time.time()

            while (time.time() - start_time < 100 and self.crashed == False and self.passed_gates < 30):
                time.sleep(1)

            print("Passed Gates are {}".format(self.passed_gates))
            num_passed_gates.append(self.passed_gates)
            time_before_crash.append(time.time() - start_time)

            print("Experiment finished")
            # Stop trajectory generation
            os.system("timeout 1s rostopic pub /hummingbird/hard_stop std_msgs/Empty")
            # Network disabled
            os.system("timeout 1s rostopic pub /hummingbird/state_change std_msgs/Bool 'False'")
            time.sleep(1)

        # endfor
        return num_passed_gates, time_before_crash
