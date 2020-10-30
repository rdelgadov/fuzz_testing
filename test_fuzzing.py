#!/usr/bin/env python
__author__ = "Rodrigo Delgado"

import rospy
import os, rospkg, sys
import smach
import smach_ros
import json
from fuzzer import Fuzzer
from datetime import datetime

# import your_behavior_state_machine as state_machine

# base skills of bender Robot
base_skills = [
    "audition",
    "follow",
    "navigation",
    "sound_localization",
    "tablet",
    "tabletapp"
]

# Params to be fuzzed as external output.
params= {
    "reached": ("y","n"),
    "is_moving": ("n","y"),
    "hear": ("yes","no","algo"),
    "num_faces": (0,1,2,3,4),
    "moved": ("y","n"),
    "detected_faces": ("y","n"),
    "saved_faces": ("y","n"),
    "recognized_faces": ("y","n"),
    "names": ("name1","name2",["name1","name2"])
}

if __name__ == "__main__":
    # Setting folders for logs.
    source_path = input("Type your ROS proyect name e.g: my_super_behavior.")
    rospy.loginfo("Setting up folder to save logs.")
    rospy.init_node("fuzz_testing")
    rp = rospkg.RosPack()

    # Configure your own path for fuzzer proyect.
    config_path = os.path.join(rp.get_path(source_path), "configuration", "config.json")
    dirName = str(datetime.now())
    if len(sys.argv) > 1:
        report_path = sys.argv[1]
    else:
        report_path = os.path.join(rp.get_path(source_path), "report")
    if not os.path.exists(report_path):
        os.makedirs(report_path)
        print "Directory " + dirName +  " Created" 
    with open(config_path) as f:
        config = json.load(f)
    rospy.loginfo("Building Fuzzer with configuration.")
    

    fuzzer = Fuzzer(config)
    # Anything necesary before the execution of state
    # As built a mock robot or initialize the necessary services.
    #
    # skills = base_skills
    # rospy.loginfo("Building robot with skills: ".join(skills))
    # robot = robot_factory.build(skills)
    
    sm = state_machine
    # Only if you need or want a instronspection server to see the state machine structure:
    # sis = smach_ros.IntrospectionServer("server_name", sm, "/SM_ROOT")
    # sis.start()

    # It can handle multiple state machines, only add to machines and it will test all togethers
    machines = [sm._states]

    # Start the execution on a second thread of fuzz external inputs.
    fuzzer.start_fuzzing_params(params)

    for states in machines:
        for label in states:
            rospy.loginfo("Starting with state: " + label)
            state = states[label]
            if len(state._input_keys) > 0:
                repeat = 100
            else:
                repeat = 1
            for i in range(repeat):
                # Execution of state with fuzz data. Log pre and post conditions.
                if not  os.path.exists(os.path.join(report_path, label)):
                    os.makedirs(os.path.join(report_path, label))
                log = fuzzer.run_with_save_data(state, label)
                with open(os.path.join(report_path, label, dirName + "_report.json"), "a+") as outfile:  
                    json.dump(log, outfile)
                    outfile.write("\n")