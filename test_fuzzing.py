#!/usr/bin/env python
__author__ = "Rodrigo Delgado"

import rospy
import os, rospkg, sys
import smach
import smach_ros
import json, ast
from fuzzer import Fuzzer
from datetime import datetime
from semu_skills import robot_factory
#from uchile_robocup.Farewell import Farewell  as state_machine
from uchile_robocup.Tour_Guide import TourGuide as state_machine
#from uchile_robocup.CocktailParty import correct_order as state_machine
#from uchile_robocup.CocktailParty.cp_states import find_calling_person
#from uchile_robocup.FindMyMates import init
#from uchile_robocup.Receptionist.states import find_seat
#from uchile_states.base import states

def set_params():
    rospy.set_param('services', True)
    rospy.set_param('~/drinks',["coca"])
    os.environ['FACE_DETECTOR']="external"
    rospy.set_param('reached', 'y')
    rospy.set_param('is_moving', 'n')
    rospy.set_param('touch', 'y')
    rospy.set_param('hear','yes')
    rospy.set_param('time_to_sleep', 0.1)
    rospy.set_param('num_faces',0)
    rospy.set_param('moved', 'y')
    rospy.set_param('detected_faces','y')
    rospy.set_param('saved_faces','y')
    rospy.set_param('recognized_faces','y')
    rospy.set_param('recognized_name','test_name')
    rospy.set_param('is_door_open','y')
    rospy.set_param('is_object_detected','y')
    rospy.set_param('object_detected','y')

base_skills = [
    "audition",
    "follow",
    "navigation",
    "sound_localization",
    "tablet",
    "tabletapp"
]
manipulation = [
    "capability_map",
    "manipulation",
    "octomap",
    "wait_bag"
]
perception = [
    "facial_features",
    "door_open_detector",
    "shelf_pose",
    "object_recognition",
    "person_detector",
    "sitting_person_detector",
    "track_person",
    "waving_deep",
    "wave_detection"
]
tools = [
    "display_interface",
    "report_generator"
]
core = [
    'base',
    'face',
    'knowledge',
    'l_arm',
    'l_gripper',
    'neck',
    'r_arm',
    'r_gripper',
    'sound',
    'tts'
]

if __name__ == "__main__":
    # Setting folders for logs.
    rospy.loginfo("Setting up folder to save logs.")
    rospy.init_node('test')
    rp = rospkg.RosPack()
    config_path = os.path.join(rp.get_path("uchile_states"), "src", "uchile_states", "test", "fuzz", "configuration", "config.json")
    dirName = str(datetime.now())
    if len(sys.argv) > 1:
        report_path = sys.argv[1]
    else:
        report_path = os.path.join(rp.get_path("uchile_states"), "src", "uchile_states", "test", "fuzz", "report")
    if not os.path.exists(report_path):
        os.makedirs(report_path)
        print("Directory " , dirName ,  " Created ") 
    set_params()
    with open(config_path) as f:
        config = json.load(f)
    rospy.loginfo("Building Fuzzer with configuration.")
    
    fuzzer = Fuzzer(config)


    skills = base_skills + core
    rospy.loginfo("Building robot with skills: ".join(skills))
    robot = robot_factory.build(base_skills+core)
    sm = state_machine.getInstance(robot)
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    #sm.execute()
    #rospy.spin()
    machines = [sm._states]
    fuzzer.start_fuzzing_params()
    for states in machines:
        for label in states:
            rospy.loginfo('starting with state: ' + label)
            state = states[label]
            if len(state._input_keys) > 0:
                repeat = 50
            else:
                repeat = 1
            for i in range(repeat):
                if not  os.path.exists(os.path.join(report_path, label)):
                    os.makedirs(os.path.join(report_path, label))
                log = fuzzer.run_with_save_data(state, label)
                with open(os.path.join(report_path, label, dirName + "_report.json"), 'a+') as outfile:  
                    json.dump(log, outfile)
                    outfile.write("\n")

    sis.stop()