#!/usr/bin/env python
__author__ = "Rodrigo Delgado"

import rospy
import os, rospkg, sys
import smach
from smach.user_data import UserData
import smach_ros
import string
import random
import re
import json, ast
import time
from threading import Thread
from datetime import datetime
from semu_skills import robot_factory
from uchile_robocup.Tour_Guide import TourGuide
from uchile_robocup.CocktailParty import cocktail_party

CHARACTERS = (string.ascii_lowercase
                            + ' ')

START_SYMBOL = "<start>"

RE_NONTERMINAL = re.compile(r'(<[^<> ]*>)')


class ExpansionError(Exception):
        pass

class Fuzzer():   
    def __init__(self, config = None):
        self.config = config
    
    def srange(self, characters):
        """Construct a list with all characters in the string"""
        return [c for c in characters]

    def crange(self, character_start, character_end):
        return [chr(i)
                for i in range(ord(character_start), ord(character_end) + 1)]

    def nonterminals(self, expansion):
        # In later chapters, we allow expansions to be tuples,
        # with the expansion being the first element
        if isinstance(expansion, tuple):
            expansion = expansion[0]

        return re.findall(RE_NONTERMINAL, expansion)

    def simple_grammar_fuzzer(self, grammar, start_symbol=START_SYMBOL,
                              max_nonterminals=10, max_expansion_trials=1000,
                              log=False):
        term = start_symbol
        expansion_trials = 0

        while len(self.nonterminals(term)) > 0:
            symbol_to_expand = random.choice(self.nonterminals(term))
            expansions = grammar[symbol_to_expand]
            expansion = random.choice(expansions)
            new_term = term.replace(symbol_to_expand, expansion, 1)

            if len(self.nonterminals(new_term)) < max_nonterminals:
                term = new_term
                if log:
                    print("%-40s" % (symbol_to_expand + " -> " + expansion), term)
                expansion_trials = 0
            else:
                expansion_trials += 1
                if expansion_trials >= max_expansion_trials:
                    raise ExpansionError("Cannot expand " + repr(term))

        return term.encode("utf-8")

    def data_to_userdata(self, data):
    	userdata = UserData()
    	for k in data.keys():
    		userdata[k] = data[k]
    	return userdata

    def userdata_from_set(self, data, state = None):
        config = self.config
        userdata = UserData()
        log = {}
        for k in data:
            value = "<value>"
            grammar = GRAMMAR
            if state!=None and config!=None and state in config:
                if k in config[state]["keys"]:
                    index = config[state]["keys"].index(k)
                    value = config[state]["types"][index]
                    if "grammar" in config[state]:
                        grammar = config[state]["grammar"] 
            userdata[k] = self.simple_grammar_fuzzer(grammar, value)
            log[k] = userdata[k]
        return userdata, log

    def run_with_save_data(self, state, label):
        log = {}
        data, log["input_data"] = self.userdata_from_set(state._input_keys, state=label)
        log["pre_params"] = self.get_params()
        start = time.time()
        try:
            log["output"] = state.execute(data)
        except Exception as e:
            log["error"] = e.message
            rospy.loginfo(data)
            rospy.logerr('error: ' + e.message)
        end = time.time()
        log["post_params"] = self.get_params()
        log["time"] = str(end-start)
        return log

    def fuzz_params(self):
        self.fuzz_param('reached', ('y','n'))
        self.fuzz_param('is_moving', ('n','y'))
        self.fuzz_param('hear', self.simple_grammar_fuzzer(GRAMMAR,"<string_array>"))
        self.fuzz_param('num_faces', self.simple_grammar_fuzzer(GRAMMAR,"<int_array>"))
        self.fuzz_param('moved', ('y','n'))
        self.fuzz_param('detected_faces', ('y','n'))
        self.fuzz_param('saved_faces', ('y','n'))
        self.fuzz_param('recognized_faces', ('y','n'))
        return map(lambda x: (x,rospy.get_param(x)),rospy.get_param_names())
        #rospy.set_param('recognized_name','test_name')
        #rospy.set_param('is_door_open','y')
        #rospy.set_param('is_object_detected','y')
        #rospy.set_param('object_detected','y')

    def fuzz_param(self, param, values):
        rospy.set_param(param, random.choice(values))

    def get_params(self):
        return map(lambda x: (x,rospy.get_param(x)),rospy.get_param_names())

    def fuzz_ros_params(self):
        while True:
            self.fuzz_params()
            time.sleep(1)

    def start_fuzzing_params(self):
        worker = Thread(target=self.fuzz_ros_params)
        worker.setDaemon(True)
        worker.start()

GRAMMAR = {
        "<start>":["{<userdata>}"],
        "<userdata>":["<vars>,<userdata>","<vars>"],
        "<vars>":["<key>:<element>"],
        "<key>":['<string>'],
        "<array>": ["<string_array>","<number_array>","<boolean_array>"],
        "<string_array>": ["[<string_elements>]"],
        "<number_array>": ["[<number_elements>]"],
        "<int_array>": ["[<int_elements>]"],
        "<boolean_array>": ["[<boolean_elements>"],
        "<string_elements>": ["<string>,<string_elements>","<string>"],
        "<number_elements>": ["<int_elements>","<float_elements>","<mixed_elements>"],
        "<boolean_elements>": ["<boolean>,<boolean_elements>","<boolean>"],
        "<int_elements>": ["<int>,<int_elements>","<int>"],
        "<float_elements>": ["<float>,<float_elements>","<float>"],
        "<mixed_elements>": ["<number>,<mixed_elements>","<number>"],
        "<boolean>":["True","False"],
        "<element>": ["<string>","<number>","<boolean>"],
        "<value>": ["<array>","<element>"],
        "<number>": ["<int>","<float>"],
        "<int>":["<digit><int>","<digit>"],
        "<float>":["<int>.<int>"],
        "<digit>": Fuzzer().crange('1','9'),
        "<string>": ['"' + "<characters>" + '"'],
        "<characters>": ["<character><characters>","<character>"],
        "<character>": Fuzzer().srange(CHARACTERS)
    }

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
    rospy.init_node('test')
    rp = rospkg.RosPack()
    config_path = os.path.join(rp.get_path("uchile_states"), "src", "uchile_states", "test", "fuzzing", "configuration", "config.json")
    dirName = str(datetime.now())
    if len(sys.argv) > 1:
        report_path = sys.argv[1]
    else:
        report_path = os.path.join(rp.get_path("uchile_states"), "src", "uchile_states", "test", "fuzzing", "report")
    if not os.path.exists(report_path):
        os.makedirs(report_path)
        print("Directory " , dirName ,  " Created ") 
    set_params()
    with open(config_path) as f:
        config = json.load(f)
    fuzzer = Fuzzer(config)
    robot = robot_factory.build(base_skills + core + tools)
    sm = TourGuide.getInstance(robot)
    #sm3 = correct_order.getInstance(robot)
    #sm2 = init.getInstance(robot)
    #sm4 = place_corrected_order.getInstance(robot)
    #sm5 = place_orders.getInstance(robot)
    #sm6 = take_orders.getInstance(robot)
    #machines = [sm._states, sm2._states, sm3._states, sm4._states, sm5._states, sm6._states ]
    machines = [sm._states]
    fuzzer.start_fuzzing_params()
    for states in machines:
        for label in states:
            rospy.loginfo('starting with state: ' + label)
            state = states[label]
            for i in range(10):
                if not  os.path.exists(os.path.join(report_path, label)):
                    os.makedirs(os.path.join(report_path, label))
                log = fuzzer.run_with_save_data(state, label)
                with open(os.path.join(report_path, label, dirName + "_report.json"), 'a+') as outfile:  
                    json.dump(log, outfile)
                    outfile.write("\n")
                