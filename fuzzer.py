import string
import random
import re
import rospy
import time

from smach.user_data import UserData
from threading import Thread
from datetime import datetime

RE_NONTERMINAL = re.compile(r'(<[^<> ]*>)')

def srange(characters):
    """Construct a list with all characterrs in the string"""
    return [c for c in characters]

def crange(character_start, character_end):
    return [chr(i)
        for i in range(ord(character_start), ord(character_end) + 1)]

def nonterminals(expansion):
    # In later chapters, we allow expansions to be tuples,
    # with the expansion being the first element
    if isinstance(expansion, tuple):
        expansion = expansion[0]

    return re.findall(RE_NONTERMINAL, expansion)

def data_to_userdata(data):
    userdata = UserData()
    for k in data.keys():
        userdata[k] = data[k]
    return userdata

class ExpansionError(Exception):
        pass

class Fuzzer():

    start_symbol = "<start>"
    characters = (string.ascii_lowercase + ' ')
    grammar = {
            "<start>":["{<userdata>}"],
            "<userdata>":["<vars>,<userdata>","<vars>"],
            "<vars>":["<key>:<element>"],
            "<key>":['<string>'],
            "<array>": ["<string_array>","<number_array>","<boolean_array>"],
            "<string_array>": ["[<string_elements>]"],
            "<number_array>": ["[<number_elements>]"],
            "<int_array>": ["[<int_elements>]"],
            "<boolean_array>": ["[<boolean_elements>]"],
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
            "<digit>": crange('1','9'),
            "<string>": ['"' + "<characters>" + '"'],
            "<characters>": ["<character><characters>","<character>"],
            "<character>": srange(characters)
        }

    def __init__(self, config = None):
        self.config = config
        
    def simple_grammar_fuzzer(self, start_symbol=start_symbol,
                              max_nonterminals=10, max_expansion_trials=1000,
                              log=False):
        return self.specific_grammar_fuzzer(self.grammar, start_symbol, 
            max_nonterminals, max_expansion_trials, log)

    def specific_grammar_fuzzer(self, grammar, start_symbol=start_symbol, 
            max_nonterminals=10, max_expansion_trials=1000, log=False):
        term = start_symbol
        expansion_trials = 0

        while len(nonterminals(term)) > 0:
            symbol_to_expand = random.choice(nonterminals(term))
            expansions = grammar[symbol_to_expand]
            expansion = random.choice(expansions)
            new_term = term.replace(symbol_to_expand, expansion, 1)

            if len(nonterminals(new_term)) < max_nonterminals:
                term = new_term
                if log:
                    print("%-40s" % (symbol_to_expand + " -> " + expansion), term)
                expansion_trials = 0
            else:
                expansion_trials += 1
                if expansion_trials >= max_expansion_trials:
                    raise ExpansionError("Cannot expand " + repr(term))
                    
        try:
            return eval(term.encode("utf-8"))
        except Exception:
            return term.encode("utf-8")

    def userdata_from_set(self, data, state = None, grammar = grammar):
        userdata = UserData()
        log = {}
        for k in data:
            value = "<value>"
            if state!=None and self.config!=None and state in self.config:
                if k in self.config[state]["data"]["keys"]:
                    index = self.config[state]["data"]["keys"].index(k)
                    value = self.config[state]["data"]["types"][index]
                    if "grammar" in self.config[state]["data"]:
                        grammar = self.config[state]["data"]["grammar"] 
            if 'list' in k:
                value = "<array>"
            elif 'text' in k:
                value = "<string>"
            userdata[k] = self.specific_grammar_fuzzer(grammar, value)
            log[k] = userdata[k]
        if state!=None and self.config!=None and state in self.config and "keys" in self.config[state]["params"]:
            for k in self.config[state]["params"]["keys"]:
                if "grammar" in self.config[state]["params"]:
                    grammar = self.config[state]["params"]["grammar"]
                index = self.config[state]["params"]["keys"].index(k)
                self.fuzz_param(k,self.specific_grammar_fuzzer(grammar, self.config[state]["params"]["types"][index]))
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
            rospy.loginfo(log["input_data"])
            rospy.logerr('error: ' + e.message)
        end = time.time()
        log["post_params"] = self.get_params()
        log["time"] = str(end-start)
        return log

    def fuzz_params(self):
        self.fuzz_param('reached', ('y','n'))
        self.fuzz_param('is_moving', ('n','y'))
        self.fuzz_param('hear', ('yes','no','algo'))
        self.fuzz_param('num_faces', self.simple_grammar_fuzzer("<int_array>"))
        self.fuzz_param('moved', ('y','n'))
        self.fuzz_param('detected_faces', ('y','n'))
        self.fuzz_param('saved_faces', ('y','n'))
        self.fuzz_param('recognized_faces', ('y','n'))
        self.fuzz_param('names',('name1','name2',['name1','name2']))
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
            time.sleep(0.01)

    def start_fuzzing_params(self):
        worker = Thread(target=self.fuzz_ros_params)
        worker.setDaemon(True)
        worker.start()