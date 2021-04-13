# Fuzz Testing

A simple grammar fuzzer for [SMACH](http://wiki.ros.org/smach) behavior. Given a configuration file and a State machine it can test your behavior with multiple different values for userdatas.

It contains a small example on how to use the fuzzer with a behavior example.

If you use a more complex architecture, it can be adapted to work with it. Our work was tested with robot_skills[[1]](#1) and there were no changes to fuzzer.

## Setting up

Before to run the fuzzer is necessary to generate the configuration file (a JSON file) as follow:
```
{
  "state_name": {
	  "params":{
      "key":["params_key_1",...,"params_key_n"],
      "types":["params_type_1",...,"params_type_n"]
    },
    "data":{
      "keys":["input_key_1",...,"input_key_n"],
      "types":["input_type_1",...,"input_type_n"]
    }
  }
}

```

This information can be rewrite each time if more information is obtained from the fuzzer.

The types of parameters/inputs must match the grammar defined by the fuzzer. 
In case you need a more specific grammar such as two or three specific values or some special data type, it can be specified in the configuration file as follow:

```
{
  "state_name": {
    "params":{
      "key":["params_key_1",...,"params_key_n"],
      "types":["params_type_1",...,"params_type_n"]
    },
    "data":{
      "keys":["input_key_1",...,"input_key_n"],
      "types":["input_type_1",...,"input_type_n"],
      "grammar": {
        "<input_type_1>":"value_1",
        "<input_type_2>":"value_2"
      }
    }
  }
}
```

We recommend giving the values at least one pass before executing the fuzzer, as this could reduce the amount of type errors that are generated. 
However, it is possible to test without the configuration file and create it based on errors, if a type generates type error, then that type is not suitable for that parameter.

Each execution of the fuzzer can give us a lot of information, if we execute 100 or 1000 times with random types, it is very likely that we will discover the type of the parameters in an execution.
Then it would be enough to try again with the same type but different values.

## Grammar

Grammar consists of production rules that allow generating different types of data if we take different rules.

The grammar used in this work is very similar to that of a JSON file, where we have types similar to the basic ones in Python.

Similarly, we add some other types as lists of unique type (they are not native to python but are used constantly).

The complete grammar is:

```
{
  "<start>":["{<userdata>}"],
  "<userdata>":["<vars>,<userdata>","<vars>"],
  "<vars>":["<key>:<value>"],
  "<key>":["<string>"],
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
  "<digit>": ["0","1","2","3","4","5","6","7","8","9"],
  "<string>": ["'" + "<characters>" + "'"],
  "<characters>": ["<character><characters>","<character>"],
  "<character>": from 'A' to 'z'
}
```

## Report

The fuzzer reports the information for each state within the state machine. 
They are separated into folders for each state and within these folders a file is generated for each execution of the fuzzer. 
If in an execution of the fuzzer the state is evaluated with 100 different values, each of these evaluations will be saved in one line of the generated file. 
The information saved are the pre and post conditions of the execution, its execution time and also the output of the status or an error if it was generated.

## References
<a id="1">[1]</a> 
DBøgh, S., Nielsen, O. S., Pedersen, M. R., Krüger, V., & Madsen, O. (2012, August). 
Does your robot have skills?. In Proceedings of the 43rd international symposium on robotics (Vol. 6, pp. 1-6). Verlag.
