# Fuzz Testing

A simple grammar fuzzer for [SMACH](http://wiki.ros.org/smach) behavior. Given a configuration file and a State machine it can test your behavior with multiple different values for userdatas.

It contains a small example on how to use the fuzzer with a behavior example.

## Setting up

Before to run the fuzzer is necessary to generate the configuration file (a JSON file) as follow:
```
{
  "<state_name>": {
	  "params":{
      "key":["<params_key_1>",...,"<params_key_n>"],
      "types":["<params_type_1>",...,"<params_type_n>"]
    },
    "data":{
      "keys":["<input_key_1>",...,"<input_key_n>"],
      "types":["<input_type_1>",...,"<input_type_n>"]
    }
  }
}

```

