### Pump
A simple library for Arduino to handle home-pool filtration and persitaltic pumps

Version 1.0.0

#### Features: 

* keeps track of running time
* keeps track of Tank Level
* set max running time limit
* prevents pump to run if running time limit is reached or tank level is low
* interlock option to prevent pump from running if interlock level is low (use example is when filtration pump is not ON, Orp and pH pumps may not run) 
* option to use a separate (input) sensor pin to sense if pump is running from the actual output pin to start/stop the pump. This is useful for instance 
when an external system is managing the start/stop of the pump and we just need to know whether it is running or not
