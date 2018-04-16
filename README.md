Copyright 2018 LAAS-CNRS



# HPP-timeopt-corba

This is module to implement python bindings for hpp-timeoptimization(https://github.com/ggroy15/hpp-timeoptimization), and presents a example file. 

Install
----
To install this moulde: 

  1. Install HPP 
	- see https://github.com/humanoid-path-planner/hpp-doc
	
  2. Install HPP-Timeoptimization
	- see https://github.com/ggroy15/hpp-timeoptimization

  3. Use CMake to install the library. For instance:
   ```bash
   mkdir $hpp-timeopt-corba/build
   cd $hpp-timeopt-corba/build
   cd cmake ..
   make install
   ```

Demo
----
To see the planner in action, one example with DYROS-red is available.

  1. Run gepetto-gui
     ```
	gepetto-gui
     ```

  2. Run hpp-timeopt-corbaserver
     	```
	$ hpp-timeopt-server
	```

  3. Run script in script folder
	```
	$ ipython demo.py
	```


