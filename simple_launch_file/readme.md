# simple launch file
- a package for learning simple launch file
- this package use prebuild nodes in ros2
    - talker, from package demo_node_cpp
    - listener, from package demo_nodes_py

run the launch file
```bash
ros2 launch simpleLaunchFile simple.launch.py 
```

output
```bash
[INFO] [launch]: All log files can be found below /home/hari/.ros/log/2025-03-19-19-32-03-034358-hari-MS-7E44-29770
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [talker-1]: process started with pid [29773]
[INFO] [listener-2]: process started with pid [29774]
[talker-1] [INFO] [1742392924.130952774] [talker]: Publishing: 'Hello World: 1'
[listener-2] [INFO] [1742392924.152992174] [listener]: I heard: [Hello World: 1]
[talker-1] [INFO] [1742392925.130963880] [talker]: Publishing: 'Hello World: 2'
[listener-2] [INFO] [1742392925.132890957] [listener]: I heard: [Hello World: 2]
[talker-1] [INFO] [1742392926.130947114] [talker]: Publishing: 'Hello World: 3'
[listener-2] [INFO] [1742392926.132732396] [listener]: I heard: [Hello World: 3]
[talker-1] [INFO] [1742392927.130939943] [talker]: Publishing: 'Hello World: 4'
[listener-2] [INFO] [1742392927.133164918] [listener]: I heard: [Hello World: 4]
[talker-1] [INFO] [1742392928.130949350] [talker]: Publishing: 'Hello World: 5'
```