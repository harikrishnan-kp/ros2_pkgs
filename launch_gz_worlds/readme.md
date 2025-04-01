# Launch gazebo worlds
- A package for launching gazebo using desired .sdf file
- Also support sdf file with local model dependencies(ie, sdf files using model files downloaded from fuel)

## How to use
- build pkg
- Run launch file, This will launch gazebo with a default sdf file (included pkg)
```bash
ros2 launch launch_gz_worlds loadworld.launch.py
```
## How to use for world files which is not in this pkg
- If you are using sdf file with local dependencies,add your depndencies to models folder before building 
- build pkg and run launch file
```bash
ros2 launch launch_gz_worlds loadworld.launch.py  world_file:= < your file path>
```
## Additional information 
- This package is created for studying how we can develop launch file for gazebo world launching.
- you can also run gazebo with a dedicated world file using the following methods
    - from gazebo CLI
    ```bash
    gz sim <path of world file>
    ```
    - from ros2 CLI, using launch file of `ros_gz_sim` package(this package will install automatically with gazebo)
    ```
    ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="< your model file path >"
    ```
- NOTE: if your world file is using any local dependencies, don't forget to set `GZ_SIM_RESOURCE_PATH` environment variable

## directory details
- launch : launch files 
- models: model files used in world file(.sdf) 
- world: world files for gazebo

# TODO
- verify launch argument is working perfectly (verified)
- resolve issues related to sdf file with local model dependencies(not solved yet)
- create a two world files, one with online and local models and one with only local models(usefull when no internet)