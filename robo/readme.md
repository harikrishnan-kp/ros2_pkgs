# readme
- This package helps to visualize the robot model described by urdf file in rviz.
- also you can use this package to understand how position of `geometrical center` and `coordinate system` assosiatd with a robot link changes with change in **< origin />** tag present in link and joint decription section of urdf file
- go through each robot model one by one, start from box_robo.urdf and move down in the order.

## imp observations
- axis colour scheme in rviz
    - red axis   = x-axis
    - green axis = y-axis
    - blue axis  = z-axis
- understand the concept of `roll`, `pitch` and `yaw`: https://www.youtube.com/watch?v=pQ24NtnaLl8
- understand the concept of `link` and `joint` in a robot
- extrution of a link(making 3D from 2D) is always along z axis
- coordinate system assosiated with a link is called as `link frame`
- `origin of link frame` and `geometrical centre` of a link is in same position by default
- **< origin />** tag present inside link description section of urdf is to modify the position of geometrical center of link with respect to the link frame
- **< origin />** tag present inside joint description section of urdf is to modify the position of `child link frame` with respect to `parent link frame`
- by default child link frame have the same orientation as the parent frame

## how to use
- build the package
```bash
colcon build --packages-select robo
```
- by default the package is configured to show box_robo.urdf, if you want to see other robo files, add the model path as argument during launch
```bash
ros2 launch robo display.launch.py model:=urdf/<urdf file name>
```

## TODO 
create new launch file without using the urdf_launch execution dependency 