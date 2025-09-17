# Viethen

## Important Information

| Gateway IP | PC IP | Robot IP | Cam IP | Subnet Mask |
| --- | --- | --- | --- | --- |
| 172.28.60.1 | 172.28.60.2 | 172.28.60.10 | 172.28.60.5 | 255.255.255.0 |



## How to Start the Robot

1. Transformator is Preset to 32V
2. Wait to Tranformator to Reach Voltage
3. Turn on the Pad
4. Press "Not Now" at the Safety warning
5. Press "Run" on the upper right corner
6. Select File
7. Press "Open"
8. At the down left Corner press launch
9. Press "Start"
10. Press "Release Breaks"
11. The Programm can now be launced with the play button at the Bottom.

> **_NOTE:_**  Only Plug in the Camera if needed, because it does not have a on or off switch.  

> **_NOTE:_**  Only Plug in the *USB* after Boot is finished.

If the Button on the back of the Tablet is Pressed the Robot Arm can be Moved freely.

## How to Launch Calibration
---
> **_WIP:_** This section is not completed and does not work
---
1. Load Script onto Robot by unsing USB-Stick
2. Launch Script
4. cd to Subfolder wich contains ```universal_robots_perfom_hand_eye_calibration```
3. Launch Python Script with ```python universal_robots_perfom_hand_eye_calibration.py --eih --ip 172.28.60.10```

### 3D Druck:
Shrinks down after print: -0.5 mm to -0.6mm. Prusa Slicer is recomended.
> **_NOTE:_**  Close Windows inorder to reduce Heat reduction.


### Link Collection
- https://support.zivid.com/en/latest/getting-started/software-installation/zivid-two-network-configuration.html
- https://github.com/zivid/zivid-python-samples
- https://support.zivid.com/en/latest/academy/applications/point-cloud-tutorial.html
- https://github.com/zivid/zivid-ros
- https://github.com/ros-industrial-consortium/bezier
- https://rosindustrial.org/scan-n-plan
- https://gazebosim.org/home
- https://github.com/csviragh/robotsim
- https://pybullet.org/wordpress/
- https://pypi.org/project/modbus-tk/
- https://www.modbustools.com/modbus_slave.html
- https://github.com/ljean/modbus-tk/
- https://forum.universal-robots.com/t/modbus-tcp-connection-between-robot-and-pc/19728/5
- https://www.universal-robots.com/articles/ur/interface-communication/modbus-server/

---
This Readme is Authored by @tw5095s


### Triggering simulation environment with ROS2 interface
1. Step 1: Clone the SRC directory in your ROS2 worksace (humble)

2. Step 2: install webots_ros2 interface, refer the below link for instattion of the interface 
  https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html

3. step 3: Build workspace and launch environment with following commands

4. Launch the simultaion world
  `ros2 launch ur5e_simulation robot_launch.py`

5. Launch moveit environment
  `ros2 launch ur5e_simulation moveit_launch.py`
  
  
### Getting the Models

We only have the entire setup as a cad m1odel but we only want to have the spoiler for our bezier courve thing.

We have some scripts for that one first you need to separate the existing one by running: 

```python scripts/convert.py WebotsProjekt/cad/Schleifvorrichtung_V3_TV.obj spoiler-setup.stl```

```
python scripts/seperate.py -i spoiler-setup.stl -o tmp
```
Open all the files look at them and find the spoiler (navigate with left and right arrow)
```
f3d models/*
```
delte the temp folder after copying it:

```
cp tmp/mesh_parth_8.stl spoiler.stl
rm -r tmp
rm spoiler-setup.stl
```

Open the spoiler to decide to rotate in which direction:
```
f3d spoiler.stl
```

For example -90 deg on the X-Axis (rotating by 90 will then be the cad file for bottom machining):
```
python scripts/rotate.py -i spoiler.stl -o spoiler-rotated.stl --rotate_x -90 --rotate_y 0 --rotate_z 0
```

(optional) Center the spoiler for ease of use when setting the correct position:
```
python scripts/center.py -i spoiler-rotated.stl -o spoiler-center.stl
```

After verifying the model copy it to the viethen_node models folder.

## Verifying the settings
We can only grind from one side at a time, so we need to decide which surfaces to grind and which not to. During the scan, we likely capture all the surfaces that are reachable, but when using a CAD model, we need to set an arbitrary threshold (based on the angle compared to the up vector) to determine which surfaces are relevant for us.

To visualize this setting:
``` shell
python scripts/visualize_up.py viethen_node/models/spoier.stl
```
Adjust the slider until all the surfaces you want to grind are highlighted in red. Press the 'S' key to switch to a down-facing vector and check the bottom surfaces..
