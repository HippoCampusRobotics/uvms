# Trajectory Tracking for Underwater vehicle-manipulator systems

This package contains an end effector trajectory tracking framework for a UVMS consisting of a BlueROV2 and a Reach Robotics Alpha 5 Arm.


# Paper and Video

Feel free to use our code for your own work.
If you find this useful for your research, please consider citing our paper and sending us a message.


* Niklas Trekel, Nathalie Bauschmann, Thies L Alff, Daniel A Duecker, Sami Haddadin, and Robert Seifried, "**Dynamic End Effector Trajectory Tracking for Small-Scale Underwater Vehicle-Manipulator Systems (UVMS): Modeling, Control, and Experimental Validation**". *2025 IEEE International Conference on Robotics and Automation (ICRA)*. IEEE, 2025. 
  ```bibtex
  @inproceedings{Trekel2025,
  title = {Dynamic End Effector Trajectory Tracking for Small-Scale Underwater Vehicle-Manipulator Systems (UVMS): Modeling, Control, and Experimental Validation},  
  author = {Trekel, Niklas and Bauschmann, Nathalie and Alff, Thies L and Duecker, Daniel A and Haddadin, Sami and Seifried, Robert},  
  year  = {2025},
  month = {May},
  publisher = {{IEEE}},
  booktitle = {2025 International Conference on Robotics and Automation (ICRA)}
  }
  ```
  **Video:** https://www.youtube.com/watch?v=IDMlI5KqlVI

  * Nathalie Bauschmann, Vincent Lenz, Robert Seifried and Daniel A Duecker, "**Experimental Open-Source Framework for Underwater Pick-and-Place Studies with Lightweight UVMS -- An Extensive Quantitative Analysis**". *2025 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2025)*. IEEE, 2025. 
    ```bibtex
    @inproceedings{Bauschmann2025,
    title = {Experimental Open-Source Framework for Underwater Pick-and-Place Studies with Lightweight UVMS -- An Extensive Quantitative Analysis},  
    author = {Bauschmann, Nathalie and Lenz, Vincent and Seifried, Robert and Duecker, Daniel A},  
    year  = {2025},
    month = {October},
    publisher = {{IEEE}},
    booktitle = {2025 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}
    }
    ```
    **Video:** https://youtu.be/C2BV5Mmz6VE

# Pick-and-Place Framework

This framework can also be used for pick-and-place tasks underwater.

**Video:** https://youtu.be/RGsInnwlMCQ

# Installation

Installation instructions for Linux (Ubuntu 24.04, ROS2 Jazzy)




If not already done so, install ROS2 (Desktop-full is recommended) and setup a workspace.
You could use [our guide](https://hippocampusrobotics.github.io/docs/contents/getting_started/ros_installation.html). 


This package depends on some of our other ROS2 packages.
They can be installed from source, or more conveniently, via apt.

Follow [this guide](https://hippocampusrobotics.github.io/docs/contents/getting_started/pre-built_packages.html) for how to add the key + sources to apt and rosdep.

Then, you should be able to install our packages via apt:
```shell script
sudo apt install ros-${ROS_DISTRO}-hippo-full
```

Finally, get this repository (e.g. via SSH)
```shell script
git clone git@github.com:HippoCampusRobotics/uvms.git
```

**Currently tested: please use the jazzy branch!**

Build & source the workspace.


### Alpha 5 Arm

In order to run the simulation, you will need an additional repository containing the Alpha 5 model. Since these parameters are proprietary, it is set to private. 
If you send us proof that you own a Alpha 5 manipulator, we can give you access!

Unfortunately, for now, we do not have a better solution to this problem.



# Starting the simulation


### UVMS

In order to start the simulation with the full UVMS, run:
```
ros2 launch uvms_sim uvms_sim.launch.py vehicle_name:=klopsi00
```
The control framework can then be started using:
```
ros2 launch uvms_kinematic_ctrl top_uvms_sim_complete.launch.py vehicle_name:=klopsi00
```

### BlueROV
To only start a simulation with the AUV, run:
```
ros2 launch hippo_sim top_bluerov_complete.launch.py vehicle_name:=klopsi00
```
The control framework can then be started using:
```
ros2 launch bluerov_ctrl top_bluerov_sim_complete.launch.py vehicle_name:=klopsi00
```
