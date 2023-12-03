# Starting the Simulation
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