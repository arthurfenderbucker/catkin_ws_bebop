# <center> Bebop Control Node </center>
### PID calibration (Dynamic reconfiguration)
<br/>
run the control node:
```
rosrun dronecontrol control.py
```
<br/>
on a different terminal:

```
rosrun rqt_gui rqt_gui -s reconfigure
```

<br/>on the leftside of the launched gui, select "control" and adjust the PID values on each axis.<br/>
once you are satisfied with the result, click on the save icon and replace the existing file "dronecontrol/config/bebop2_config.yaml" with the new configuration.
<br/>In order to the changes effectively take place, rebuild the package
```
cd ~/catkin_ws_bebop
catkin build dronecontrol
```
done!

---

