# <img align="left" src="https://avatars1.githubusercontent.com/u/36579711?s=200&v=4" width="40" ><div style="text-align:center">catkin_ws_bebop</div>

### <div style="text-align:center;color:red"><bold >Repo under development</bold> :exclamation:</div>

<br>

ros workspace to simulate imav2019 indor


### what you will find in this repo
* catkin workspace and ros codes for [imav2019](https://imav2019.org/) competition

----
#### Prerequisites
* [install ros](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* [install sphinx](https://github.com/Insper/bebop_sphinx)

#### Install
-----
clone repo
```
cd ~/
git clone https://github.com/SkyRats/catkin_ws_bebop
```

build catkin workspace
```
cd ~/catkin_ws_bebop/
catkin build
```
----
#### Run simulation
* launch firmwared

open a terminal and run:
```
. ~/catkin_ws_bebop/setup_simulation.sh
```
in another terminal run
```
. ~/launch_simulation_world.sh
```
if firmwared is running correctly you should get "PONG", otherwise check [install sphinx](https://github.com/Insper/bebop_sphinx) step.

... under development ...

----

#### Run on real live (offboard)
tested on parrot bebop 2 drone  <img align="center" style="margin:-50px 0px -50px 0px;" src=".assets/parrot-bebop-2" width="100" >

... under development ...


