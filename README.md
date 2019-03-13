# <img align="left" src="https://avatars1.githubusercontent.com/u/36579711?s=200&v=4" width="40" ><center>catkin_ws_bebop </center> 

### <center><bold color="#f03c15">Repo under development</bold> :exclamation:</center>

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
sudo systemctl start firmwared.service
sudo firmwared
```
in another terminal run
```
fdc ping 
```
if firmwared is running correctly you should get "PONG", otherwise check [install sphinx](https://github.com/Insper/bebop_sphinx) step.

... under development ...

----

#### Run on real live (offboard) <img align="right" src=".assets/parrot-bebop-2" width="100" >
tested on parrot bebop 2 drone

... under development ...


