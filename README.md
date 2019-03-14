# <img align="left" src="https://avatars1.githubusercontent.com/u/36579711?s=200&v=4" width="40" ><div style="text-align:center">catkin_ws_bebop</div>

### <div style="text-align:center;color:red"><bold >Repo under development</bold> :exclamation:</div>

<br>

ros workspace to simulate imav2019 indor


### what you will find in this repo
* catkin workspace and ros codes for [imav2019](https://imav2019.org/) competition

----
#### Prerequisites
* [install ros](http://wiki.ros.org/kinetic/Installation/Ubuntu) (check documentation)


----
#### Setup Repository 
(tested on ubuntu 16.04)


clone repo
```
cd ~/
git clone --recurse-submodules https://github.com/SkyRats/catkin_ws_bebop
```
install sphinx:
```
. ~/catkin_ws_bebop/install.sh
```

build catkin workspace
```
cd ~/catkin_ws_bebop/
git clone https://github.com/AutonomyLab/bebop_autonomy.git src/bebop_autonomy
rosdep update
rosdep install --from-paths src -i
catkin build
echo "source /home/arthur/catkin_ws_bebop/devel/setup.bash" >> ~/.bashrc
```
----
#### Run simulation
* launch firmwared

open a terminal and run:
```
. ~/catkin_ws_bebop/setup_simulation.sh
```
in another terminal run:
```
. ~/catkin_ws_bebop/launch_simulation_world.sh
```

If firmwared is running correctly you should get "PONG", otherwise check instructions[install sphinx](https://github.com/Insper/bebop_sphinx) step.

check if the drone have spawned correctly by running:
```
ping 10.202.0.1   #it might be other ip like 10.202.X.1
```
you shoud get some thing like:
>64 bytes from 10.202.0.1: icmp_seq=1 ttl=64 time=0.030 ms<br>
64 bytes from 10.202.0.1: icmp_seq=2 ttl=64 time=0.034 ms<br>
64 bytes from 10.202.0.1: icmp_seq=3 ttl=64 time=0.046 ms<br>
...

Great! now you are ready to simulate!
... under development ...

----

#### Run on real live (offboard)
tested on parrot bebop 2 drone  <img align="center" style="margin:-50px 0px -50px 0px;" src=".assets/parrot-bebop-2" width="100" >

... under development ...


