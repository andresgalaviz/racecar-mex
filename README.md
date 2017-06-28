## BW México 2017
This repository was created for the mexican students who will be participating in MIT's Beaver Works Autonomous-Racecar Competition  of 2017.

# racecar
Run the following commands in the shell screipt(open it bye clicking: ctrl + alt + t)
To setup git: 
```
git config --global user.email "yourmail@you.com"
git config --global user.name "Your Github Name(not username)"`
git config --global credential.helper "cache --timeout=36000"
```
To add this repository to the racecar vm:
```
cd ~/racecar-ws/
wstool set src/racecar-mex --git "https://github.com/AndresGalaviz/racecar-mex"
wstool update
catkin_make
```

In order to test that it is working run the following commands:
```
roslaunch racecar_gazebo racecar.launch
rosrun racecar_mex keyboard_teleop.py
```
