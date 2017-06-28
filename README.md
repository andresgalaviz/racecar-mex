# racecar
To setup git: 
```
git config --global user.email "yourmail@you.com"
git config --global user.name "Your Github Name(not username)"`
```
Run the following commands in the shell screipt(open it bye clicking: ctrl + alt + t)to add this repository to the racecar vm:
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
