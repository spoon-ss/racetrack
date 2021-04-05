# Race Track
Author: Jacque, Shuo Shi

Date: 04/05/2021
## Project structure

## How to run
```bash
cd ~/catkin
catkin_make
. devel/setup.bash

<-------> TO DO
roslaunch race_car xxx.py
<--------->
```



## Core algorithm
### CV auto pilot(Shuo)
The auto pilot algorithm is based on opencv library. The algorihtm is described as below:
* Use mask filter to get the blue edge of the track.
* Take portion of the masked image and calculate contours.
* Devide contours into two groups(one the one left side, another one on the right side)
* If both sides exist, keep robot moving straight. If left side is missing, set angular velocity to make robot move in spiral to the left; if right side is missing, set angular velocity to make robot move in spiral to the right.

### Obstacle avoiding(Jacque)
