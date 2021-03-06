# Race Track
Author: Jacque, Shuo Shi

Date: 04/05/2021
## Project structure
/src/race_middle_blue.py only provides cv auto pilot functionality
/src/race_car.py provides auto pilot and obstacle avoiding functionality.

## How to run
```bash
cd ~/catkin
catkin_make
. devel/setup.bash
roslaunch race_car race_car.py
```

## Core algorithm
### CV auto pilot(Shuo)
The auto pilot algorithm is based on opencv library. The algorihtm is described as below:
* Use mask filter to get the blue edge of the track.
* Take portion of the masked image and calculate contours.
* Devide contours into two groups(one the one left side, another one on the right side)
* If both sides exist, keep robot moving straight. If left side is missing, set angular velocity to make robot move in spiral to the left; if right side is missing, set angular velocity to make robot move in spiral to the right.

### Obstacle avoiding(Jacque)
On the robot's way racing through the track, it will be able to avoid and pass by the object with minimum detour from its originally supposed track.
It is still not able to avoid obstacle in the very middle of the track.
