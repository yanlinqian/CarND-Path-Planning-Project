# Udacity Self-Driving Car Engineer Nanodegree
# Path Planning Project

## Introduction: path planning in highway
Given: map waypoints, ego car state (location & speed), other cars' state.

Target: drive a car safely and effciently to pass through others cars, with avoding traffic collision, keeping a enough high speed, avoding breaking traffic limits, avoding reaching maximum acceleration and jerk.

Steps:

1. get nearby waypoints of the ego car
2. intepolate nearby waypoints
3. get ego car state
4. get other cars' states
5. check check whether some other car is left or right to our car
6. find the best trajectory through which the ego car should move
7. make a new path for the ego car

## Detail

### 1. get nearby waypoints of the ego car

Given ego car state including car location information, we retrieve the cloest waypoint in front of ego car and 10 nearby waypoints from the map (contained in `highway_map.csv`). Code: line 359~373 in main.cpp.

### 2. intepolate nearby waypoints.

intepolate nearway waypoints in intepolation unit of 0.2 meter, which is useful for finding the best trajectory in step 6. The smaller the intepolation unit is, the smoother the found trajectory is. Code: line 375~397 in main.cpp.

### 3. get ego car state

If simulator gives the previous generated trajectory for ego car, use the last three to determine ego car's state. In this condition, ego car's state is in fact assumed future state, handing latency of simulator-mainprogram communication. If not, use returned ego car's state directly, setting speed and accelration equals 0. Code: line 400~418 in main.cpp.

### 4. get other cars' states

Generate s and d positions for other vehicles with assumed constant speed. Code: line 425~440 in main.cpp.


### 5. check check whether some other car is left or right to our car.

It is fatal to check whether some other car is beside us. If there is nothing in the left, we have option to change to the left lane. Code: line 444~458 in main.cpp.

### 6. find the best trajectory through which the ego car should move

Given the ego car's and other cars' states, the best trajectory of moving ego car is as follows:

1. for each action (stay in the current lane, move to the left lane, move to the right lane), a target (frenet position) is given based on the predicted traffic condition (line 50~108 in vehicle.cpp). 
Code: line 467~481 in main.cpp.
2. for each action and the corresponding target, jerk-minimizing trajectory connecting the current frenet position to the target frenet position is made (line 133~163 in vehicle.app).
3. to find which trajectory is best, we rely on a hybrid cost mixing: car collision, distance to the cloest car, speed, spatial shift from middle lane, spatial shift from the middle of the lane, acceletion, jerk(costs.h). Their weights (in order) are 10000,1,1000,10,100,10,10, which are based on tweaking.

Code: line 467~481 in main.cpp.

### 7. make a new path for the ego car

Given the best trajectory for ego car, we make a new path (lists of `x` and `y`) guiding ego car to run. First we get the coarse trajectory in frenet s axis, taht is the concatenation of the last two points of the unused previous path, 25-meter-ahead point, 50-meter-ahead point. The coarse trajectory for x and y can be computed by transfroming frenet coordinates to cartesian coordinates.

The speed increases or decreases in a fixed step if the speed differs from the target speed by the pre-defined gap. The speed helps us yield a dense s trajectory, from which we get dense x trajectory and y trajectory by spline interpolation tool. 

Concatenating previous path and dense x/y trajectory gives us the next path ego car should move.
Code: line 485~555 in main.cpp


## Conclusion
This solution performs acceptable and safely runs for over 20 mins in testing. Ego car has no better idea but slowly following the car ahead, when three lanes are all blocked. 

![a slow example](./slow.png)

*the description below is Udacity's original README for the project repo*

---

# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator. You can download the Term3 Simulator BETA which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
