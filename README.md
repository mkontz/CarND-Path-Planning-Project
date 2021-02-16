# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### My Solution
##### Overview
The main class, Planner, is implemented in src/pathPlanner.h. The plan method  has four main parts.

- Fits a spline to the lanes points near the car
- Creates the initial path from the preious path (if available)
- Set the new path extending from the previous path that either stays in the lane or transitions to a new lane
- Speed control module that sets the point spacing (speed) along the path.

##### Fitting the lane points
Achieving a smooth, accurate path is critical to  stay in a lane and minimize acceleration and jerk.  Based on the current location of the car, a range of waypoints are selected both before and after the car.  It is important that these point extend further than  longest planned path.  Each lane is fitted with a quintic spline that require x-y position, heading and curvature at each point.  It is desirable that the lane splines do not move as segment are added to the front and removed from the end.  

A waypoint's position, heading and curvature are calculating by averaging two cubic splines fit through the waypoints before and after.  One cubic spline is fit through waypoints and the other is fit through the midpoints of the linear segments connecting the waypoints.  Going around a corner the end  point spline will tend toward the outside of the lane and the midpoint spline will tend towards the inside of the lane.  So averaging two sets of parameter results in a quintic spline that stays centered in the lane even through curves.

Both the quintic spline and the cubic spline are implemented as classes that inherit from a common base spline class that implements methods to advance or retreat along the spline a set distance and find the closet point on a spline to a given position.

##### Initial path
Id no previous path information is available, the the previous path is simply the car position and heading and curvature is set to 0.  If previous path information is available a portion of it is used to create the new path.  In this cases, x and y polynomial are fit to the previous data in order to calculate heading and curvature the end of the initial path points.  

##### New Path
If the car is not changing lanes and the end of the initial path in on the target lane, then the new path is simply the target lane spline.  Otherwise an addition spline needs to be defined in order to transition from the end of the initial path to the target lane spline.

This transition spline is a single segment quintic spline requiring position, heading and curvature at each end.  The initial positional, heading and curvature is defined form the initial path and the other end needs to match the point where it joins the lane spline.

##### Deciding when to change lanes
The position and speed of other cars is used to decide when to change lanes.  The first criteria used to change lane is that there is a car in front causing the ego car to slow down.  If there is adjacent lane that has no car in front and no cars are blocking then that lane is chosen.  Otherwise other cars must be moving faster then the car being followed.

The logic worked relatively well, but had limits such as  

-  In general was overly conservative
- Did not consider 2 lanes over
- Applied the same speed logic as if it was following the car being passed until the lane change was almost complete.
-  Once a lane change start it was not adjusted or abort.  So occasionally it would collide with a car from two lanes over changing into the same lane

##### Speed Control
If no car is infront, then the speed increasesto the target speed at the target acceleration.  If a car was present then a following distance was set based on the ego car's speed.  A simple tracking controller was implemented with the speed of the car being followed fedforward and portional position error fedback.  To minimize acceleration and jerk this speed control input was rate limited by desired acceleration and jerk limits.

This speed control is simple, but pretty effective.  One area where the simplicity was a detriment was during lane changes, the speed logic would continue to queue of the car being passed until the lane change was almost complete.

##### Evauation
Overall I feel this solution works relatively well, but is by no means optimized.

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

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

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

