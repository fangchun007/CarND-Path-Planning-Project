# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)
[image1]: ./src/pathPlanning_flowchart.pdf
[image2]: ./pathPlanning_flowchart.jpg

## Goal
In this project, our goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. 

### Input (data obtained from [simulator](https://github.com/udacity/self-driving-car-sim/releases))
*  Main car's localization data (No Noise)
*  Sensor fusion data
*  Previous path data given to the planner
*  Previous path's end s and d values
*  Map of the highway

### Output (data needs to be generated for [simulator](https://github.com/udacity/self-driving-car-sim/releases))
*  a path made up of (x,y) points that the car will visit sequentially every .02 seconds

## Flowchart
![alt text][image2]

## Implementation
### Trajectory Generation
Let's start with trajectory generation. This module aims to generate the next path points, which will be used by the car's perfect controller and makes it visit every (x,y) point it recieves in the list every .02 seconds. The data we can use includes sensor fusion data which describes the current traffic conditions, main car's localization data, previous path data, and map of highway. 

First, we are going to use some of previous path data. It has at least two merits. One, save computation time. Two, improve driving stability. In this version, I used all of the left previous path data. Namely, the end of previous path will be considered as the reference when we add new way points.

Second, generate a sample path by using the updated lane information. Here, a C++ cubic spline interpolation library [spline.h](http://kluge.in-chemnitz.de/opensource/spline/) and five points were used to generate a cubic polynomial curve. The first two points is achieved from the previous path data or generated from the main car localization data if necessary. The other three is produced as follows.
```
// for i = 1, 2, 3
double new_car_s = ref_s + (i + 1) * 30;
double new_car_d = 2 + 4 * lane.current_lane;
```

Third, sample several points from the above path. In this step, we concern the updated velocity information in order to avoid the possible cllision. The main code is as follows.
```
double N = target_dist / (0.02 * this->ref_vel / 2.24);
double step_length = target_x / N;
for (int i = 1; i <= 50 - previous_path_x.size(); i++)
```
### Path Planning
Suppose the traffic status for the future several senconds are clear. We now design a  

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

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

