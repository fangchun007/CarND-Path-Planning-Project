# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)
[image1]: ./src/pathPlanning_flowchart.pdf
[image2]: ./pathPlanningFlowchart.jpg

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
Please visit [trajectory.h](https://github.com/fangchun007/CarND-Path-Planning-Project/blob/master/src/trajectory.h) for more detail.

### Path Planning
Suppose the traffic status for the future several senconds are clear. We now need to decide which lane we should drive and what's the idea velocity. 

In this version of implementation, we used a simple path planning rule. But it should be suitable for highway driving in this simulator. In detail, we will keep the same lane if the current lane don't slow down. In case we consider changing lane, the left lane is always preferred than the right lane. If the adjacent lanes are not available, then slow down.

For relations between this method and the one we learned from lectures, this method used a step function as the cost function. 

Please visit [behavior.h](https://github.com/fangchun007/CarND-Path-Planning-Project/blob/master/src/behavior.h) for the code.

### Prediction 
In this module, we try to figure out the traffic situation in the future several seconds. 

In the implementation, the idea of recursion is used. Namely, we assume by this moment the path (from where the car is to the reference point. In our case, it is the whole previous path) we generated previously is safe. This path needs to be extended so that the car can drvie safely in the future several seconds. So we only consider the traffic situation starts from reference point.
```
        for (int j = 0; j < num_cars; j++){
            if (sensor_fusion[j][SF.d] >= this->lane_boundaries[i][0] && sensor_fusion[j][SF.d] < this->lane_boundaries[i][1]) {
                double vx = sensor_fusion[j][SF.vx];
                double vy = sensor_fusion[j][SF.vy];
                double v = sqrt(vx*vx + vy*vy);
                double s = sensor_fusion[j][SF.s];
                s += v*TS*N;
                distances.push_back(s-ref_s);
            }
        }
```

Please visit [lane.h](https://github.com/fangchun007/CarND-Path-Planning-Project/blob/master/src/lane.h) for more detail.
