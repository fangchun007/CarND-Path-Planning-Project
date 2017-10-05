//
//  trajectory.h
//
//  Created by Chun Fang on 03/10/2017.
//

#ifndef trajectory_h
#define trajectory_h

#include "spline.h"
#include "tools.h"

class Trajectory {
public:
    bool initialized = false;
    double ref_vel;
    Trajectory();
    void init(double ref_vel_init);
    // This function is used to generate next path points
    vector<vector<double>> generateTrajectory(double car_x, double car_y, double car_yaw,
                                              double ref_s, Lane lane,
                                              vector<double> previous_path_x,
                                              vector<double> previous_path_y,
                                              vector<double> map_waypoints_s,
                                              vector<double> map_waypoints_x,
                                              vector<double> map_waypoints_y);
};

Trajectory::Trajectory() {}

void Trajectory::init(double ref_vel_init) {
    this->ref_vel = 0;
    this->initialized = true;
}

vector<vector<double>> Trajectory::generateTrajectory(double car_x, double car_y, double car_yaw,
                                                      double ref_s, Lane lane,
                                                      vector<double> previous_path_x,
                                                      vector<double> previous_path_y,
                                                      vector<double> map_waypoints_s,
                                                      vector<double> map_waypoints_x,
                                                      vector<double> map_waypoints_y) {
    vector<double> ptsx;
    vector<double> ptsy;
    // In this implementation, we always consider the end of previous path as the reference (original
    // point) position of points we are going to generate.
    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);
    int prev_size = previous_path_x.size();
    if (prev_size < 2) {
        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);
        ptsx.push_back(prev_car_x);
        ptsx.push_back(car_x);
        ptsy.push_back(prev_car_y);
        ptsy.push_back(car_y);
    } else {
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];
        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);
        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }
    // We take two from our previous path data as above and generate the left three as below.
    // Note they are all represented in global coordinate system.
    for (int i = 0; i < 3; i++) {
        double new_car_s = ref_s + (i + 1) * 30;
        double new_car_d = 2 + 4 * lane.current_lane;
        vector<double> next_wp = getXY(new_car_s,
                                       new_car_d,
                                       map_waypoints_s,
                                       map_waypoints_x,
                                       map_waypoints_y);
        ptsx.push_back(next_wp[0]);
        ptsy.push_back(next_wp[1]);
    }
    // Change to local system
    for (int i = 0; i < ptsx.size(); i++) {
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;
        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
    }
    // Cubic Spline interpolation (spline.h)
    tk::spline s;
    s.set_points(ptsx, ptsy);
    // Generate next path values
    vector<double> next_x_vals;
    vector<double> next_y_vals;
    // Since we start from the end of the previous path data, we are going to use all previous
    // path points. This should be changed correspondingly if the reference position changed.
    for (int i = 0; i < previous_path_x.size(); i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);
    double x_add_on = 0;
    // According to present velocity, we can calculate the step length.
    double N = target_dist / (TS * this->ref_vel / 2.24);
    double step_length = target_x / N;
    for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
        double x_point = x_add_on + step_length;
        double y_point = s(x_point);
        x_add_on = x_point;
        double x_ref = x_point;
        double y_ref = y_point;
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
        x_point += ref_x;
        y_point += ref_y;
        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }
    return {next_x_vals, next_y_vals};
}


#endif /* trajectory_h */
