//
//  behavior.h
//  
//
//  Created by Chun Fang on 03/10/2017.
//

#ifndef behavior_h
#define behavior_h

#include "tools.h"
#include "trajectory.h"

class Behavior {
public:
    Behavior();
    // This function aims to update lane status and velocity information if necessary.
    void behaviorPlan(Lane &lane, Trajectory &trajectory);
};


Behavior::Behavior(void) {}

void Behavior::behaviorPlan(Lane &lane, Trajectory &trajectory) {
    // Is it too close to the front car
    bool too_close = false;
    int curr_lane = lane.current_lane;
    int num_cars = lane.interval_distances[curr_lane].size();
    for (int i=0; i<num_cars; i++) {
        if (lane.interval_distances[curr_lane][i] > 0 &&
            lane.interval_distances[curr_lane][i] < 30) {
            too_close = true;
            break;
        }
    }
    if (too_close) {
        // If it is too close, consider the availablility of its adjacent lanes.
        bool changed_lane = false;
        vector<int> adjacent_lanes = {-1, 1};
        for (int i = 0; i < adjacent_lanes.size(); i++) {
            bool is_safe = true;
            int target_lane = lane.current_lane + adjacent_lanes[i];
            if (target_lane < 0 || target_lane > 2) {
                continue;
            }
            // In this for loop, we observe that the left lane is prefered.
            int num_cars = lane.interval_distances[target_lane].size();
            for (int j=0; j<num_cars; j++) {
                // Adjust those values, e.g. '0', '30', to make sure the car can drive safely.
                if (lane.interval_distances[target_lane][j] > 0 &&
                    lane.interval_distances[target_lane][j] < 30) {
                    is_safe = false;
                    // This version prefers the left lane than right lane
                    break;
                }
                if (lane.interval_distances[target_lane][j] < 0 &&
                    lane.interval_distances[target_lane][j] > - 25) {
                    is_safe = false;
                    break;
                }
            }
            if (is_safe) {
                lane.current_lane = target_lane;
                changed_lane = true;
                break;
            }
        }
        if(!changed_lane){
            trajectory.ref_vel -= 0.224;
        }
    } else if (trajectory.ref_vel < 49.5) {
        trajectory.ref_vel += 0.224;
    }
}



#endif /* behavior_h */
