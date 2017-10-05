//
//  lane.h
//
//  Created by Chun Fang on 03/10/2017.
//

#ifndef lane_h
#define lane_h

class Lane {
public:
    bool initialized = false;
    int current_lane;
    int num_lanes;
    double lane_width;
    vector<vector<double>> lane_boundaries;
    vector<vector<double>> interval_distances; // records distances from main car to detected cars for each lane
    
    Lane();
    void init(int num_lanes, double lane_width, int current_lane);
    // This function predict the traffic situation, such as distances from main car to other cars
    // at the time moment N*TS
    // :sensor_fusion: sensor fusion data
    // :ref_s:         reference point
    // :N:             number of steps from the current position to reference point
    void predict(vector<vector<double>> sensor_fusion, double ref_s, int N);
};

Lane::Lane(void) {}

void Lane::init(int num_lanes, double lane_width, int current_lane) {
    this->num_lanes = num_lanes;
    this->lane_width = lane_width;
    this->current_lane = current_lane;
    for (int i=0; i<num_lanes; i++){
        vector<double> boundaries_lr = {double(i)*lane_width, double(i+1)*lane_width};
        this->lane_boundaries.push_back(boundaries_lr);
    }
    this->initialized = true;
}

void Lane::predict(vector<vector<double>> sensor_fusion, double ref_s, int N) {
    this->interval_distances = {};
    for (int i = 0; i < this->num_lanes; i++) {
        // check each lane for nearby vehicles
        int num_cars = sensor_fusion.size();
        vector<double> distances;
        for (int j = 0; j < num_cars; j++){
            if (sensor_fusion[j][SF.d] >= this->lane_boundaries[i][0] &&
                sensor_fusion[j][SF.d] <  this->lane_boundaries[i][1])
            {
                double vx = sensor_fusion[j][SF.vx];
                double vy = sensor_fusion[j][SF.vy];
                double v = sqrt(vx*vx + vy*vy);
                double s = sensor_fusion[j][SF.s];
                s += v*TS*N;
                distances.push_back(s-ref_s);
            }
        }
        this->interval_distances.push_back(distances);
        
    }
}

#endif /* lane_h */
