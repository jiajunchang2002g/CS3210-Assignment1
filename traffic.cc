#include <algorithm>
#include <iostream>
#include <vector>

#include "traffic.h"

#include <omp.h>


// --------------------------------------------------
// Helper: find distance btw two cars
// --------------------------------------------------

int find_dist(Params params, std::vector<Car>& cars, int back, int front) {
        int dist = cars[front].position - cars[back].position;
        if (dist <= 0) dist += params.L; // dist == 0 is when there is only one car in the lane
        return dist;
}

// --------------------------------------------------
// Helper: decide lane changes for all cars
// --------------------------------------------------

void decideLaneChanges(Params params, std::vector<Car>& cars,
                const std::vector<std::vector<int>>& lanes,
                std::vector<bool>& lane_flags)
{
        for (int lane_idx = 0; lane_idx < 2; ++lane_idx) {
                auto& lane = lanes[lane_idx];
                auto& other = lanes[(lane_idx + 1) % 2];

                int n = other.size();
                if (n == 0) continue; 

#pragma omp parallel for 
                for (int index = 0; index < int(lane.size()); ++index) {
                        int id = lane[index];
                        int v1 = cars[id].v;
                        auto low_it = std::lower_bound(other.begin(), other.end(), cars[id].position,
                                        [&](int cid, int pos){ return cars[cid].position < pos; });

                        if (low_it != other.end() && cars[*low_it].position == cars[id].position) continue;

                        int front;
                        int back;

                        if (low_it == other.end()) {
                                front = other[0];
                                back = other[n - 1]; // last element
                        } else {
                                front = *low_it;
                                if (low_it == other.begin()) {
                                        back = other[n - 1]; // wrap around to last element
                                } else {
                                        back = *(low_it - 1);
                                }
                        }

                        int d0 = find_dist(params, cars, back, id);
                        int d2 = find_dist(params, cars, id, lane[(index + 1) % lane.size()]); 
                        int d3 = find_dist(params, cars, id, front);
                        int v0 = cars[back].v;

                        if (d0 > 0 && d2 < d3 && v1 >= d2 && d0 > v0) {
                                lane_flags[id] = true;
                        }
                }
        }
}

// --------------------------------------------------
// Helper: apply lane changes
// --------------------------------------------------
void applyLaneChanges(std::vector<Car>& cars, std::vector<bool>& lane_flags) {
#pragma omp parallel for 
        for (int id = 0; id < int(cars.size()); ++id) {
                if (lane_flags[id]) {
                        cars[id].lane = (cars[id].lane + 1) % 2;
                        lane_flags[id] = false;
                }
        }
}

bool avoid_crash(std::vector<Car>& cars, int id, int dist, int v_i, int v_front) {
        if (dist <= v_i) {
                if (v_i < v_front || v_i < 2) {
                        cars[id].v = std::max(0, dist - 1);
                        return true;
                } else if (v_i >= v_front && v_i >= 2) {
                        cars[id].v = std::max(0, std::min(dist - 1, cars[id].v - 2));
                        return true;
                }
        } else if (dist <= 2 * v_i && v_i >= v_front) {
                cars[id].v = v_i - floor((v_i - v_front) / 2.0);
                return true;
        }
        return false;
}
// --------------------------------------------------
// Helper: update velocities for all cars in lanes
// --------------------------------------------------
void updateVelocities(Params params, std::vector<Car>& cars,
                std::vector<Car>& cars_old,
                const std::vector<std::vector<int>>& lanes,
                std::vector<bool>& ss_flags,
                const std::vector<bool>& start,
                const std::vector<bool>& dec)
{
        for (int lane_idx = 0; lane_idx < 2; ++lane_idx) {
                auto& lane = lanes[lane_idx];
                int n = lane.size();
                for (int index = 0; index < n; ++index) {
                        int id = lane[index];
                        int next_id = lane[(index + 1) % n];
                        int dist = find_dist(params, cars, id, next_id);
                        int v_front = cars_old[next_id].v;

                        cars_old[id] = cars[id];

                        bool rule1_applied = false;

                        if (ss_flags[id]) {
                                // Slow Start Part 2
                                cars[id].v = 1;
                                ss_flags[id] = false;
                                rule1_applied = true;
                        }  

                        if (cars[id].v == 0 && dist > 1) {
                                if (start[id]) {
                                        // Step 3: Acceleration
                                        if (cars[id].v + 1 < dist) {
                                                cars[id].v = std::min(params.vmax, cars[id].v + 1);
                                        }
                                } else {
                                        // Slow Start Part 1
                                        ss_flags[id] = true;
                                }
                        } else {
                                // Step 2: Avoid crashing
                                bool rule2_applied = false;

                                if (dist <= cars[id].v) {
                                        if (cars[id].v < v_front || cars[id].v < 2) {
                                                cars[id].v = std::max(0, dist - 1);
                                                rule2_applied = true;
                                        } else {
                                                cars[id].v = std::max(0, std::min(dist - 1, cars[id].v - 2));
                                                rule2_applied = true;
                                        }
                                } else if (dist <= 2 * cars[id].v && cars[id].v >= v_front) {
                                        cars[id].v = cars[id].v - floor((cars[id].v - v_front) / 2.0);
                                        rule2_applied = true;
                                }

                                // Step 3: Acceleration
                                if (!rule2_applied && !rule1_applied) {
                                        if (cars[id].v + 1 < dist) {
                                                cars[id].v = std::min(params.vmax, cars[id].v + 1);
                                        }
                                }
                        }

                        // Step 4: Random deceleration
                        if (dec[id]) {
                                cars[id].v = std::max(0, cars[id].v - 1);
                        }
                }
        }
}


// --------------------------------------------------
// Helper: generate RNG results
// --------------------------------------------------
void generateRNGResults(Params params, std::vector<bool>& start, std::vector<bool>& dec) {
        for (int id = 0; id < params.n; ++id) {
                start[id] = flip_coin(params.p_start, traffic_prng::engine);
                dec[id] = flip_coin(params.p_dec, traffic_prng::engine);
        }
}


// --------------------------------------------------
// Helper: rebuild lanes and sort by position
// --------------------------------------------------
void rebuildAndSortLanes(std::vector<Car>& cars, std::vector<std::vector<int>>& lanes) {
        lanes[0].clear();
        lanes[1].clear();
        for (int i = 0; i < int(cars.size()); ++i) {
                Car car = cars[i];
                lanes[car.lane].push_back(car.id); 
        }
        for (int i = 0; i < 2; ++i) {
                std::sort(lanes[i].begin(), lanes[i].end(),
                                [&](int a, int b){ return cars[a].position < cars[b].position; }); // O(n log n)
        }
}

// --------------------------------------------------
// Helper: move cars according to current velocity
// --------------------------------------------------
void moveCars(std::vector<Car>& cars, Params params) {
#pragma omp parallel for
        for (int i = 0; i < params.n; ++i) {
                cars[i].position = (cars[i].position + cars[i].v) % params.L;
        }
}

