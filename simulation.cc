#include <algorithm> 
#include <iostream> 
#include <vector> 
#include <assert.h> 
#include <omp.h> 
#include "traffic.h" 

namespace traffic_prng { 
        extern PRNG* engine; 
}

void executeSimulation(Params params, std::vector<Car> cars) {
        std::vector<char> start(params.n);
        std::vector<char> dec(params.n);
        std::vector<char> ss_flags(params.n, false);
        std::vector<char> lane_flags(params.n, false);
        std::vector<Car> cars_old(params.n);

        std::vector<std::vector<int>> lanes(2);
        for (auto& car : cars) lanes[car.lane].push_back(car.id);

        for (int step = 0; step < params.steps; ++step) {
                // --- Step 1: generate RNG ---
                for (int id = 0; id < params.n; ++id) {
                        start[id] = flip_coin(params.p_start, traffic_prng::engine);
                        dec[id] = flip_coin(params.p_dec, traffic_prng::engine);
                }

#pragma omp parallel 
                {
                        // --- Step 2: make a copy of state ---
#pragma omp for nowait
                        for (int id = 0; id < params.n; ++id) {
                                cars_old[id] = cars[id];
                        }

                        // --- Step 3: decide lane changes ---
#pragma omp for nowait
                        for (int i = 0; i < (int)lanes[0].size(); ++i) {
                                decideLaneChangeForCar(params, cars, lanes[0], lanes[1], lane_flags, i);
                        }
#pragma omp for 
                        for (int i = 0; i < (int)lanes[1].size(); ++i) {
                                decideLaneChangeForCar(params, cars, lanes[1], lanes[0], lane_flags, i);
                        }
                        // --- Implicit barrier ---
                        // --- Step 4: apply lane changes ---
#pragma omp for
                        for (int id = 0; id < (int)cars.size(); ++id) {
                                if (lane_flags[id]) {
                                        cars[id].lane = (cars[id].lane + 1) % 2;
                                        lane_flags[id] = false;
                                }
                        }

                        // --- Step 5: rebuild and sort lanes ---
#pragma omp single 
                        {
                                lanes[0].clear();
                                lanes[1].clear();
                                for (int i = 0; i < int(cars.size()); ++i) {
                                        Car car = cars[i];
                                        lanes[car.lane].push_back(car.id);
                                }
                        }
#pragma omp single nowait
                        {
                                std::sort(lanes[0].begin(), lanes[0].end(),
                                                [&](int a, int b) { return cars[a].position < cars[b].position; });
                        }
#pragma omp single 
                        {
                                std::sort(lanes[1].begin(), lanes[1].end(),
                                                [&](int a, int b) { return cars[a].position < cars[b].position; });
                        }

                        // --- Step 6: update velocities lane by lane ---
#pragma omp for nowait
                        for (int idx = 0; idx < (int)lanes[0].size(); ++idx) {
                                updateVelocityForCar(params, cars, cars_old, ss_flags, start, dec, lanes[0], idx);
                        }
#pragma omp for 
                        for (int idx = 0; idx < (int)lanes[1].size(); ++idx) {
                                updateVelocityForCar(params, cars, cars_old, ss_flags, start, dec, lanes[1], idx);
                        }
                        // --- Implicit barrier --- 
#pragma omp for 
                        // --- Step 7: move cars ---
                        for (int i = 0; i < params.n; ++i) {
                                cars[i].position = (cars[i].position + cars[i].v) % params.L;
                        }
                } // END OF PARALLEL REGION

#ifdef DEBUG
                reportResult(cars, step);
#endif
        }

#ifdef DEBUG
        reportFinalResult(cars);
#endif
}

