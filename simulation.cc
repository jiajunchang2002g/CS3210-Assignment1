#include <algorithm>
#include <iostream>
#include <vector>
#include <cassert>

#include <omp.h>

#include "traffic.h"

void executeSimulation(Params params, std::vector<Car>og_cars) {
        std::vector<char> start(params.n);
        std::vector<char> dec(params.n);
        std::vector<char> ss_flags(params.n, false);
        std::vector<char> lane_flags(params.n, false);

        CarsSoA cars(og_cars);
        CarsSoA cars_old(params.n);

        std::vector<std::vector<int>> lanes(2);
        for (int i = 0; i < params.n; ++i)
                lanes[cars.lane[i]].push_back(cars.id[i]);

        for (int step = 0; step < params.steps; ++step) {
                // --- Step 1: generate RNG ---
                for (int id = 0; id < params.n; ++id) {
                        start[id] = flip_coin(params.p_start, traffic_prng::engine);
                        dec[id]   = flip_coin(params.p_dec, traffic_prng::engine);
                }

#pragma omp parallel
                {
                        // --- Step 2: copy state ---
#pragma omp for nowait
                        for (int i = 0; i < params.n; ++i) {
                                cars_old.id[i]       = cars.id[i];
                                cars_old.lane[i]     = cars.lane[i];
                                cars_old.position[i] = cars.position[i];
                                cars_old.velocity[i] = cars.velocity[i];
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
                        // --- Barrier here ---

                        // --- Step 4: apply lane changes ---
#pragma omp for
                        for (int i = 0; i < params.n; ++i) {
                                if (lane_flags[i]) {
                                        cars.lane[i] = (cars.lane[i] + 1) % 2;
                                        lane_flags[i] = false;
                                }
                        }

                        // --- Step 5: rebuild and sort lanes ---
#pragma omp single
                        {
                                lanes[0].clear();
                                lanes[1].clear();
                        }

                        std::vector<int> thread_lane0, thread_lane1;
#pragma omp for
                        for (int i = 0; i < params.n; ++i) {
                                if (cars.lane[i] == 0)
                                        thread_lane0.push_back(cars.id[i]);
                                else
                                        thread_lane1.push_back(cars.id[i]);
                        }
#pragma omp critical
                        {
                                lanes[0].insert(lanes[0].end(), thread_lane0.begin(), thread_lane0.end());
                                lanes[1].insert(lanes[1].end(), thread_lane1.begin(), thread_lane1.end());
                        }
#pragma omp barrier

#pragma omp single nowait
                        {
                                std::sort(lanes[0].begin(), lanes[0].end(),
                                                [&](int a, int b) { return cars.position[a] < cars.position[b]; });
                        }
#pragma omp single
                        {
                                std::sort(lanes[1].begin(), lanes[1].end(),
                                                [&](int a, int b) { return cars.position[a] < cars.position[b]; });
                        }

                        // --- Step 6: update velocities ---
#pragma omp for nowait
                        for (int idx = 0; idx < (int)lanes[0].size(); ++idx) {
                                updateVelocityForCar(params, cars, cars_old, ss_flags, start, dec, lanes[0], idx);
                        }
#pragma omp for
                        for (int idx = 0; idx < (int)lanes[1].size(); ++idx) {
                                updateVelocityForCar(params, cars, cars_old, ss_flags, start, dec, lanes[1], idx);
                        }
                        // --- Barrier ---

                        // --- Step 7: move cars ---
#pragma omp for
                        for (int i = 0; i < params.n; ++i) {
                                cars.position[i] = (cars.position[i] + cars.velocity[i]) % params.L;
                        }
                } // END parallel region

#ifdef DEBUG
                og_cars = cars.toAoS();
                reportResult(og_cars, step);
#endif
        }

#ifdef DEBUG
        reportFinalResult(og_cars);
#endif
}

