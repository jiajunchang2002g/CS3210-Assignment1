#include <assert.h>
#include <omp.h>

#include "traffic.h"

void executeSimulation(Params params, std::vector<Car> cars) {
        std::vector<bool> start(params.n);
        std::vector<bool> dec(params.n);
        std::vector<bool> ss_flags(params.n, false);
        std::vector<bool> lane_flags(params.n, false);

        // init lanes
        std::vector<std::vector<int>> lanes(2);
        rebuildAndSortLanes(cars, lanes);
        std::vector<Car> cars_old(params.n);

        for (int step = 0; step < params.steps; ++step) {
                cars_old = cars;  // Capture state before this timestep
                
                // 1. Generate RNG results
                generateRNGResults(params, start, dec);

                // 2. Decide lane changes
                decideLaneChanges(params, cars, lanes, lane_flags);

                // 3. Apply lane changes
                applyLaneChanges(cars, lane_flags);

                // 4. Rebuild and sort lanes after lane changes
                rebuildAndSortLanes(cars, lanes);

                // 5. Update velocities
                updateVelocities(params, cars, cars_old, lanes, ss_flags, start, dec);

                // 6. Move cars
                moveCars(cars, params);

                reportResult(cars, step);
        }

        reportFinalResult(cars);
}


