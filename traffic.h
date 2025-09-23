#include "common.h"

namespace traffic_prng {
        extern PRNG* engine;
}

void decideLaneChanges(Params params, std::vector<Car>& cars,
                const std::vector<std::vector<int>>& lanes,
                std::vector<bool>& lane_flags);

void applyLaneChanges(std::vector<Car>& cars, std::vector<bool>& lane_flags); 

void updateVelocities(Params params, std::vector<Car>& cars,
                std::vector<Car>& cars_old,
                const std::vector<std::vector<int>>& lanes,
                std::vector<bool>& ss_flags,
                const std::vector<bool>& start,
                const std::vector<bool>& dec);

void generateRNGResults(Params params, std::vector<bool>& start, std::vector<bool>& dec); 

void rebuildAndSortLanes(std::vector<Car>& cars, std::vector<std::vector<int>>& lanes); 

void moveCars(std::vector<Car>& cars, Params params); 

