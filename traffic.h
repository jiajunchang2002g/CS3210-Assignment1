#include "common.h"

namespace traffic_prng {
        extern PRNG* engine;
}

template <typename T>
std::vector<T> parallel_copy(const std::vector<T>& src) {
    std::vector<T> dst(src.size());

    #pragma omp parallel for
    for (std::size_t i = 0; i < src.size(); i++) {
        dst[i] = src[i];
    }

    return dst;
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

