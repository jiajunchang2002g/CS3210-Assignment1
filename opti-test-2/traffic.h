#include "common.h"

namespace traffic_prng {
        extern PRNG* engine;
}

int find_dist(Params params, const std::vector<Car>& cars, int back, int front);

void decideLaneChangeForCar(const Params& params,
                const std::vector<Car>& cars,
                const std::vector<int>& lane,
                const std::vector<int>& other_lane,
                std::vector<char>& lane_flags,
                int idx);

void updateVelocityForCar(Params params, std::vector<Car>& cars,
                const std::vector<Car>& cars_old,
                std::vector<char>& ss_flags,
                const std::vector<char>& start,
                const std::vector<char>& dec,
                const std::vector<int>& lane,
                int index);
