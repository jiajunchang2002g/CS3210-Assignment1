#include "common.h"

namespace traffic_prng {
        extern PRNG* engine;
}

int find_dist(Params params, const std::vector<Car>& cars, int back, int front);

void decideLaneChangeForCar(const Params& params,
                const std::vector<Car>& cars,
                const std::vector<int>& lane,
                const std::vector<int>& other_lane,
                std::vector<uint8_t>& lane_flags,
                int idx);

void updateVelocityForCar(Params params, std::vector<Car>& cars,
                const std::vector<Car>& cars_old,
                std::vector<uint8_t>& ss_flags,
                const std::vector<uint8_t>& start,
                const std::vector<uint8_t>& dec,
                const std::vector<int>& lane,
                int index);
