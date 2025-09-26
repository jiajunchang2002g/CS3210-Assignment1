#include "common.h"

namespace traffic_prng { 
        extern PRNG* engine; 
}

// --- SoA Data Layout ---
struct CarsSoA {
    std::vector<int> id;
    std::vector<int> lane;
    std::vector<int> position;
    std::vector<int> velocity;

    CarsSoA() = default;
    CarsSoA(int n) {
        id.resize(n);
        lane.resize(n);
        position.resize(n);
        velocity.resize(n);
    }

    // Construct SoA from AoS vector<Car>
    CarsSoA(const std::vector<Car>& cars) {
        int n = cars.size();
        id.resize(n);
        lane.resize(n);
        position.resize(n);
        velocity.resize(n);

        for (int i = 0; i < n; ++i) {
            id[i]       = cars[i].id;
            lane[i]     = cars[i].lane;
            position[i] = cars[i].position;
            velocity[i] = cars[i].v;
        }
    }

    std::vector<Car> toAoS() const {
        int n = id.size();
        std::vector<Car> cars(n);
        for (int i = 0; i < n; ++i) {
            cars[i].id       = id[i];
            cars[i].lane     = lane[i];
            cars[i].position = position[i];
            cars[i].v = velocity[i];
        }
        return cars;
    }
};

int find_dist(const Params& params, const CarsSoA& cars, int back, int front);

void decideLaneChangeForCar(const Params& params,
                const CarsSoA& cars,
                const std::vector<int>& lane,
                const std::vector<int>& other_lane,
                std::vector<char>& lane_flags,
                int idx);

void updateVelocityForCar(const Params& params,
                CarsSoA& cars,
                const CarsSoA& cars_old,
                std::vector<char>& ss_flags,
                const std::vector<char>& start,
                const std::vector<char>& dec,
                const std::vector<int>& lane,
                int index);

