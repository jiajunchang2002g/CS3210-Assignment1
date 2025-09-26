#include "traffic.h"
#include <algorithm>

int find_dist(const Params& params, const CarsSoA& cars, int back, int front) {
        int dist = cars.position[front] - cars.position[back];
        if (dist <= 0) dist += params.L;
        return dist;
}

void decideLaneChangeForCar(const Params& params,
                const CarsSoA& cars,
                const std::vector<int>& lane,
                const std::vector<int>& other_lane,
                std::vector<char>& lane_flags,
                int idx)
{
        int id = lane[idx];
        int v1 = cars.velocity[id];  // updated to velocity

        auto it = std::lower_bound(other_lane.begin(), other_lane.end(), cars.position[id],
                        [&](int cid, int pos) { return cars.position[cid] < pos; });

        int front = (it == other_lane.end()) ? other_lane[0] : *it;
        int back  = (it == other_lane.begin()) ? other_lane.back() : *(it - 1);

        int d0 = find_dist(params, cars, back, id);
        int d2 = find_dist(params, cars, id, lane[(idx + 1) % lane.size()]);
        int d3 = find_dist(params, cars, id, front);
        int v0 = cars.velocity[back];

        int valid_position = (it == other_lane.end() || cars.position[*it] != cars.position[id]);
        int cond_mask = (d0 > 0) & (d2 < d3) & (v1 >= d2) & (d0 > v0) & valid_position;

        lane_flags[id] = cond_mask ? 1 : 0;
}

void updateVelocityForCar(const Params& params,
                CarsSoA& cars,
                const CarsSoA& cars_old,
                std::vector<char>& ss_flags,
                const std::vector<char>& start,
                const std::vector<char>& dec,
                const std::vector<int>& lane,
                int index)
{
        int id = lane[index];
        int next_id = lane[(index + 1) % lane.size()];

        int dist = find_dist(params, cars, id, next_id);
        int v_front = cars_old.velocity[next_id];  // updated to velocity

        bool rule1_applied = false;

        if (ss_flags[id]) {
                cars.velocity[id] = 1;
                ss_flags[id] = false;
                rule1_applied = true;
        }

        if (cars.velocity[id] == 0 && dist > 1) {
                if (start[id]) {
                        if (cars.velocity[id] + 1 < dist) {
                                cars.velocity[id] = std::min(params.vmax, cars.velocity[id] + 1);
                        }
                } else {
                        ss_flags[id] = true;
                }
        } else {
                bool rule2_applied = false;
                if (dist <= cars.velocity[id]) {
                        if (cars.velocity[id] < v_front || cars.velocity[id] < 2) {
                                cars.velocity[id] = std::max(0, dist - 1);
                                rule2_applied = true;
                        } else {
                                cars.velocity[id] = std::max(0, std::min(dist - 1, cars.velocity[id] - 2));
                                rule2_applied = true;
                        }
                } else if (dist <= 2 * cars.velocity[id] && cars.velocity[id] >= v_front) {
                        cars.velocity[id] = cars.velocity[id] - (cars.velocity[id] - v_front) / 2;
                        rule2_applied = true;
                }

                if (!rule2_applied && !rule1_applied) {
                        if (cars.velocity[id] + 1 < dist) {
                                cars.velocity[id] = std::min(params.vmax, cars.velocity[id] + 1);
                        }
                }
        }

        if (dec[id]) {
                cars.velocity[id] = std::max(0, cars.velocity[id] - 1);
        }
}

