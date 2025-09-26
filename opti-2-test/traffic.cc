#include "traffic.h"
#include <algorithm>

/// --------------------------------------------------
// Helper: find distance between two cars
// --------------------------------------------------
int find_dist(Params params, const std::vector<Car>& cars, int back, int front) {
        int dist = cars[front].position - cars[back].position;
        if (dist <= 0) dist += params.L;
        return dist;
}

// --------------------------------------------------
// Branchless lane change for a single car
// --------------------------------------------------
void decideLaneChangeForCar(const Params& params,
                const std::vector<Car>& cars,
                const std::vector<int>& lane,
                const std::vector<int>& other_lane,
                std::vector<char>& lane_flags,
                int idx)
{
        int id = lane[idx];
        int v1 = cars[id].v;

        // Find closest cars in other lane
        auto it = std::lower_bound(other_lane.begin(), other_lane.end(), cars[id].position,
                        [&](int cid, int pos) { return cars[cid].position < pos; });

        int front = (it == other_lane.end()) ? other_lane[0] : *it;
        int back  = (it == other_lane.begin()) ? other_lane.back() : *(it - 1);

        int d0 = find_dist(params, cars, back, id);
        int d2 = find_dist(params, cars, id, lane[(idx + 1) % lane.size()]);
        int d3 = find_dist(params, cars, id, front);
        int v0 = cars[back].v;

        // Compute branchless mask
        // 1 if all conditions satisfied, 0 otherwise
        int valid_position = (it == other_lane.end() || cars[*it].position != cars[id].position);
        int cond_mask = (d0 > 0) & (d2 < d3) & (v1 >= d2) & (d0 > v0) & valid_position;

        // Apply mask
        lane_flags[id] = cond_mask ? 1 : 0;
}


// --------------------------------------------------
// Helper: update velocity of a single car (uses cars_old as read-only)
// --------------------------------------------------
void updateVelocityForCar(Params params,
                std::vector<Car>& cars,
                const std::vector<Car>& cars_old,
                std::vector<char>& ss_flags,
                const std::vector<char>& start,
                const std::vector<char>& dec,
                const std::vector<int>& lane,
                int index)
{
        // cache locality check
        int id = lane[index];
        int next_id = lane[(index + 1) % lane.size()];
        int dist = find_dist(params, cars, id, next_id);
        int v_front = cars_old[next_id].v;   // old velocity of car in front

        bool rule1_applied = false;

        if (ss_flags[id]) {
                cars[id].v = 1;
                ss_flags[id] = false;
                rule1_applied = true;
        }

        if (cars[id].v == 0 && dist > 1) {
                if (start[id]) {
                        if (cars[id].v + 1 < dist) {
                                cars[id].v = std::min(params.vmax, cars[id].v + 1);
                        }
                } else {
                        ss_flags[id] = true;
                }
        } else {
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
                        cars[id].v = cars[id].v - (cars[id].v - v_front) / 2;
                        rule2_applied = true;
                }

                if (!rule2_applied && !rule1_applied) {
                        if (cars[id].v + 1 < dist) {
                                cars[id].v = std::min(params.vmax, cars[id].v + 1);
                        }
                }
        }

        if (dec[id]) {
                cars[id].v = std::max(0, cars[id].v - 1);
        }
}
