#ifndef SKIPLIST_H
#define SKIPLIST_H

#include <vector>
#include <random>
#include <omp.h>

struct SkipNode {
    int value;
    std::vector<SkipNode*> forward;
    omp_lock_t lock;

    SkipNode(int val, int level);
    ~SkipNode();
};

struct SkipList {
    int maxLevel;
    float p;
    SkipNode* header;
    std::mt19937 gen;

    SkipList(int maxLvl, float prob);
    ~SkipList();

    int randomLevel();

    void parallel_batch_insert(std::vector<int>& values);
    SkipNode* parallel_search(int target);
};

#endif // SKIPLIST_H

