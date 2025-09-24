#include "skiplist.h"
#include <algorithm>
#include <iostream>

// -------------------- SkipNode --------------------
SkipNode::SkipNode(int val, int level) : value(val), forward(level, nullptr) {
    omp_init_lock(&lock);
}

SkipNode::~SkipNode() {
    omp_destroy_lock(&lock);
}

// -------------------- SkipList --------------------
SkipList::SkipList(int maxLvl, float prob)
    : maxLevel(maxLvl), p(prob), gen(std::random_device{}()) {
    header = new SkipNode(-1, maxLevel);
}

SkipList::~SkipList() {
    SkipNode* node = header;
    while (node) {
        SkipNode* next = node->forward[0];
        delete node;
        node = next;
    }
}

int SkipList::randomLevel() {
    int lvl = 1;
    std::uniform_real_distribution<float> dist(0.0, 1.0);
    while (dist(gen) < p && lvl < maxLevel) lvl++;
    return lvl;
}

// -------------------- Parallel Batch Insert --------------------
void SkipList::parallel_batch_insert(std::vector<int>& values) {
    int nthreads = omp_get_max_threads();
    std::vector<std::vector<int>> thread_batches(nthreads);

    // divide values among threads
    for (size_t i = 0; i < values.size(); i++) {
        thread_batches[i % nthreads].push_back(values[i]);
    }

    #pragma omp parallel for
    for (int t = 0; t < nthreads; t++) {
        auto& batch = thread_batches[t];
        std::sort(batch.begin(), batch.end());

        for (int val : batch) {
            std::vector<SkipNode*> update(maxLevel, nullptr);
            SkipNode* x = header;

            for (int i = maxLevel - 1; i >= 0; i--) {
                while (x->forward[i] && x->forward[i]->value < val)
                    x = x->forward[i];
                update[i] = x;
            }

            int lvl = randomLevel();
            SkipNode* newNode = new SkipNode(val, lvl);

            for (int i = 0; i < lvl; i++) {
                omp_set_lock(&update[i]->lock);
                newNode->forward[i] = update[i]->forward[i];
                update[i]->forward[i] = newNode;
                omp_unset_lock(&update[i]->lock);
            }
        }
    }
}

// -------------------- Parallel Search --------------------
SkipNode* SkipList::parallel_search(int target) {
    int topLevel = maxLevel - 1;
    SkipNode* found = nullptr;

    #pragma omp parallel shared(found)
    {
        #pragma omp single
        {
            for (SkipNode* x = header->forward[topLevel]; x && !found; x = x->forward[topLevel]) {
                #pragma omp task firstprivate(x)
                {
                    SkipNode* node = x;
                    for (int lvl = topLevel; lvl >= 0; lvl--) {
                        while (node->forward[lvl] && node->forward[lvl]->value < target)
                            node = node->forward[lvl];
                    }
                    node = node->forward[0];
                    if (node && node->value == target) {
                        #pragma omp critical
                        if (!found) found = node;
                    }
                }
            }
        }
    }
    return found;
}

