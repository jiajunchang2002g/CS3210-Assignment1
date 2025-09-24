#include <vector>
#include <omp.h>
#include <cstdint>

void parallel_radix_sort(std::vector<uint32_t>& data) {
    const int num_threads = omp_get_max_threads();
    const int n = data.size();
    std::vector<uint32_t> output(n);
    
    for (int shift = 0; shift < 32; shift += 8) { // 8 bits per pass
        std::vector<int> counts(num_threads * 256, 0);
        
        // Step 1: Count digits in parallel
        #pragma omp parallel
        {
            int tid = omp_get_thread_num();
            int start = tid * n / num_threads;
            int end   = (tid + 1) * n / num_threads;
            for (int i = start; i < end; ++i) {
                int byte = (data[i] >> shift) & 0xFF;
                counts[tid*256 + byte]++;
            }
        }

        // Step 2: Compute prefix sum per digit
        std::vector<int> offsets(256, 0);
        for (int b = 0; b < 256; ++b) {
            for (int t = 0; t < num_threads; ++t)
                offsets[b] += counts[t*256 + b];
            offsets[b] -= counts[b]; // starting index for digit b
        }

        // Step 3: Move elements in parallel
        #pragma omp parallel
        {
            int tid = omp_get_thread_num();
            int start = tid * n / num_threads;
            int end   = (tid + 1) * n / num_threads;
            std::vector<int> local_offsets(256);
            for (int b = 0; b < 256; ++b) {
                local_offsets[b] = offsets[b]; 
                offsets[b] += counts[tid*256 + b]; // update global offset
            }
            for (int i = start; i < end; ++i) {
                int byte = (data[i] >> shift) & 0xFF;
                output[local_offsets[byte]++] = data[i];
            }
        }

        std::swap(data, output); // prepare for next byte
    }
}

