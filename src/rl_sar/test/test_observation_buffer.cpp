/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "observation_buffer.hpp"
#include <iostream>
#include <stdexcept>
#include <algorithm>

/*
Output:

Initializing buffer with: obs_dims=[2 3 4], history_length=3, observations_history=[0 0 1 2], priority=time
Inserting observation t-2:
[ 11 12 21 22 23 31 32 33 34 ]
Inserting observation t-1:
[ 110 120 210 220 230 310 320 330 340 ]
Inserting observation t:
[ 1100 1200 2100 2200 2300 3100 3200 3300 3400 ]
time priority output:
[ 11 12 21 22 23 31 32 33 34 11 12 21 22 23 31 32 33 34 110 120 210 220 230 310 320 330 340 1100 1200 2100 2200 2300 3100 3200 3300 3400 ]

Initializing buffer with: obs_dims=[2 3 4], history_length=3, observations_history=[0 0 1 2], priority=term
Inserting observation t-2:
[ 11 12 21 22 23 31 32 33 34 ]
Inserting observation t-1:
[ 110 120 210 220 230 310 320 330 340 ]
Inserting observation t:
[ 1100 1200 2100 2200 2300 3100 3200 3300 3400 ]
term priority output:
[ 11 12 11 12 110 120 1100 1200 21 22 23 21 22 23 210 220 230 2100 2200 2300 31 32 33 34 31 32 33 34 310 320 330 340 3100 3200 3300 3400 ]
 
*/

void test_buffer(const std::string& priority)
{
    std::vector<int> obs_dims = {2, 3, 4};
    int num_envs = 1;
    std::vector<int> observations_history = {0, 0, 1, 2};
    int history_length = *std::max_element(observations_history.begin(), observations_history.end()) + 1;

    std::cout << "Initializing buffer with: "
                << "obs_dims=[" << obs_dims << "], "
                << "history_length=" << history_length << ", "
                << "observations_history=[" << observations_history << "], "
                << "priority=" << priority << "\n";

    ObservationBuffer buffer(num_envs, obs_dims, history_length, priority);

    std::vector<float> obs1 = {11, 12, 21, 22, 23, 31, 32, 33, 34};
    std::vector<float> obs2 = {110, 120, 210, 220, 230, 310, 320, 330, 340};
    std::vector<float> obs3 = {1100, 1200, 2100, 2200, 2300, 3100, 3200, 3300, 3400};

    std::cout << "Inserting observation t-2:\n" << obs1 << "\n";
    buffer.insert(obs1);

    std::cout << "Inserting observation t-1:\n" << obs2 << "\n";
    buffer.insert(obs2);

    std::cout << "Inserting observation t:\n" << obs3 << "\n";
    buffer.insert(obs3);

    auto history = buffer.get_obs_vec(observations_history);

    std::cout << priority << " priority output:\n" << history << "\n\n";
}

int main()
{
    test_buffer("time");
    test_buffer("term");
    return 0;
}
