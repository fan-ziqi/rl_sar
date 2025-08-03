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
 11  12  21  22  23  31  32  33  34
[ CPULongType{1,9} ]
Inserting observation t-1:
 110  120  210  220  230  310  320  330  340
[ CPULongType{1,9} ]
Inserting observation t:
 1100  1200  2100  2200  2300  3100  3200  3300  3400
[ CPULongType{1,9} ]
time priority output:
[ 1100 1200 2100 2200 2300 3100 3200 3300 3400 1100 1200 2100 2200 2300 3100 3200 3300 3400 110 120 210 220 230 310 320 330 340 11 12 21 22 23 31 32 33 34 ]

Initializing buffer with: obs_dims=[2 3 4], history_length=3, observations_history=[0 0 1 2], priority=term
Inserting observation t-2:
 11  12  21  22  23  31  32  33  34
[ CPULongType{1,9} ]
Inserting observation t-1:
 110  120  210  220  230  310  320  330  340
[ CPULongType{1,9} ]
Inserting observation t:
 1100  1200  2100  2200  2300  3100  3200  3300  3400
[ CPULongType{1,9} ]
term priority output:
[ 1100 1200 1100 1200 110 120 11 12 2100 2200 2300 2100 2200 2300 210 220 230 21 22 23 3100 3200 3300 3400 3100 3200 3300 3400 310 320 330 340 31 32 33 34 ]

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

    torch::Tensor obs1 = torch::tensor({{11, 12, 21, 22, 23, 31, 32, 33, 34}});
    torch::Tensor obs2 = obs1 * 10;
    torch::Tensor obs3 = obs1 * 100;

    std::cout << "Inserting observation t-2:\n" << obs1 << "\n";
    buffer.insert(obs1);

    std::cout << "Inserting observation t-1:\n" << obs2 << "\n";
    buffer.insert(obs2);

    std::cout << "Inserting observation t:\n" << obs3 << "\n";
    buffer.insert(obs3);

    auto history = buffer.get_obs_vec(observations_history);

    std::cout << priority << " priority output:\n[ ";
    for (int i = 0; i < history.size(1); ++i)
    {
        std::cout << history.index({0, i}).item<float>() << " ";
    }
    std::cout << "]\n\n";
}

int main()
{
    test_buffer("time");
    test_buffer("term");
    return 0;
}
