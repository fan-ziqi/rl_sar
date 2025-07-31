/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef OBSERVATION_BUFFER_HPP
#define OBSERVATION_BUFFER_HPP

#include <torch/torch.h>
#include <vector>

class ObservationBuffer
{
public:
    ObservationBuffer(int num_envs, const std::vector<int>& obs_dims, int history_length, const std::string& priority);
    ObservationBuffer();

    void reset(std::vector<int> reset_idxs, torch::Tensor new_obs);
    void insert(torch::Tensor new_obs);
    torch::Tensor get_obs_vec(std::vector<int> obs_ids);

private:
    int num_envs;
    std::vector<int> obs_dims;
    std::string priority;
    int num_obs = 0;
    int history_length = 0;
    int num_obs_total = 0;
    torch::Tensor obs_buf;
};

#endif // OBSERVATION_BUFFER_HPP
