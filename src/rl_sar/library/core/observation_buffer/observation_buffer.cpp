/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "observation_buffer.hpp"

ObservationBuffer::ObservationBuffer() {}

ObservationBuffer::ObservationBuffer(int num_envs,
                                     const std::vector<int>& obs_dims,
                                     int history_length,
                                     const std::string& priority)
    : num_envs(num_envs),
      obs_dims(obs_dims),
      history_length(history_length),
      priority(priority)
{
    if (num_envs <= 0 || history_length <= 0) throw std::invalid_argument("num_envs and history_length must be positive");

    for (int dim : obs_dims)
    {
        if (dim <= 0) throw std::invalid_argument("All observation dimensions must be positive");
        num_obs += dim;
    }

    num_obs_total = num_obs * history_length;
    if (num_obs_total <= 0) throw std::runtime_error("Invalid total observation dimension");
    obs_buf = torch::zeros({num_envs, num_obs_total}, torch::dtype(torch::kFloat32));
}

void ObservationBuffer::reset(std::vector<int> reset_idxs, torch::Tensor new_obs)
{
    std::vector<torch::indexing::TensorIndex> indices;
    for (int idx : reset_idxs)
    {
        indices.push_back(torch::indexing::Slice(idx));
    }
    obs_buf.index_put_(indices, new_obs.repeat({1, history_length}));
}

void ObservationBuffer::insert(torch::Tensor new_obs)
{
    // Shift observations back.
    torch::Tensor shifted_obs = obs_buf.index({torch::indexing::Slice(torch::indexing::None), torch::indexing::Slice(num_obs, num_obs * history_length)}).clone();
    obs_buf.index({torch::indexing::Slice(torch::indexing::None), torch::indexing::Slice(0, num_obs * (history_length - 1))}) = shifted_obs;

    // Add new observation.
    obs_buf.index({torch::indexing::Slice(torch::indexing::None), torch::indexing::Slice(-num_obs, torch::indexing::None)}) = new_obs;
}

/**
 * @brief Gets history of observations indexed by obs_ids.
 *
 * @param obs_ids An array of integers with which to index the desired
 *                observations, where 0 is the latest observation and
 *                history_length - 1 is the oldest observation.
 * @return A torch::Tensor containing the concatenated observations.
 */
torch::Tensor ObservationBuffer::get_obs_vec(std::vector<int> obs_ids)
{
    std::vector<torch::Tensor> obs;

    if (this->priority == "time")
    {
        for (int i = 0; i < obs_ids.size(); ++i)
        {
            int obs_id = obs_ids[i];
            int slice_idx = history_length - obs_id - 1;
            obs.push_back(obs_buf.index({torch::indexing::Slice(torch::indexing::None), torch::indexing::Slice(slice_idx * num_obs, (slice_idx + 1) * num_obs)}));
        }
    }
    else if(this->priority == "term")
    {
        int obs_offset = 0;
        for (size_t i = 0; i < obs_dims.size(); ++i)
        {
            int dim = obs_dims[i];
            for (int step : obs_ids)
            {
                int time_offset = (history_length - step - 1) * num_obs;
                int pos = obs_offset + time_offset;
                obs.push_back(obs_buf.index({torch::indexing::Slice(), torch::indexing::Slice(pos, pos + dim)}));
            }
            obs_offset += dim;
        }
    }

    return torch::cat(obs, -1);
}
