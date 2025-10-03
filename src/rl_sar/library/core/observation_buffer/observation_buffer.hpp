/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef OBSERVATION_BUFFER_HPP
#define OBSERVATION_BUFFER_HPP

#include <vector>
#include <string>

/**
 * @brief Observation buffer for storing historical observations
 * 
 * Manages a circular buffer of observations with configurable history length
 * and supports different output priority modes (time/term)
 */
class ObservationBuffer
{
public:
    /**
     * @brief Constructor with parameters
     * @param num_envs Number of environments
     * @param obs_dims Dimensions of each observation component
     * @param history_length Length of observation history to maintain
     * @param priority Output priority mode ("time" or "term")
     */
    ObservationBuffer(int num_envs, const std::vector<int>& obs_dims, int history_length, const std::string& priority);
    
    /**
     * @brief Default constructor
     */
    ObservationBuffer();

    /**
     * @brief Reset specified environments with new observations
     * @param reset_idxs Indices of environments to reset
     * @param new_obs New observation data to fill the buffer
     */
    void reset(std::vector<int> reset_idxs, const std::vector<float>& new_obs);
    
    /**
     * @brief Insert new observation into buffer
     * @param new_obs New observation data to insert
     */
    void insert(const std::vector<float>& new_obs);
    
    /**
     * @brief Get observation vector based on specified indices
     * @param obs_ids Indices specifying which observations to retrieve
     * @return Concatenated observation vector
     */
    std::vector<float> get_obs_vec(std::vector<int> obs_ids);

private:
    int num_envs;                                           ///< Number of environments
    std::vector<int> obs_dims;                              ///< Dimensions of observation components
    std::string priority;                                   ///< Output priority mode
    int num_obs = 0;                                        ///< Total observation dimension
    int history_length = 0;                                 ///< History buffer length
    int num_obs_total = 0;                                  ///< Total observation size
    std::vector<std::vector<std::vector<float>>> obs_buf;   ///< Observation buffer [env][time][obs]
};

#endif // OBSERVATION_BUFFER_HPP
