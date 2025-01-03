# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

import torch

class ObservationBuffer:
    def __init__(self, num_envs, num_obs, include_history_steps):

        self.num_envs = num_envs
        self.num_obs = num_obs
        self.include_history_steps = include_history_steps

        self.num_obs_total = num_obs * include_history_steps

        self.obs_buf = torch.zeros(self.num_envs, self.num_obs_total, dtype=torch.float)

    def reset(self, reset_idxs, new_obs):
        self.obs_buf[reset_idxs] = new_obs.repeat(1, self.include_history_steps)

    def insert(self, new_obs):
        # Shift observations back.
        self.obs_buf[:, : self.num_obs * (self.include_history_steps - 1)] = self.obs_buf[:,self.num_obs : self.num_obs * self.include_history_steps].clone()

        # Add new observation.
        self.obs_buf[:, -self.num_obs:] = new_obs

    def get_obs_vec(self, obs_ids):
        """Gets history of observations indexed by obs_ids.

        Arguments:
            obs_ids: An array of integers with which to index the desired
                observations, where 0 is the latest observation and
                include_history_steps - 1 is the oldest observation.
        """

        obs = []
        for obs_id in reversed(obs_ids):
            slice_idx = self.include_history_steps - obs_id - 1
            obs.append(self.obs_buf[:, slice_idx * self.num_obs : (slice_idx + 1) * self.num_obs])
        return torch.cat(obs, dim=-1)
