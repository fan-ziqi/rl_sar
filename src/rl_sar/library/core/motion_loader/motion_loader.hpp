/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTION_LOADER_HPP
#define MOTION_LOADER_HPP

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <algorithm>
#include "vector_math.hpp"
#include "../logger/logger.hpp"

/**
 * @brief Motion data loader for mimic/dance tasks
 *
 * Loads motion capture data from CSV files and provides interpolated
 * joint positions and velocities for a given time.
 *
 * CSV Format (per row):
 * root_pos_x, root_pos_y, root_pos_z, root_quat_x, root_quat_y, root_quat_z, root_quat_w,
 * joint_0, joint_1, ..., joint_N
 */
class MotionLoader
{
public:
    /**
     * @brief Constructor
     * @param motion_file Path to CSV motion file
     * @param fps Frames per second of the motion data
     */
    MotionLoader(const std::string& motion_file, float fps);

    /**
     * @brief Update motion to a specific time
     * @param time Current time in seconds
     */
    void Update(float time);

    /**
     * @brief Reset motion to start with yaw alignment
     * @param robot_base_quat Current robot base quaternion [w, x, y, z]
     * @param robot_waist_angles Current robot waist joint angles [yaw, roll, pitch]
     */
    void Reset(const std::vector<float>& robot_base_quat, const std::vector<float>& robot_waist_angles);

    /**
     * @brief Get interpolated joint positions at current time
     * @return Joint positions vector
     */
    std::vector<float> GetJointPos() const;

    /**
     * @brief Get interpolated joint velocities at current time
     * @return Joint velocities vector
     */
    std::vector<float> GetJointVel() const;

    /**
     * @brief Get interpolated root quaternion at current time
     * @return Root quaternion [w, x, y, z]
     */
    std::vector<float> GetRootQuat() const;

    /**
     * @brief Get anchor (torso) quaternion at current time
     *
     * For G1: torso = root * Rz(yaw) * Rx(roll) * Ry(pitch)
     * where yaw=joint[12], roll=joint[13], pitch=joint[14]
     *
     * @return Anchor quaternion [w, x, y, z]
     */
    std::vector<float> GetAnchorQuat() const;

    /**
     * @brief Get motion duration in seconds
     */
    float GetDuration() const { return duration_; }

    /**
     * @brief Get world to init transformation quaternion (yaw alignment)
     */
    std::vector<float> GetInitQuat() const { return world_to_init_; }

    /**
     * @brief Compute torso quaternion from base quaternion and waist joint angles
     * @param base_quat Base (pelvis) quaternion [w, x, y, z]
     * @param waist_angles Waist joint angles [yaw, roll, pitch]
     * @return Torso quaternion [w, x, y, z]
     */
    static std::vector<float> ComputeTorsoQuat(const std::vector<float>& base_quat, const std::vector<float>& waist_angles);

    /**
     * @brief Compute initial yaw alignment quaternion
     * @param robot_torso_quat Robot's torso quaternion [w, x, y, z]
     * @param motion_torso_quat Motion's torso quaternion [w, x, y, z]
     * @return Yaw alignment quaternion [w, x, y, z]
     */
    static std::vector<float> ComputeYawAlignment(const std::vector<float>& robot_torso_quat, const std::vector<float>& motion_torso_quat);

private:
    /**
     * @brief Load motion data from CSV file
     * @param filename Path to CSV file
     */
    void LoadFromCSV(const std::string& filename);

    /**
     * @brief Compute velocities from positions using finite differences
     */
    void ComputeVelocities();

    /**
     * @brief Spherical linear interpolation between two quaternions
     * @param q0 First quaternion [w, x, y, z]
     * @param q1 Second quaternion [w, x, y, z]
     * @param t Interpolation parameter [0, 1]
     * @return Interpolated quaternion [w, x, y, z]
     */
    std::vector<float> Slerp(const std::vector<float>& q0, const std::vector<float>& q1, float t) const;

    // Motion data storage
    std::vector<std::vector<float>> root_positions_;     // [T, 3]
    std::vector<std::vector<float>> root_quaternions_;   // [T, 4] - each is [w, x, y, z]
    std::vector<std::vector<float>> joint_positions_;    // [T, N]
    std::vector<std::vector<float>> joint_velocities_;   // [T, N]

    // Motion properties
    int num_frames_;
    int num_joints_;
    float dt_;           // Time between frames
    float duration_;     // Total duration

    // Current interpolation state
    int index_0_;        // Current frame index
    int index_1_;        // Next frame index
    float blend_;        // Interpolation factor [0, 1]

    // Coordinate transformation
    std::vector<float> world_to_init_;  // For yaw alignment between robot and motion [w, x, y, z]
};

#endif // MOTION_LOADER_HPP
