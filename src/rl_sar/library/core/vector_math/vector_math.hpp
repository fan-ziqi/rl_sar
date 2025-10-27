/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file vector_math.hpp
 * @brief Template-based vector mathematical operations library
 *
 * This header-only library provides mathematical operations for std::vector containers,
 * enabling vector arithmetic similar to NumPy or PyTorch but using standard C++ containers.
 *
 * OVERLOADED OPERATORS AND FUNCTIONS:
 *
 * Clamp Functions:
 *   - clamp(T value, T min, T max)                           // Scalar clamp
 *   - clamp(vector, T min, T max)                           // Vector clamp with single bounds
 *   - clamp(vector, min_vector, max_vector)                 // Vector clamp with vector bounds
 *
 * Arithmetic Operators:
 *   - vector * scalar                                       // Element-wise scaling
 *   - scalar * vector                                       // Element-wise scaling (commutative)
 *   - vector / scalar                                       // Element-wise division by scalar
 *   - vector1 * vector2                                     // Element-wise multiplication (Hadamard product)
 *   - vector1 / vector2                                     // Element-wise division
 *   - vector1 + vector2                                     // Element-wise addition
 *   - vector1 - vector2                                     // Element-wise subtraction
 *   - -vector                                              // Unary minus (negation)
 *
 * Compound Assignment Operators:
 *   - vector += vector                                      // In-place addition
 *   - vector -= vector                                      // In-place subtraction
 *   - vector *= scalar                                      // In-place scalar multiplication
 *   - vector /= scalar                                      // In-place scalar division
 *
 * Stream Output Operator:
 *   - operator<<(ostream, vector)                           // Stream output with formatting
 *
 * All functions are templated to work with any numeric type (float, double, int, etc.)
 * and use efficient memory management with proper vector reservation.
 */

#ifndef VECTOR_MATH_HPP
#define VECTOR_MATH_HPP

#include <vector>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <type_traits>
#include <cmath>

/**
 * @brief Clamps a scalar value between minimum and maximum bounds
 *
 * @tparam T Numeric type (float, double, int, etc.)
 * @param value The value to clamp
 * @param min_val Minimum allowed value
 * @param max_val Maximum allowed value
 * @return T The clamped value
 *
 * @note If value < min_val, returns min_val
 *       If value > max_val, returns max_val
 *       Otherwise returns value unchanged
 */
template <typename T>
inline T clamp(T value, T min_val, T max_val)
{
    return (value < min_val) ? min_val : (value > max_val) ? max_val : value;
}

/**
 * @brief Clamps all elements of a vector using single min/max values
 *
 * @tparam T Numeric type of vector elements
 * @param vec Input vector to clamp
 * @param min_val Minimum value for all elements
 * @param max_val Maximum value for all elements
 * @return std::vector<T> New vector with clamped values
 *
 * @note Each element is independently clamped to [min_val, max_val]
 *       Original vector remains unchanged
 */
template <typename T>
inline std::vector<T> clamp(const std::vector<T> &vec, T min_val, T max_val)
{
    std::vector<T> result;
    result.reserve(vec.size());
    for (T val : vec)
    {
        result.push_back(clamp(val, min_val, max_val));
    }
    return result;
}

/**
 * @brief Clamps vector elements using corresponding min/max vectors
 *
 * @tparam T Numeric type of vector elements
 * @param vec Input vector to clamp
 * @param min_vals Vector of minimum values for each element
 * @param max_vals Vector of maximum values for each element
 * @return std::vector<T> New vector with clamped values
 *
 * @note If min_vals or max_vals are shorter than vec, the last value is reused
 *       If min_vals or max_vals are empty, the original values are preserved
 *       Element i is clamped to [min_vals[i], max_vals[i]]
 */
template <typename T>
inline std::vector<T> clamp(const std::vector<T> &vec, const std::vector<T> &min_vals, const std::vector<T> &max_vals)
{
    std::vector<T> result;
    result.reserve(vec.size());
    for (size_t i = 0; i < vec.size(); ++i)
    {
        T min_val = (i < min_vals.size()) ? min_vals[i] : (min_vals.empty() ? vec[i] : min_vals[0]);
        T max_val = (i < max_vals.size()) ? max_vals[i] : (max_vals.empty() ? vec[i] : max_vals[0]);
        result.push_back(clamp(vec[i], min_val, max_val));
    }
    return result;
}

/**
 * @brief Multiplies each element of a vector by a scalar (vector * scalar)
 *
 * @tparam T Numeric type
 * @param vec Input vector
 * @param scalar Scalar multiplier
 * @return std::vector<T> New vector with scaled values
 *
 * @note Equivalent to: [v1*s, v2*s, v3*s, ...] where vi are vector elements
 */
template <typename T>
inline std::vector<T> operator*(const std::vector<T> &vec, T scalar)
{
    std::vector<T> result;
    result.reserve(vec.size());
    for (T val : vec)
    {
        result.push_back(val * scalar);
    }
    return result;
}

/**
 * @brief Multiplies each element of a vector by a scalar (scalar * vector)
 *
 * @tparam T Numeric type
 * @param scalar Scalar multiplier
 * @param vec Input vector
 * @return std::vector<T> New vector with scaled values
 *
 * @note Commutative version of vector * scalar
 *       Equivalent to: [s*v1, s*v2, s*v3, ...] where vi are vector elements
 */
template <typename T>
inline std::vector<T> operator*(T scalar, const std::vector<T> &vec)
{
    return vec * scalar;
}

/**
 * @brief Element-wise multiplication of two vectors
 *
 * @tparam T Numeric type
 * @param vec1 First input vector
 * @param vec2 Second input vector
 * @return std::vector<T> New vector with element-wise products
 *
 * @note Result size is min(vec1.size(), vec2.size())
 *       Equivalent to: [v1[0]*v2[0], v1[1]*v2[1], ...]
 *       Also known as Hadamard product or Schur product
 */
template <typename T>
inline std::vector<T> operator*(const std::vector<T> &vec1, const std::vector<T> &vec2)
{
    std::vector<T> result;
    result.reserve(std::min(vec1.size(), vec2.size()));
    for (size_t i = 0; i < std::min(vec1.size(), vec2.size()); ++i)
    {
        result.push_back(vec1[i] * vec2[i]);
    }
    return result;
}

/**
 * @brief Element-wise subtraction of two vectors
 *
 * @tparam T Numeric type
 * @param vec1 First input vector (minuend)
 * @param vec2 Second input vector (subtrahend)
 * @return std::vector<T> New vector with element-wise differences
 *
 * @note Result size is min(vec1.size(), vec2.size())
 *       Equivalent to: [v1[0]-v2[0], v1[1]-v2[1], ...]
 */
template <typename T>
inline std::vector<T> operator-(const std::vector<T> &vec1, const std::vector<T> &vec2)
{
    std::vector<T> result;
    result.reserve(std::min(vec1.size(), vec2.size()));
    for (size_t i = 0; i < std::min(vec1.size(), vec2.size()); ++i)
    {
        result.push_back(vec1[i] - vec2[i]);
    }
    return result;
}

/**
 * @brief Element-wise addition of two vectors
 *
 * @tparam T Numeric type
 * @param vec1 First input vector
 * @param vec2 Second input vector
 * @return std::vector<T> New vector with element-wise sums
 *
 * @note Result size is min(vec1.size(), vec2.size())
 *       Equivalent to: [v1[0]+v2[0], v1[1]+v2[1], ...]
 */
template <typename T>
inline std::vector<T> operator+(const std::vector<T> &vec1, const std::vector<T> &vec2)
{
    std::vector<T> result;
    result.reserve(std::min(vec1.size(), vec2.size()));
    for (size_t i = 0; i < std::min(vec1.size(), vec2.size()); ++i)
    {
        result.push_back(vec1[i] + vec2[i]);
    }
    return result;
}

/**
 * @brief Unary minus operator - negates all elements of a vector
 *
 * @tparam T Numeric type
 * @param vec Input vector
 * @return std::vector<T> New vector with negated values
 *
 * @note Equivalent to: [-v1, -v2, -v3, ...] where vi are vector elements
 *       Also known as additive inverse
 */
template <typename T>
inline std::vector<T> operator-(const std::vector<T> &vec)
{
    std::vector<T> result;
    result.reserve(vec.size());
    for (T val : vec)
    {
        result.push_back(-val);
    }
    return result;
}

/**
 * @brief Divides each element of a vector by a scalar (vector / scalar)
 *
 * @tparam T Numeric type
 * @param vec Input vector
 * @param scalar Scalar divisor
 * @return std::vector<T> New vector with scaled values
 *
 * @note Equivalent to: [v1/s, v2/s, v3/s, ...] where vi are vector elements
 *       Division by zero behavior depends on the numeric type
 */
template <typename T>
inline std::vector<T> operator/(const std::vector<T> &vec, T scalar)
{
    std::vector<T> result;
    result.reserve(vec.size());
    for (T val : vec)
    {
        result.push_back(val / scalar);
    }
    return result;
}

/**
 * @brief Element-wise division of two vectors
 *
 * @tparam T Numeric type
 * @param vec1 First input vector (dividend)
 * @param vec2 Second input vector (divisor)
 * @return std::vector<T> New vector with element-wise quotients
 *
 * @note Result size is min(vec1.size(), vec2.size())
 *       Equivalent to: [v1[0]/v2[0], v1[1]/v2[1], ...]
 *       Division by zero behavior depends on the numeric type
 */
template <typename T>
inline std::vector<T> operator/(const std::vector<T> &vec1, const std::vector<T> &vec2)
{
    std::vector<T> result;
    result.reserve(std::min(vec1.size(), vec2.size()));
    for (size_t i = 0; i < std::min(vec1.size(), vec2.size()); ++i)
    {
        result.push_back(vec1[i] / vec2[i]);
    }
    return result;
}

/**
 * @brief In-place addition of two vectors
 *
 * @tparam T Numeric type
 * @param vec1 Vector to modify (left-hand side)
 * @param vec2 Vector to add (right-hand side)
 * @return std::vector<T>& Reference to the modified vector
 *
 * @note Modifies vec1 in place, operates on min(vec1.size(), vec2.size()) elements
 *       Equivalent to: vec1[i] += vec2[i] for each valid index i
 */
template <typename T>
inline std::vector<T>& operator+=(std::vector<T> &vec1, const std::vector<T> &vec2)
{
    for (size_t i = 0; i < std::min(vec1.size(), vec2.size()); ++i)
    {
        vec1[i] += vec2[i];
    }
    return vec1;
}

/**
 * @brief In-place subtraction of two vectors
 *
 * @tparam T Numeric type
 * @param vec1 Vector to modify (left-hand side)
 * @param vec2 Vector to subtract (right-hand side)
 * @return std::vector<T>& Reference to the modified vector
 *
 * @note Modifies vec1 in place, operates on min(vec1.size(), vec2.size()) elements
 *       Equivalent to: vec1[i] -= vec2[i] for each valid index i
 */
template <typename T>
inline std::vector<T>& operator-=(std::vector<T> &vec1, const std::vector<T> &vec2)
{
    for (size_t i = 0; i < std::min(vec1.size(), vec2.size()); ++i)
    {
        vec1[i] -= vec2[i];
    }
    return vec1;
}

/**
 * @brief In-place scalar multiplication of a vector
 *
 * @tparam T Numeric type
 * @param vec Vector to modify
 * @param scalar Scalar multiplier
 * @return std::vector<T>& Reference to the modified vector
 *
 * @note Modifies vec in place
 *       Equivalent to: vec[i] *= scalar for each element
 */
template <typename T>
inline std::vector<T>& operator*=(std::vector<T> &vec, T scalar)
{
    for (T &val : vec)
    {
        val *= scalar;
    }
    return vec;
}

/**
 * @brief In-place scalar division of a vector
 *
 * @tparam T Numeric type
 * @param vec Vector to modify
 * @param scalar Scalar divisor
 * @return std::vector<T>& Reference to the modified vector
 *
 * @note Modifies vec in place
 *       Equivalent to: vec[i] /= scalar for each element
 *       Division by zero behavior depends on the numeric type
 */
template <typename T>
inline std::vector<T>& operator/=(std::vector<T> &vec, T scalar)
{
    for (T &val : vec)
    {
        val /= scalar;
    }
    return vec;
}

/**
 * @brief Output stream operator for std::vector
 *
 * @tparam T Numeric type of vector elements
 * @param os Output stream
 * @param vec Vector to output
 * @return std::ostream& Reference to the output stream
 *
 * @note Outputs vector in format: [elem1, elem2, elem3, ...]
 *       Floating point numbers are formatted with 2 decimal places
 */
template <typename T>
inline std::ostream& operator<<(std::ostream& os, const std::vector<T>& vec)
{
    os << "[";
    for (size_t i = 0; i < vec.size(); ++i)
    {
        if constexpr (std::is_floating_point_v<T>)
        {
            os << std::fixed << std::setprecision(2) << vec[i];
        }
        else
        {
            os << vec[i];
        }
        if (i < vec.size() - 1) os << ", ";
    }
    os << "]";
    return os;
}

// ============================================================================
// QUATERNION MATH FUNCTIONS
// ============================================================================

/**
 * @brief Normalize a quaternion
 * @param q Input quaternion [w, x, y, z]
 * @return Normalized quaternion [w, x, y, z]
 * @note Quaternion format: [w, x, y, z] (scalar-first convention)
 */
inline std::vector<float> QuaternionNormalize(const std::vector<float>& q)
{
    float norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (norm < 1e-8f) {
        return {1.0f, 0.0f, 0.0f, 0.0f};
    }
    return {q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm};
}

/**
 * @brief Multiply two quaternions (Hamilton product)
 * @param q1 First quaternion [w, x, y, z]
 * @param q2 Second quaternion [w, x, y, z]
 * @return Product quaternion q1 * q2 as [w, x, y, z]
 */
inline std::vector<float> QuaternionMultiply(const std::vector<float>& q1, const std::vector<float>& q2)
{
    return {
        q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3],
        q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2],
        q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1],
        q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]
    };
}

/**
 * @brief Compute the conjugate of a quaternion
 * @param q Input quaternion [w, x, y, z]
 * @return Conjugate quaternion [w, -x, -y, -z]
 */
inline std::vector<float> QuaternionConjugate(const std::vector<float>& q)
{
    return {q[0], -q[1], -q[2], -q[3]};
}

/**
 * @brief Create quaternion from axis-angle representation
 * @param axis Rotation axis (should be normalized)
 * @param angle Rotation angle in radians
 * @return Quaternion [w, x, y, z] representing the rotation
 */
inline std::vector<float> QuaternionFromAxisAngle(const std::vector<float>& axis, float angle)
{
    float half_angle = angle * 0.5f;
    float sin_half = std::sin(half_angle);
    return {
        std::cos(half_angle),
        axis[0] * sin_half,
        axis[1] * sin_half,
        axis[2] * sin_half
    };
}

/**
 * @brief Rotate a vector by a quaternion (inverse rotation)
 * @param q Quaternion [w, x, y, z]
 * @param v Vector [x, y, z]
 * @return Rotated vector
 * @note This applies q^-1 * v * q rotation (inverse/conjugate rotation)
 */
inline std::vector<float> QuatRotateInverse(const std::vector<float>& q, const std::vector<float>& v)
{
    float q_w = q[0];
    float q_x = q[1];
    float q_y = q[2];
    float q_z = q[3];

    float v_x = v[0];
    float v_y = v[1];
    float v_z = v[2];

    float a_x = v_x * (2.0f * q_w * q_w - 1.0f);
    float a_y = v_y * (2.0f * q_w * q_w - 1.0f);
    float a_z = v_z * (2.0f * q_w * q_w - 1.0f);

    float cross_x = q_y * v_z - q_z * v_y;
    float cross_y = q_z * v_x - q_x * v_z;
    float cross_z = q_x * v_y - q_y * v_x;

    float b_x = cross_x * q_w * 2.0f;
    float b_y = cross_y * q_w * 2.0f;
    float b_z = cross_z * q_w * 2.0f;

    float dot = q_x * v_x + q_y * v_y + q_z * v_z;

    float c_x = q_x * dot * 2.0f;
    float c_y = q_y * dot * 2.0f;
    float c_z = q_z * dot * 2.0f;

    return {a_x - b_x + c_x, a_y - b_y + c_y, a_z - b_z + c_z};
}

/**
 * @brief Convert quaternion to rotation matrix (3x3)
 * @param q Quaternion [w, x, y, z]
 * @return Rotation matrix as vector of 9 elements [row-major: R00, R01, R02, R10, R11, ...]
 */
inline std::vector<float> QuaternionToRotationMatrix(const std::vector<float>& q)
{
    float w = q[0];
    float x = q[1];
    float y = q[2];
    float z = q[3];

    float xx = x * x;
    float yy = y * y;
    float zz = z * z;
    float xy = x * y;
    float xz = x * z;
    float yz = y * z;
    float wx = w * x;
    float wy = w * y;
    float wz = w * z;

    return {
        1.0f - 2.0f * (yy + zz), 2.0f * (xy - wz), 2.0f * (xz + wy),
        2.0f * (xy + wz), 1.0f - 2.0f * (xx + zz), 2.0f * (yz - wx),
        2.0f * (xz - wy), 2.0f * (yz + wx), 1.0f - 2.0f * (xx + yy)
    };
}

/**
 * @brief Convert quaternion to Euler angles (roll, pitch, yaw)
 * @param q Quaternion [w, x, y, z]
 * @return Euler angles [roll, pitch, yaw] in radians
 */
inline std::vector<float> QuaternionToEuler(const std::vector<float>& q)
{
    float w = q[0];
    float x = q[1];
    float y = q[2];
    float z = q[3];

    // Roll (rotation around X-axis)
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    float roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (rotation around Y-axis)
    float sinp = 2.0f * (w * y - z * x);
    float pitch;
    if (std::fabs(sinp) >= 1.0f)
        pitch = std::copysign(3.14159265f / 2.0f, sinp); // Use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    // Yaw (rotation around Z-axis)
    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    float yaw = std::atan2(siny_cosp, cosy_cosp);

    return {roll, pitch, yaw};
}

/**
 * @brief Extract yaw component from a quaternion
 * @param q Input quaternion [w, x, y, z]
 * @return Quaternion representing only the yaw rotation around Z-axis [w, x, y, z]
 */
inline std::vector<float> QuaternionYawOnly(const std::vector<float>& q)
{
    // Extract yaw angle from quaternion
    float siny_cosp = 2.0f * (q[0] * q[3] + q[1] * q[2]);
    float cosy_cosp = 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]);
    float yaw = std::atan2(siny_cosp, cosy_cosp);

    // Create quaternion with only yaw rotation
    float half_yaw = yaw * 0.5f;
    return {std::cos(half_yaw), 0.0f, 0.0f, std::sin(half_yaw)};
}

/**
 * @brief Transpose a 3x3 rotation matrix
 * @param mat Input matrix [9 elements, row-major]
 * @return Transposed matrix
 */
inline std::vector<float> TransposeMatrix3x3(const std::vector<float>& mat)
{
    return {
        mat[0], mat[3], mat[6],
        mat[1], mat[4], mat[7],
        mat[2], mat[5], mat[8]
    };
}

/**
 * @brief Extract first two columns of a 3x3 matrix (6 elements)
 * @param mat Input matrix [9 elements, row-major]
 * @return First two columns as vector [R00, R01, R10, R11, R20, R21]
 */
inline std::vector<float> MatrixFirstTwoColumns(const std::vector<float>& mat)
{
    return {
        mat[0], mat[1],  // First column: R00, R01
        mat[3], mat[4],  // Second column: R10, R11
        mat[6], mat[7]   // Third column: R20, R21
    };
}

#endif // VECTOR_MATH_HPP
