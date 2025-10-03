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

#endif // VECTOR_MATH_HPP
