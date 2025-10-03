/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file test_vector_math.cpp
 * @brief Test program for vector_math.hpp functionality
 *
 * This program tests all the overloaded operators and functions in vector_math.hpp
 * by performing various operations and printing the results for visual verification.
 */

#include <iostream>
#include <vector>
#include <iomanip>
#include "vector_math.hpp"

int main()
{
    std::cout << "=== Vector Math Library Test ===" << std::endl << std::endl;

    // Test vectors
    std::vector<float> vec1 = {1.0f, 2.0f, 3.0f, 4.0f};
    std::vector<float> vec2 = {2.0f, 4.0f, 6.0f, 8.0f};
    std::vector<float> vec3 = {0.5f, 1.0f, 1.5f, 2.0f};

    std::cout << "vec1: " << vec1 << std::endl;
    std::cout << "vec2: " << vec2 << std::endl;
    std::cout << "vec3: " << vec3 << std::endl << std::endl;

    // Test scalar clamp
    std::cout << "=== Scalar Clamp Tests ===" << std::endl;
    std::cout << "clamp(5.5f, 2.0f, 10.0f) = " << clamp(5.5f, 2.0f, 10.0f) << std::endl;
    std::cout << "clamp(1.0f, 2.0f, 10.0f) = " << clamp(1.0f, 2.0f, 10.0f) << std::endl;
    std::cout << "clamp(15.0f, 2.0f, 10.0f) = " << clamp(15.0f, 2.0f, 10.0f) << std::endl;
    std::cout << std::endl;

    // Test vector clamp with single bounds
    std::cout << "=== Vector Clamp (Single Bounds) Tests ===" << std::endl;
    std::cout << "clamp(vec1, 1.5f, 3.5f) = " << clamp(vec1, 1.5f, 3.5f) << std::endl;
    std::cout << std::endl;

    // Test vector clamp with vector bounds
    std::cout << "=== Vector Clamp (Vector Bounds) Tests ===" << std::endl;
    std::vector<float> min_vals = {1.2f, 1.8f, 2.5f, 3.2f};
    std::vector<float> max_vals = {1.8f, 2.5f, 3.2f, 4.5f};
    std::cout << "min_vals: " << min_vals << std::endl;
    std::cout << "max_vals: " << max_vals << std::endl;
    std::cout << "clamp(vec1, min_vals, max_vals) = " << clamp(vec1, min_vals, max_vals) << std::endl;
    std::cout << std::endl;

    // Test arithmetic operators
    std::cout << "=== Arithmetic Operators Tests ===" << std::endl;
    std::cout << "vec1 * 2.5f = " << (vec1 * 2.5f) << std::endl;
    std::cout << "3.0f * vec1 = " << (3.0f * vec1) << std::endl;
    std::cout << "vec2 / 2.0f = " << (vec2 / 2.0f) << std::endl;
    std::cout << "vec1 * vec3 = " << (vec1 * vec3) << std::endl;
    std::cout << "vec2 / vec1 = " << (vec2 / vec1) << std::endl;
    std::cout << "vec1 + vec2 = " << (vec1 + vec2) << std::endl;
    std::cout << "vec2 - vec1 = " << (vec2 - vec1) << std::endl;
    std::cout << "-vec1 = " << (-vec1) << std::endl;
    std::cout << std::endl;

    // Test compound assignment operators
    std::cout << "=== Compound Assignment Operators Tests ===" << std::endl;

    std::vector<float> test_vec1 = vec1;
    std::cout << "test_vec1 (before): " << test_vec1 << std::endl;
    test_vec1 += vec3;
    std::cout << "test_vec1 after += vec3: " << test_vec1 << std::endl;

    std::vector<float> test_vec2 = vec2;
    std::cout << "test_vec2 (before): " << test_vec2 << std::endl;
    test_vec2 -= vec1;
    std::cout << "test_vec2 after -= vec1: " << test_vec2 << std::endl;

    std::vector<float> test_vec3 = vec1;
    std::cout << "test_vec3 (before): " << test_vec3 << std::endl;
    test_vec3 *= 2.0f;
    std::cout << "test_vec3 after *= 2.0f: " << test_vec3 << std::endl;

    std::vector<float> test_vec4 = vec2;
    std::cout << "test_vec4 (before): " << test_vec4 << std::endl;
    test_vec4 /= 2.0f;
    std::cout << "test_vec4 after /= 2.0f: " << test_vec4 << std::endl;
    std::cout << std::endl;

    // Test with different numeric types
    std::cout << "=== Different Numeric Types Tests ===" << std::endl;

    std::vector<double> double_vec = {1.1, 2.2, 3.3};
    std::cout << "double_vec * 2.0 = " << (double_vec * 2.0) << std::endl;

    std::vector<int> int_vec = {10, 20, 30};
    std::cout << "int_vec + {5, 10, 15} = " << (int_vec + std::vector<int>{5, 10, 15}) << std::endl;
    std::cout << std::endl;

    // Test edge cases
    std::cout << "=== Edge Cases Tests ===" << std::endl;

    std::vector<float> short_vec = {1.0f, 2.0f};
    std::vector<float> long_vec = {10.0f, 20.0f, 30.0f, 40.0f, 50.0f};
    std::cout << "short_vec: " << short_vec << std::endl;
    std::cout << "long_vec: " << long_vec << std::endl;
    std::cout << "short_vec + long_vec (size = min): " << (short_vec + long_vec) << std::endl;

    std::vector<float> empty_bounds;
    std::cout << "clamp(vec1, empty, empty): " << clamp(vec1, empty_bounds, empty_bounds) << std::endl;
    std::cout << std::endl;

    // Complex expression test
    std::cout << "=== Complex Expression Test ===" << std::endl;
    std::cout << "(vec1 + vec2) * 0.5f - vec3 / 2.0f = " << ((vec1 + vec2) * 0.5f - vec3 / 2.0f) << std::endl;
    std::cout << std::endl;

    std::cout << "=== All Tests Completed ===" << std::endl;

    return 0;
}
