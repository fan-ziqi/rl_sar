/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "observation_buffer.hpp"

#include <stdexcept>
#include <string>
#include <vector>

void expect_equal(const std::vector<float>& actual,
                  const std::vector<float>& expected,
                  const std::string& description)
{
    if (actual != expected)
    {
        throw std::runtime_error("Unexpected observation history: " + description);
    }
}

void test_first_observation_fills_history()
{
    ObservationBuffer buffer(1, {2}, 3, "time");

    buffer.insert({1.0f, 2.0f});
    expect_equal(
        buffer.get_obs_vec({0, 1, 2}),
        {1.0f, 2.0f, 1.0f, 2.0f, 1.0f, 2.0f},
        "initial observation");

    buffer.insert({3.0f, 4.0f});
    expect_equal(
        buffer.get_obs_vec({0, 1, 2}),
        {3.0f, 4.0f, 1.0f, 2.0f, 1.0f, 2.0f},
        "second observation");
}

void test_reset_replaces_stale_history()
{
    ObservationBuffer buffer(1, {2}, 3, "time");
    buffer.insert({1.0f, 2.0f});
    buffer.insert({3.0f, 4.0f});

    buffer.reset({0}, {9.0f, 8.0f});

    expect_equal(
        buffer.get_obs_vec({0, 1, 2}),
        {9.0f, 8.0f, 9.0f, 8.0f, 9.0f, 8.0f},
        "reset observation");
}

void test_term_priority_preserves_seeded_history()
{
    ObservationBuffer buffer(1, {1, 2}, 2, "term");
    buffer.insert({1.0f, 10.0f, 11.0f});
    buffer.insert({2.0f, 20.0f, 21.0f});

    expect_equal(
        buffer.get_obs_vec({0, 1}),
        {2.0f, 1.0f, 20.0f, 21.0f, 10.0f, 11.0f},
        "term priority");
}

void test_reset_only_changes_selected_environment()
{
    ObservationBuffer buffer(2, {1}, 2, "time");
    buffer.insert({1.0f});
    buffer.reset({1}, {9.0f});

    expect_equal(buffer.get_obs_vec({0, 1}), {1.0f, 1.0f, 9.0f, 9.0f}, "selected reset");

    buffer.insert({2.0f});
    expect_equal(buffer.get_obs_vec({0, 1}), {2.0f, 1.0f, 2.0f, 9.0f}, "selected update");
}

void test_history_index_is_independent_of_term_count()
{
    ObservationBuffer buffer(1, {2}, 4, "time");
    buffer.insert({5.0f, 6.0f});

    expect_equal(buffer.get_obs_vec({3}), {5.0f, 6.0f}, "oldest history index");
}

void test_reset_ignores_wrong_observation_size()
{
    ObservationBuffer buffer(1, {2}, 3, "time");
    buffer.insert({1.0f, 2.0f});
    buffer.reset({0}, {9.0f});

    expect_equal(
        buffer.get_obs_vec({0, 1, 2}),
        {1.0f, 2.0f, 1.0f, 2.0f, 1.0f, 2.0f},
        "invalid reset observation");
}

int main()
{
    test_first_observation_fills_history();
    test_reset_replaces_stale_history();
    test_term_priority_preserves_seeded_history();
    test_reset_only_changes_selected_environment();
    test_history_index_is_independent_of_term_count();
    test_reset_ignores_wrong_observation_size();
    return 0;
}
