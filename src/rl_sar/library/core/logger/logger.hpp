/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <iostream>
#include <iomanip>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>

namespace LOGGER
{
    const char *const INFO    = "\033[0;37m[INFO]   \033[0m ";
    const char *const WARNING = "\033[0;33m[WARNING]\033[0m ";
    const char *const ERROR   = "\033[0;31m[ERROR]  \033[0m ";
    const char *const DEBUG   = "\033[0;32m[DEBUG]  \033[0m ";
    const char *const NOTE    = "\033[0;34m[NOTE]   \033[0m ";

    // Progress bar color
    const char *const PROGRESS_BAR_COLOR = "\033[36m"; // Cyan
    const char *const COLOR_RESET = "\033[0m";

    /**
     * @brief Get terminal width
     * @return Terminal width in columns, or 80 as default if unavailable
     */
    inline int GetTerminalWidth()
    {
        struct winsize w;
        if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &w) == 0 && w.ws_col > 0)
        {
            return w.ws_col;
        }
        return 80; // Default fallback width
    }

    /**
     * @brief Print progress bar with graphical representation
     * @param percent Progress percentage (0.0-1.0)
     * @param description Progress description
     * @note The bar width will be automatically adjusted based on terminal width
     */
    inline void PrintProgress(float percent, const std::string& description)
    {
        if (percent >= 1.0f)
        {
            // Clear progress bar line and show completion message
            std::cout << "\r\033[K" << INFO << description << " completed" << std::endl;
            return;
        }

        int term_width = GetTerminalWidth();

        // Fixed overhead: "[INFO]   " (9) + "[] " (3) + "100.00% - " (10) = 22 characters
        int fixed_overhead = 22;

        // Priority: keep description complete, use remaining space for progress bar
        int desc_length = description.length();

        // Calculate remaining space after description
        int remaining_space = term_width - fixed_overhead - desc_length;

        // Determine bar width based on remaining space
        int bar_width = remaining_space;

        // Ensure bar width is within reasonable bounds
        if (bar_width < 5)
        {
            bar_width = 5;  // Minimum bar width to show some progress
        }
        else if (bar_width > 50)
        {
            bar_width = 50;  // Maximum bar width for readability
        }

        int filled = static_cast<int>(percent * bar_width);
        if (filled > bar_width) filled = bar_width;

        std::string bar;
        for (int i = 0; i < bar_width; ++i)
        {
            bar += (i < filled) ? "█" : "░";
        }

        // Write to stdout with percentage showing 2 decimal places
        std::cout << "\r\033[K" << INFO << "[" << PROGRESS_BAR_COLOR << bar << COLOR_RESET << "] "
                  << std::fixed << std::setprecision(2) << (percent * 100) << "% - " << description << std::flush;
    }
}

#endif // LOGGER_HPP

