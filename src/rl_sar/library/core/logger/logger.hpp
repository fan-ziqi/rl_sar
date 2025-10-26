/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <iostream>
#include <iomanip>
#include <string>

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
     * @brief Print progress bar with graphical representation
     * @param percent Progress percentage (0.0-1.0)
     * @param description Progress description
     * @param bar_width Width of the progress bar in characters (default 40)
     */
    inline void PrintProgress(float percent, const std::string& description, int bar_width = 40)
    {
        if (percent >= 1.0f)
        {
            // Clear progress bar line and show completion message
            std::cout << "\r\033[K" << INFO << description << " completed" << std::endl;
            return;
        }

        int filled = static_cast<int>(percent * bar_width);
        if (filled > bar_width) filled = bar_width;

        std::string bar;
        for (int i = 0; i < bar_width; ++i)
        {
            bar += (i < filled) ? "█" : "░";
        }

        // Write to stdout
        std::cout << "\r\033[K" << INFO << "[" << PROGRESS_BAR_COLOR << bar << COLOR_RESET << "] "
                  << static_cast<int>(percent * 100) << "% - " << description << std::flush;
    }
}

#endif // LOGGER_HPP

