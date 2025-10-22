/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LOGGER_HPP
#define LOGGER_HPP

namespace LOGGER
{
    const char *const INFO    = "\033[0;37m[INFO]   \033[0m ";
    const char *const WARNING = "\033[0;33m[WARNING]\033[0m ";
    const char *const ERROR   = "\033[0;31m[ERROR]  \033[0m ";
    const char *const DEBUG   = "\033[0;32m[DEBUG]  \033[0m ";
    const char *const NOTE    = "\033[0;34m[NOTE]   \033[0m ";
}

#endif // LOGGER_HPP

