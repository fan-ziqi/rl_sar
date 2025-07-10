/// @file dr_timer.h
/// @author vcb (www.deeprobotics.cn)
/// @brief 
/// @version 0.1
/// @date 2023-03-17
/// @copyright Copyright (c) 2023

#ifndef DR_TIMER_H_
#define DR_TIMER_H_

#include <iostream>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <chrono>
#include <sys/timerfd.h>
#include <sys/epoll.h>

/// @brief The DRTimer class provides timer functionality with millisecond precision.
class DRTimer {
  private:
    int tfd_;    
    int efd_;   
    int fds_, ret_;
    struct epoll_event ev_, *evptr_;

  public:
    /// @brief Initializes the timer with a specified interval in milliseconds. 
    /// @param ms The interval of the timer in milliseconds.
    void TimeInit(int ms);

    /// @brief Handles timer interrupt events.
    /// @return Returns the number of interrupts that have occurred since the last call to this function.
    bool TimerInterrupt(void);

    /// @brief Calculates the current time relative to a given start time.
    /// @param start_time The start time to calculate the relative time from.
    /// @return Returns the relative time from the given start time in seconds.
    double GetIntervalTime(double start_time);

    /// @brief Gets the current system time in seconds.
    /// @return Returns the current system time in seconds.
    double GetCurrentTime(void);
};



#endif  ///< PARSE_CMD_H_