/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LOOP_H
#define LOOP_H

#include <iostream>
#include <thread>
#include <chrono>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <vector>
#include <sstream>
#include <iomanip>

class LoopFunc
{
public:
    LoopFunc(const std::string &name, double period, std::function<void()> func, int bindCPU = -1)
        : _name(name), _period(period), _func(func), _bindCPU(bindCPU), _running(false) {}

    void start()
    {
        _running = true;
        log("[Loop Start] named: " + _name + ", period: " + formatPeriod() + "(ms)" + (_bindCPU != -1 ? ", run at cpu: " + std::to_string(_bindCPU) : ", cpu unspecified"));
        if (_bindCPU != -1)
        {
            _thread = std::thread(&LoopFunc::loop, this);
            setThreadAffinity(_thread.native_handle(), _bindCPU);
        }
        else
        {
            _thread = std::thread(&LoopFunc::loop, this);
        }
        _thread.detach();
    }

    void shutdown()
    {
        {
            std::unique_lock<std::mutex> lock(_mutex);
            _running = false;
            _cv.notify_one();
        }
        if (_thread.joinable())
        {
            _thread.join();
        }
        log("[Loop End] named: " + _name);
    }

private:
    std::string _name;
    double _period;
    std::function<void()> _func;
    int _bindCPU;
    std::atomic<bool> _running;
    std::mutex _mutex;
    std::condition_variable _cv;
    std::thread _thread;

    void loop()
    {
        while (_running)
        {
            auto start = std::chrono::steady_clock::now();

            _func();

            auto end = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            auto sleepTime = std::chrono::milliseconds(static_cast<int>((_period * 1000) - elapsed.count()));
            if (sleepTime.count() > 0)
            {
                std::unique_lock<std::mutex> lock(_mutex);
                if (_cv.wait_for(lock, sleepTime, [this]
                                 { return !_running; }))
                {
                    break;
                }
            }
        }
    }

    std::string formatPeriod() const
    {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(0) << _period * 1000;
        return stream.str();
    }

    void log(const std::string &message)
    {
        static std::mutex logMutex;
        std::lock_guard<std::mutex> lock(logMutex);
        std::cout << message << std::endl;
    }

    void setThreadAffinity(std::thread::native_handle_type threadHandle, int cpuId)
    {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(cpuId, &cpuset);
        if (pthread_setaffinity_np(threadHandle, sizeof(cpu_set_t), &cpuset) != 0)
        {
            std::ostringstream oss;
            oss << "Error setting thread affinity: CPU " << cpuId << " may not be valid or accessible.";
            throw std::runtime_error(oss.str());
        }
    }
};

#endif // LOOP_H
