#ifndef EXECUTIONTIMER_HPP
#define EXECUTIONTIMER_HPP

#include <chrono>
#include <iostream>
#include <fstream>
#include <vector>

template <typename T>
struct ExecTime
{
    uint64_t timestamp;
    int64_t execution_time;
    T data;
};

template <typename T>
struct ExecutionTimer
{
    using duration = std::chrono::duration<int64_t, std::micro>;
    int64_t min, max, avg, execution_time;
    duration elapsed;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    const int N = 100;
    bool iniOK = false;
    bool log_;

    std::vector<ExecTime<T>> exectime_vec;
    std::ofstream logfile;

    std::chrono::time_point<std::chrono::high_resolution_clock> app_start_time;

    ExecutionTimer()
    {
        ExecutionTimer(false);
    }

    ExecutionTimer(bool log) : exectime_vec()
    {
        log_ = log;
        app_start_time = std::chrono::high_resolution_clock::now();
    }

    ~ExecutionTimer()
    {

        // if (log_)
        // {
        //     write_log();
        // }
    }

    void write_log(std::string filepath)
    {
        write_log(filepath.c_str());
    }

    void write_log(const char *filepath)
    {
        std::cout << "write log with size: " << exectime_vec.size() << std::endl;

        if (log_)
        {
            logfile.open(filepath);
            logfile << "timestamp; execution_time; data\n";

            for (auto e : exectime_vec)
            {
                logfile << e.timestamp << "; " << e.execution_time << "; " << e.data << "\n";
            }
            logfile.flush();
            logfile.close();
        }

        std::cout << "write log completed!" << std::endl;
    }

    void reset()
    {
        avg = execution_time;
        max = execution_time;
        min = execution_time;
    }

    void tic()
    {
        start_time = std::chrono::high_resolution_clock::now();
    }

    void toc(T data)
    {
        auto stop_time = std::chrono::high_resolution_clock::now();
        elapsed = std::chrono::duration_cast<std::chrono::microseconds>(stop_time - start_time);
        execution_time = elapsed.count();

        if (log_)
        {
            ExecTime<T> e;
            auto app_runtime = std::chrono::duration_cast<std::chrono::microseconds>(stop_time - app_start_time);
            e.timestamp = app_runtime.count();
            e.execution_time = execution_time;
            e.data = data;
            exectime_vec.push_back(e);
        }

        if (!iniOK)
        {
            avg = execution_time;
            max = execution_time;
            min = execution_time;
            iniOK = true;
        }
        else
        {
            min = std::min(min, execution_time);
            max = std::max(max, execution_time);
            avg = ((N - 1) * avg + execution_time) / N;
        }
    }

    void toc()
    {
        toc(0);
    }

    void print(bool reverse_carriage)
    {
        std::cout << "execution time avg: " << avg << "\tmin: " << min << "\tmax: " << max;
        if (reverse_carriage)
        {
            std::cout << "\r" << std::flush;
        }
        else
        {
            std::cout << std::endl;
        }
    }
};

#endif // EXECUTIONTIMER_HPP