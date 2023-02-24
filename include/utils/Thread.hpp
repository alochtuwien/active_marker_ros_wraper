#ifndef THREAD_HPP
#define THREAD_HPP

#include <thread>
#include <pthread.h>
#include <iostream>
#include <cstring>

class Thread : public std::thread
{
  public:
    Thread() {}
    static void setScheduling(std::thread &th, int policy, int priority) {
        sched_param sch_params;
        sch_params.sched_priority = priority;
        if(pthread_setschedparam(th.native_handle(), policy, &sch_params)) {
            std::cerr << "Failed to set Thread scheduling : " << std::strerror(errno) << std::endl;
        }
    }
  private: 
};

// how to use it
// see: https://stackoverflow.com/questions/18884510/portable-way-of-setting-stdthread-priority-in-c11

// // create thread
// std::thread example_thread(example_function);

// // set scheduling of created thread
// thread::setScheduling(example_thread, SCHED_RR, 2);

#endif // THREAD_HPP