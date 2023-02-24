
#include <string>
#include <stdio.h>
#include "Thread.hpp"
#include "ExecutionTimer.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <chrono>
#include <thread>
#include <atomic>

class Trigger{

    int serial_fd = -1;
    unsigned int sercmd;
    std::chrono::milliseconds  pulse_duration_;
    std::chrono::time_point<std::chrono::high_resolution_clock> trigger_sent, trigger_received;
    std::thread thread_rx_ptr;
    ExecutionTimer<int> exectimer;
    std::atomic_bool running_;

public: 
    Trigger(std::string serial_device) : exectimer(true){
        
        serial_fd = open("/dev/ttyS0", O_RDWR | O_NONBLOCK | O_NOCTTY);
        
        sercmd = TIOCM_RTS;

        // ioctl(serial_fd, TIOCMBIS, &sercmd); // turn on
        // set_high(); // default
    }

    // void send_trigger(int pulsewidth_ms){
    //             //    thread_rx_ptr = std::thread(&TriggerSync::reading_thread, this);

        

    //     pulse_duration_ = std::chrono::milliseconds(pulsewidth_ms);

    //     trigger_sent = std::chrono::high_resolution_clock::now();
    //     exectimer.tic();
    //     ioctl(serial_fd, TIOCMBIC, &sercmd);

    //     std::this_thread::sleep_for(pulse_duration_);

    //     ioctl(serial_fd, TIOCMBIS, &sercmd);
    // }   

    void set_high(){
        ioctl(serial_fd, TIOCMBIC, &sercmd);
    }

    void set_low(){
        ioctl(serial_fd, TIOCMBIS, &sercmd);
    }


    // void stop_listen(){
    //     running_ = false;
    //     send_trigger(1);
    //     thread_rx_ptr.join();
    //     exectimer.write_log("execution_times.csv");
    // }


    void start_listen(){
        running_ = true;
        thread_rx_ptr = std::thread([&]
                                    { listening_thread(running_); });
    }

private:
   void listening_thread(std::atomic_bool &run_flag) {
    static const unsigned int ri_flag = TIOCM_RNG;


    /* Wait for positive RI transition.  TIOCMIWAIT takes a mask
    * as argument, only returning when the appropriate signal has changed.
    */
    std::cout << "start listening loop..." << std::endl;

    while(run_flag) {
        if (ioctl(serial_fd, TIOCMIWAIT, ri_flag)) {
            fprintf(stderr, "ioctl() failed waiting for RI edge [%s]\n", strerror(errno));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            //break;
        }
        // trigger_received = std::chrono::high_resolution_clock::now();

        // auto time_elapsed = trigger_received - trigger_sent;
        exectimer.toc();
        exectimer.print(false);
    }
}

};
