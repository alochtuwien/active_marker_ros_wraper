#include <iostream>

#include "include/utils/options.hpp"
#include "include/camera.hpp"
#include "include/utils/visualization.hpp"
#include "include/markers.hpp"
#include "include/runtime_manager.hpp"
#include "include/utils/trigger.hpp"

int main(int argc, char *argv[]) {




    //Parse options
    Utils::Options::Parser parser(argc, argv);

    

    if (parser.current_setup.cam_config.is_recording){
        std::cout << "Using recording: " << parser.current_setup.cam_config.file_path << std::endl;
    }
    else{
        std::cout << "Using camera with biases from: " << parser.current_setup.cam_config.biases_file << std::endl;
        std::cout << "Using camera config from: " << parser.current_setup.cam_config.config_file_path << std::endl;
    }

    Camera cam(&parser.current_setup.getCamConfig());
    cam.initialize_camera();

    MarkersManager markers(parser.current_setup.getMarkerConfig(), &cam);

    RuntimeManager runtime(markers, &cam);
    runtime.buffers.setInputBuffer(cam.reader.buffers.getOutputBuffer());

    VisualizationController vis(cam.width, cam.height, &cam, &runtime);
    vis.buffers.setInputBuffer(cam.reader.buffers.getOutputBuffer());

    cam.start();


    std::string serial_device = "/dev/ttyS0";
    Trigger trg(serial_device);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // trg.set_high();

    int cnt = 0;

    // auto time_now = std::chrono::system_clock::now();
    std::cout << "Start recording" << cnt << std::endl;

    auto recording_start = std::chrono::system_clock::now();
    auto recording_start_epoch = recording_start.time_since_epoch();
    auto recording_start_value = std::chrono::duration_cast<std::chrono::microseconds>(recording_start_epoch);

    auto now_ms = std::chrono::system_clock::now();
    auto epoch = now_ms.time_since_epoch();
    auto value = std::chrono::duration_cast<std::chrono::microseconds>(epoch);
    // std::cout << "LOW " << value.count() << std::endl;

    // trg.set_low();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    now_ms = std::chrono::system_clock::now();
    epoch = now_ms.time_since_epoch();
    value = std::chrono::duration_cast<std::chrono::microseconds>(epoch);
    std::cout << "HIGH " << value.count() << std::endl;
    trg.set_high();

    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    now_ms = std::chrono::system_clock::now();
    epoch = now_ms.time_since_epoch();
    value = std::chrono::duration_cast<std::chrono::microseconds>(epoch);
    std::cout << "LOW " << value.count() << std::endl;
    trg.set_low();
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // now_ms = std::chrono::system_clock::now();
    // epoch = now_ms.time_since_epoch();
    // value = std::chrono::duration_cast<std::chrono::microseconds>(epoch);
    // std::cout << "HIGH " << value.count() << std::endl;
    // trg.set_high();



    vis.start();
    runtime.start();

    // std::this_thread::sleep_for(std::chrono::milliseconds(300));

    cam.reader.start();


    bool recording = true;
    while (!vis.isFinished()) {
        if(recording){
            std::this_thread::sleep_for(std::chrono::milliseconds(30000));
        
            now_ms = std::chrono::system_clock::now();
            epoch = now_ms.time_since_epoch();
            value = std::chrono::duration_cast<std::chrono::microseconds>(epoch);
            std::cout << "END HIGH " << value.count() << std::endl;
            trg.set_high();

            
            auto recording_end = std::chrono::system_clock::now();
            auto recording_end_epoch = recording_end.time_since_epoch();
            auto recording_end_value = std::chrono::duration_cast<std::chrono::microseconds>(recording_end_epoch);

            std::cout << "LENGTH " << recording_end_value.count() - recording_start_value.count() << std::endl;
            recording = false;
        }
    }

    return 0;
}
