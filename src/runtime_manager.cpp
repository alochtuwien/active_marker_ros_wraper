//
// Created by aloch on 05.01.23.
//

#include "../include/runtime_manager.hpp"

RuntimeManager::RuntimeManager(MarkersManager &markers_manager, Camera *cam) {
    logger = std::make_shared<Logger>();
    markers = &markers_manager;
    markers->setLogger(logger);
    detector = new DetectionAlgorithm(cam->width, cam->height, markers, logger);
}

void RuntimeManager::start() {
    logger->start();

    detector->buffers.setInputBuffer(buffers.getInputBuffer());
    detector->start();


}




