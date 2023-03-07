//
// Created by aloch on 03.01.23.
//

#include "../include/markers.hpp"

//Model3D::Model3D() {
//    clearModel();
//}
//
//void Model3D::clearModel() {
//        vert_object_cs_.clear();
//        vert_world_cs_.clear();
//        vert_frame_cs_.clear();
//}
//
//void Model3D::setPose(const Eigen::Vector3f &trans, const Eigen::Quaternionf &rot) {
//    translation_ = trans;
//    rot_quaternion_ = rot;
//    transform();
//}
//
//Eigen::Vector2f Model3D::project(const Eigen::Vector3f &point)
//{
//    Eigen::Vector3f tmp = camera_matrix_eigen_ * point;
//    if (abs(tmp[2]) > 0)
//    {
//        tmp /= tmp[2];
//    }
//    return Eigen::Vector2f{tmp[0], tmp[1]};
//}
//
//Eigen::Vector2f Model3D::project(unsigned int ivert)
//{
//    Eigen::Vector3f tmp = camera_matrix_eigen_ * vert_world_cs_[ivert];
//    if (abs(tmp[2]) > 0)
//    {
//        tmp /= tmp[2];
//    }
//    return Eigen::Vector2f{tmp[0], tmp[1]};
//}
//
//void Model3D::transform()
//{
//    projected_center_ = project(translation_);
//
//    max_projected_distance_ = 0;
//    for (unsigned int ii = 0; ii < vert_object_cs_.size(); ii++)
//    {
//        vert_world_cs_[ii] = rot_quaternion_ * vert_object_cs_[ii] + translation_;
//        vert_frame_cs_[ii] = project(ii);
//        if (ii < 4)
//        {
//            if ((vert_frame_cs_[ii] - projected_center_).norm() > max_projected_distance_)
//                max_projected_distance_ = (vert_frame_cs_[ii] - projected_center_).norm();
//        }
//    }
//    max_projected_distance_ *= 1.2;
//}


MarkersManager::MarkersManager(Utils::Options::MarkersSetup setup, Camera *cam) {
    config = setup;
    cam_obj = cam;
}

std::vector<std::shared_ptr<Marker>> MarkersManager::matchMarkers(std::vector<double> frequencies,
                                                 std::vector<std::vector<cv::Point>> filtered_contours,
                                                 long long detection_time)
{
    std::vector<std::pair<std::vector<int>, int>> output;
    std::vector<int> matches(frequencies.size(), 0);
//    std::vector<Marker> detected_markers;
    std::vector<std::shared_ptr<Marker>> detected_markers;

    for (unsigned int i = 0; i < config.ids.size(); ++i){
        std::vector<int> assigned(config.frequencies[i].size(), -1);
        std::vector<std::vector<cv::Point>> assigned_contours(config.frequencies[i].size());
        bool not_found = false;

        for (unsigned int j = 0; j < config.frequencies[i].size(); ++j){
            auto it = std::find(frequencies.begin(), frequencies.end(), config.frequencies[i][j]);
            if (it != frequencies.end())
            {
                int index = it - frequencies.begin();
                assigned[j] = index;
            }
            else{
                not_found = true;
                break;
            }
        }

        if (!not_found){
            output.push_back(std::make_pair(assigned, i));
            for (unsigned int j = 0; j<assigned.size(); ++j){
                assigned_contours[j] = filtered_contours[assigned[j]];
            }

            detected_markers.push_back(std::make_shared<Marker>(config.ids[i],
                                                                config.coordinates[i],
                                                                assigned_contours,
                                                                detection_time,
                                                                cam_obj,
                                                                logger_ptr,
                                                                this));
        }
    }
    return detected_markers;
}

void MarkersManager::registerDetections(std::vector<double> frequencies,
                                        std::vector<std::vector<cv::Point>> filtered_contours,
                                        long long int detection_timestamp) {
    auto detected_markers = matchMarkers(frequencies, filtered_contours, detection_timestamp);

    tracked_markers_ids_mutex.lock();
    for (auto &marker : detected_markers){
//        std::cout << tracked_markers_ids.size() << std::endl;
        auto it = std::find(tracked_markers_ids.begin(), tracked_markers_ids.end(), marker->id);
        if (it != tracked_markers_ids.end()) {

        }
        else{
            tracked_markers_ids.push_back(marker->id);
            markers_to_spawn.push_back(marker);
        }
    }
    tracked_markers_ids_mutex.unlock();
};

void MarkersManager::deregisterMarker(Marker *marker_to_remove) {
    tracked_markers_ids_mutex.lock();
    auto it = std::find(tracked_markers_ids.begin(), tracked_markers_ids.end(), marker_to_remove->id);
    if (it != tracked_markers_ids.end()) {
        tracked_markers_ids.erase(it);
    }
    tracked_markers_ids_mutex.unlock();
    tracked_markers_mutex.lock();
    for(size_t i = 0; i < tracked_markers.size(); i++){
        if (tracked_markers[i].get() == marker_to_remove){
            markers_to_destroy.push_back(tracked_markers[i]);
            tracked_markers_mutex.unlock();
            break;
        }
    }
}

void MarkersManager::spawnMarkers(std::shared_ptr<Deque> deque, Buffers *input_buffers){
    tracked_markers_mutex.lock();

    for (auto &marker : markers_to_destroy){
        auto it = std::find(tracked_markers.begin(), tracked_markers.end(), marker);
        if (it != tracked_markers.end()) {
            tracked_markers.erase(it);
        }
    }
    markers_to_destroy.clear();
    for (auto &marker : markers_to_spawn){

        tracked_markers.push_back(marker);
        tracked_markers.back()->startTracking(deque, input_buffers);
    }
    tracked_markers_mutex.unlock();
    markers_to_spawn.clear();

}

std::shared_ptr<OutputEntry> Marker::getOutput(bool detection = false) {
    current_output = std::make_shared<OutputEntry>();
    current_output->id = id;
    current_output->camera_timestamp = current_camera_timestamp;
    current_output->pc_timestamp = current_pc_timestamp;
    current_output->detection = detection;
    current_output->pose_trans.x = t_vec.at<double>(0);
    current_output->pose_trans.y = t_vec.at<double>(1);
    current_output->pose_trans.z = t_vec.at<double>(2);
    current_output->pose_rot.r_0 = r_vec.at<double>(0);
    current_output->pose_rot.r_1 = r_vec.at<double>(1);
    current_output->pose_rot.r_2 = r_vec.at<double>(2);





    return current_output;
}


void Marker::startTracking(std::shared_ptr<Deque> deque, Buffers *input_buffers) {
    initial_buffer = deque;
    input_buffers_ptr = input_buffers;
    buffers.setInputBuffer(input_buffers_ptr->getOutputBuffer());
    tracking_thread = std::make_shared<std::jthread>(&Marker::track, this);
}

void Marker::stopTracking(){
    input_buffers_ptr->deregisterBuffer(buffers.getInputBuffer());
}

void Marker::track() {

    bool status = cv::solvePnP(objectpoints_3d, detected_initial_points, cam_obj->camera_matrix_cv, cam_obj->dist_coeffs, r_vec, t_vec, false);

    if (!status){
        stopTracking();
        markers_manager_ptr->deregisterMarker(this);
    }

    std::vector<cv::Point2d> projected;
    cv::projectPoints(objectpoints_3d, r_vec, t_vec, cam_obj->camera_matrix_cv, cam_obj->dist_coeffs, projected);


    std::vector<cv::Point2d> center_projected;
    std::vector<cv::Point3d> center_3d;
    center_3d.push_back(cv::Point3d(0.0,0.0,0.0));

    cv::projectPoints(center_3d, r_vec, t_vec, cam_obj->camera_matrix_cv, cam_obj->dist_coeffs, center_projected);

    centerpoint_3d = center_3d[0];
    projected_centerpoint = center_projected[0];

    projected_objectpoints = projected;

    for (auto &point_projected : detected_initial_points){
        tracked_blobs.push_back(std::make_shared<TrackedBlob>(point_projected));
    }
    pnp_running = false;
    new_result = false;

    final_accumulation_time = detected_at + accumulation_period;
    working_buffer = std::make_shared<Deque>();

    auto out = getOutput(true);
    logger_ptr->output_queue->enqueue(out);

    size_t i = 0;

    while (!tracking_lost) {
        if(i < initial_buffer->size()){
            buffers.current_batch = initial_buffer->at(i);
            if(buffers.current_batch->first->front()->t >= detected_at){
                new_batch = true;
            }
            i++;
        }
        else{
            new_batch = buffers.getBatch();

        }
        if(new_batch){
            for (auto ev : *buffers.current_batch->first){
                if(!pnp_running.load()){
                    pnp_running = true;
                    blobs_for_current_pnp.clear();
                    average_distances.clear();
                    for (auto blob : tracked_blobs){
                        blobs_for_current_pnp.push_back(blob->get_output());
                        average_distances.push_back(blob->avg_distance);
                    }
                    current_pnp_thread = std::jthread([&](){
                        solvePnP();
                    });
                }
                if(tracking_lost.load()) {
                    break;
                }

                for (auto &blob : tracked_blobs) {
                    if(blob->update(ev)){
                        break;
                    }
                }
                current_camera_timestamp = ev->t;
            }
        }
    }
    stopTracking();
    markers_manager_ptr->deregisterMarker(this);
}



Marker::Marker(int idx,
               std::vector<cv::Point3d> points_3d,
               std::vector<std::vector<cv::Point>> contours,
               long long detection_time,
               Camera *cam,std::shared_ptr<Logger> logger,
               MarkersManager *markers_manager)
{
    markers_manager_ptr = markers_manager;
    logger_ptr = logger;
    detected_at = detection_time;
    current_camera_timestamp = detection_time;
    auto now_ms = std::chrono::system_clock::now();
    auto epoch = now_ms.time_since_epoch();
    auto value = std::chrono::duration_cast<std::chrono::microseconds>(epoch);

    current_pc_timestamp = value.count();


    id = idx;
    std::vector<cv::Point2d> centers;
    for (unsigned int i=0; i<contours.size(); i++){
        cv::Moments M = cv::moments(contours[i]);
        cv::Point2f center(M.m10/M.m00, M.m01/M.m00);
        centers.push_back(center);
    }
    objectpoints_3d = points_3d;
    detected_initial_points = centers;

    cam_obj = cam;

}

void Marker::solvePnP() {
    std::vector<cv::Point2d> blob_centers;
    std::vector<double> distances;

    for (auto blob : blobs_for_current_pnp){
        blob_centers.emplace_back(cv::Point2d(blob.x, blob.y));
    }
    for (auto distance : average_distances){
        distances.push_back(distance);
    }

    bool status = cv::solvePnP(objectpoints_3d,
                 blob_centers,
                 cam_obj->camera_matrix_cv,
                 cam_obj->dist_coeffs,
                 r_vec,
                 t_vec,
                 true,
                 solve_method);

    if (!status) {
        tracking_lost = true;
        return;
    }

    cv::Mat projected_tmp;
    cv::projectPoints(objectpoints_3d, r_vec, t_vec, cam_obj->camera_matrix_cv, cam_obj->dist_coeffs, projected_tmp);
    projected_tmp.convertTo(projected_objectpoints, cv::Mat(projected_objectpoints).type());

    auto now_ms = std::chrono::system_clock::now();
    auto epoch = now_ms.time_since_epoch();
    auto value = std::chrono::duration_cast<std::chrono::microseconds>(epoch);
    current_pc_timestamp = value.count();

    double error_sum = 0.;
    double dist_sum = 0.;
    for (size_t i = 0; i < blob_centers.size() ; i++) {
        error_sum += cv::norm(projected_objectpoints[i] - blob_centers[i]);
        dist_sum += (3*distances[i]);

    }
    if(error_sum > dist_sum){
        std::cout << "tracking lost" << std::endl;
        std::cout << error_sum << " vs " << dist_sum << std::endl;
        tracking_lost = true;
    }


    auto out = getOutput();
    out->tracker_outputs = blobs_for_current_pnp;
    logger_ptr->output_queue->enqueue(out);

    pnp_running = false;
    new_result = true;

}

