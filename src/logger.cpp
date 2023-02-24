//
// Created by aloch on 26.01.23.
//

#include "../include/logger.hpp"
#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovariance.h"


Logger::Logger() {
    output_queue = std::make_shared<OutputQueue>(20000);
    current_outputs_map = std::make_shared<std::unordered_map<int, std::shared_ptr<OutputEntry>>>();
    output_queue_ptr = std::make_shared<OutputQueue>();
    record_status = false;
    char buffer [80];
    time_t t = time(0);
    struct tm * now = localtime( & t );
    strftime (buffer,80,"%Y-%m-%d-%H-%M-%S.csv",now);
    csv = new csvfile(buffer);
    *csv << "PC_TS" << "C_TS" << "ID" << "X" << "Y" << "Z" << "R1" << "R2" << "R3" << "DET" << endrow;
}

void Logger::start() {
    thread_ptr = std::make_shared<std::jthread>(&Logger::loggerLoop, this);
}

[[noreturn]] void Logger::loggerLoop() {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "active_logger");
    ros::NodeHandle n;
    ros::Publisher positions_pub = n.advertise<geometry_msgs::PoseStamped>("positions", 1000);

    while (true) {
        if(output_queue->try_dequeue(current_object)){
            // std::cout << "PUBLISHED" << std::endl;
            

            geometry_msgs::PoseStamped msg;
            cv::Mat rvec(3, 1, CV_64FC1);
            rvec.at<double>(0) = current_object->pose_rot.r_0;
            rvec.at<double>(1) = current_object->pose_rot.r_1;
            rvec.at<double>(2) = current_object->pose_rot.r_2;
            double angle = cv::norm(rvec);
            cv::normalize(rvec, rvec);
            msg.pose.orientation.x = rvec.at<double>(0) * sin(angle/2.0);
            msg.pose.orientation.y = rvec.at<double>(1) * sin(angle/2.0);
            msg.pose.orientation.z = rvec.at<double>(2) * sin(angle/2.0);
            msg.pose.orientation.w = cos(angle/2.0);
            
            msg.pose.position.x = current_object->pose_trans.x;
            msg.pose.position.y = current_object->pose_trans.y;
            msg.pose.position.z = current_object->pose_trans.z;

            positions_pub.publish(msg);
            ros::spinOnce();

            logToCsv(current_object);
            if(record_status.load()){

                (*current_outputs_map)[current_object->id] = current_object;

                logged_outputs++;
                if(logged_outputs > 5){
                    record_status.store(false);
                    logged_outputs = 0;
                }
            }
            
        }
        else{
            std::this_thread::sleep_for(std::chrono::microseconds(5));
        }
    }
}

std::shared_ptr<std::unordered_map<int, std::shared_ptr<OutputEntry>>> Logger::getCurrentStatus() {
    current_outputs_map = std::make_shared<std::unordered_map<int, std::shared_ptr<OutputEntry>>>();
    record_status.store(true);
    std::chrono::time_point start = std::chrono::steady_clock::now();
    while(record_status.load() && std::chrono::steady_clock::now() - start < std::chrono::microseconds (10000  )){
        std::this_thread::sleep_for(std::chrono::microseconds(5));
    }
    if(record_status.load()){
        return std::make_shared<std::unordered_map<int, std::shared_ptr<OutputEntry>>>();
    }
    else{
        return current_outputs_map;
    }

}


void Logger::logToCsv(std::shared_ptr<OutputEntry> object) {
    *csv << object->pc_timestamp
        << object->camera_timestamp
        << object->id
        << object->pose_trans.x
        << object->pose_trans.y
        << object->pose_trans.z
        << object->pose_rot.r_0
        << object->pose_rot.r_1
        << object->pose_rot.r_2
        << object->detection
        << endrow;
    // std::cout << object->pose_trans.x << " " << object->pose_trans.y << " " << object->pose_trans.z << std::endl;
}







