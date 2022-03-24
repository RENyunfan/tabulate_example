//
// Created by yunfan on 2022/3/22.
//


#ifndef INCLUDE_DEVICE_MONITOR_HPP
#define INCLUDE_DEVICE_MONITOR_HPP

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "tabulate/tabulate.hpp"
#include "iostream"
#include "string"
using namespace tabulate;
using namespace std;

class DeviceMonitor{
private:

    struct DeviceState{
        ros::Time last_call_back{0};
        double hz;
        double* dt_queue;
        string name;
    };


    template<class T>
    bool LoadParam(string param_name, T &param_value, T default_value) {
        if (nh_.getParam(param_name, param_value)) {
            printf("\033[0;32m Load param %s succes: \033[0;0m", param_name.c_str());
            cout << param_value << endl;
            return true;
        } else {
            printf("\033[0;33m Load param %s failed, use default value: \033[0;0m", param_name.c_str());
            param_value = default_value;
            cout << param_value << endl;
            return false;
        }
    }

    template<class T>
    bool LoadParam(string param_name, vector<T> &param_value, vector<T> default_value) {
        if (nh_.getParam(param_name, param_value)) {
            printf("\033[0;32m Load param %s succes: \033[0;0m", param_name.c_str());
            for (int i = 0; i < param_value.size(); i++) {
                cout << param_value[i] << " ";
            }
            cout << endl;
            return true;
        } else {
            printf("\033[0;33m Load param %s failed, use default value: \033[0;0m", param_name.c_str());
            param_value = default_value;
            for (int i = 0; i < param_value.size(); i++) {
                cout << param_value[i] << " ";
            }
            cout << endl;
            return false;
        }
    }

    void ComputeAverageTime(DeviceState & ds){
        double all = 0;
        for(int i = 0 ; i < cfg_.sample_num; i++) {
            all += ds.dt_queue[i];
        }

        ds.hz = 1.0 / (all/cfg_.sample_num);

    }

public:
    DeviceMonitor(const ros::NodeHandle & nh){
        nh_ = nh;
        // Load params
        LoadParam("camera1_topic",cfg_.camera1_topic,string("cam1"));
        LoadParam("camera2_topic",cfg_.camera2_topic,string("cam2"));
        LoadParam("camera3_topic",cfg_.camera3_topic,string("cam3"));
        LoadParam("camera4_topic",cfg_.camera4_topic,string("cam4"));
        LoadParam("camera5_topic",cfg_.camera5_topic,string("cam5"));

        LoadParam("lidar1_topic",cfg_.lidar1_topic,string("lidar1"));
        LoadParam("lidar2_topic",cfg_.lidar2_topic,string("lidar2"));
        LoadParam("lidar3_topic",cfg_.lidar3_topic,string("lidar3"));
        LoadParam("lidar4_topic",cfg_.lidar4_topic,string("lidar4"));

        LoadParam("apx_topic", cfg_.apx_topic, string("apx"));

        LoadParam("print_hz", cfg_.desired_hz, 10.0);
        LoadParam("print_hz", cfg_.print_hz, 5.0);
        LoadParam("sample_num", cfg_.sample_num, 20);

        for(int i = 0 ; i < 10 ; i++){
            devices_[i].dt_queue = new double[cfg_.sample_num];
        }
        devices_[0].name = "[Camera 1]: ";
        devices_[1].name = "[Camera 2]: ";
        devices_[2].name = "[Camera 3]: ";
        devices_[3].name = "[Camera 4]: ";
        devices_[4].name = "[Camera 5]: ";

        devices_[5].name = "[Lidar 1]: ";
        devices_[6].name = "[Lidar 2]: ";
        devices_[7].name = "[Lidar 3]: ";
        devices_[8].name = "[Lidar 4]: ";
        devices_[9].name = "[Apx]: " ;


//        lidar1_sub = nh_.subscribe(cfg_.lidar1_topic, DeviceMonitor::)
        cam1_sub = nh_.subscribe(cfg_.camera1_topic, 10,&DeviceMonitor::Camera1Callback,this);
        cam2_sub = nh_.subscribe(cfg_.camera2_topic, 10,&DeviceMonitor::Camera2Callback,this);
        cam3_sub = nh_.subscribe(cfg_.camera3_topic, 10,&DeviceMonitor::Camera3Callback,this);
        cam4_sub = nh_.subscribe(cfg_.camera4_topic, 10,&DeviceMonitor::Camera5Callback,this);
        cam5_sub = nh_.subscribe(cfg_.camera5_topic, 10,&DeviceMonitor::Camera5Callback,this);




        ros::Duration(1.0).sleep();
        print_timer_ = nh_.createTimer(ros::Duration(1.0/cfg_.print_hz),&DeviceMonitor::PrintTimerCallback,this);

    }
    ~DeviceMonitor(){}

private:
    Table monitor_;

    bool IsTimeout(ros::Time t, double dt = 5.0){
        if((ros::Time::now() - t).toSec() > dt){
            return true;
        }else{
            return false;
        }
    }

    string ConverDoubleToString(const long double value, const int precision = 0){
        std::stringstream stream{};
        stream<<std::fixed<<std::setprecision(precision)<<value;
        return stream.str();
    }

    void PrintTimerCallback(const ros::TimerEvent & e){
//        clear();
//        std::cout << monitor_ << std::endl;
        Table monitor_;
        // Init table
        monitor_.add_row({"Livox Data Collector",""});
        monitor_.add_row({"camera1", "apx"});
        monitor_.add_row({"camera2", "lidar1"});
        monitor_.add_row({"camera3", "lidar2"});
        monitor_.add_row({"camera4", "lidar3"});
        monitor_.add_row({"camera5", "lidar4"});

        clear();
        // camera
        double max_t = -DBL_MAX, min_t = DBL_MAX;
        for(int i = 0 ; i < 10 ; i++){
            int row,cell;
            if(i <5){
                row = i+1;
                cell = 0;
            }else{
                row = i-4;
                cell = 1;
            }
            string tex = devices_[i].name;
            if(devices_[i].last_call_back.toSec() > max_t){
                max_t = devices_[i].last_call_back.toSec();
            }
            if(devices_[i].last_call_back.toSec() < min_t){
                min_t = devices_[i].last_call_back.toSec();
            }
            if(IsTimeout(devices_[i].last_call_back, 5.0)){
                tex+=" NC";
                monitor_.row(row).cell(cell).set_text(tex);
                monitor_.row(row).cell(cell).format()
                        .font_background_color(Color::red)
                        .font_color(Color::white)  .font_align(FontAlign::center)
                        .border_top(" ")
                        .border_bottom(" ")
                        .border_left(" ")
                        .border_right(" ")
                        .corner(" ");
            }else{
                ComputeAverageTime(devices_[i]);
                tex+= ConverDoubleToString(devices_[i].hz,3);
                tex+= " Hz";
                monitor_.row(row).cell(cell).set_text(tex);
                if(abs(devices_[i].hz - cfg_.desired_hz) < 0.8){
                    monitor_.row(row).cell(cell).format()
                            .font_background_color(Color::green)
                            .font_color(Color::white)  .font_align(FontAlign::center)
                            .border_top(" ")
                            .border_bottom(" ")
                            .border_left(" ")
                            .border_right(" ")
                            .corner(" ");
                }else{
                    monitor_.row(row).cell(cell).format()
                            .font_background_color(Color::yellow)
                            .font_color(Color::white)  .font_align(FontAlign::center)
                            .border_top(" ")
                            .border_bottom(" ")
                            .border_left(" ")
                            .border_right(" ")
                            .corner(" ");
                }

            }
        }
        double dt = max_t - min_t;
        string dt_s = ConverDoubleToString(dt,5);
        monitor_.row(0).cell(1).set_text(dt_s);
        monitor_.row(0).cell(0).format().border_top(" ")
                .border_top(" ")
                .border_bottom(" ")
                .border_left(" ")
                .border_right(" ")
                .corner(" ").font_background_color(Color::blue).font_color(Color::white);
        if(dt>0.5){
            monitor_.row(0).cell(1).format().border_top(" ")
                    .border_top(" ")
                    .border_bottom(" ")
                    .border_left(" ")
                    .border_right(" ")
                    .corner(" ").font_background_color(Color::red).font_color(Color::white);;
        }else{
            monitor_.row(0).cell(1).format().border_top(" ")
                    .border_top(" ")
                    .border_bottom(" ")
                    .border_left(" ")
                    .border_right(" ")
                    .corner(" ").font_background_color(Color::green).font_color(Color::white);;
        }


        std::cout << monitor_ << std::endl;
        sleep(1);
    }

    void UpdateDeviceState(const int device_id,const ros::Time stamp){
        devices_[device_id].last_call_back = stamp;
        static int cnt = 0;
        if(cnt++>=cfg_.sample_num){
            cnt = 0;
        }
        static ros::Time last_t = ros::Time::now();
        double dt = (stamp - last_t).toSec();
        last_t =stamp;
        devices_[device_id].dt_queue[cnt] = dt;
    }

    void Camera1Callback(const sensor_msgs::ImageConstPtr & msg){
        UpdateDeviceState(0,msg->header.stamp);
    }
    void Camera2Callback(const sensor_msgs::ImageConstPtr & msg){
        UpdateDeviceState(1,msg->header.stamp);
    }
    void Camera3Callback(const sensor_msgs::ImageConstPtr & msg){
        UpdateDeviceState(2,msg->header.stamp);
    }
    void Camera4Callback(const sensor_msgs::ImageConstPtr & msg){
        UpdateDeviceState(3,msg->header.stamp);
    }
    void Camera5Callback(const sensor_msgs::ImageConstPtr & msg){
        UpdateDeviceState(4,msg->header.stamp);
    }


    void clear() {
        // CSI[2J clears screen, CSI[H moves the cursor to top-left corner
        std::cout << "\x1B[2J\x1B[H";
    }

    ros::NodeHandle nh_;
    ros::Timer print_timer_;
    ros::Subscriber cam1_sub, cam2_sub, cam3_sub, cam4_sub, cam5_sub;
    ros::Subscriber lidar1_sub, lidar2_sub, lidar3_sub,lidar4_sub;


    // 0-4 are camera 1-5
    // 5-9 are lidar 1-4
    // 10 is Apx
    DeviceState devices_[10];


    struct Config{
        string camera1_topic,camera2_topic,camera3_topic,camera4_topic,camera5_topic;
        string lidar1_topic,lidar2_topic,lidar3_topic,lidar4_topic;
        string apx_topic;

        double desired_hz;
        double print_hz;

        int sample_num{20};
    }cfg_;







};


#endif //SRC_KRRT_PLANNER_H
