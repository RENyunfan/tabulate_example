//
// Created by yunfan on 2022/3/22.
//

#ifndef INCLUDE_DEVICE_MONITOR_HPP
#define INCLUDE_DEVICE_MONITOR_HPP

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "livox_ros_driver/CustomMsg.h"
 #include "tabulate/table.hpp"
#include "iostream"
#include "string"
 using namespace tabulate;
using namespace std;

#define USE_TABULATE

class DeviceMonitor
{
private:
    struct DeviceState
    {
        ros::Time last_callback{0}, current_callback;
        double hz, desired_hz;
        double *dt_queue;
        string name;
    };

    template <class T>
    bool LoadParam(string param_name, T &param_value, T default_value)
    {
        if (nh_.getParam(param_name, param_value))
        {
            printf("\033[0;32m Load param %s success: \033[0;0m", param_name.c_str());
            cout << param_value << endl;
            return true;
        }
        else
        {
            printf("\033[0;33m Load param %s failed, use default value: \033[0;0m", param_name.c_str());
            param_value = default_value;
            cout << param_value << endl;
            return false;
        }
    }

    template <class T>
    bool LoadParam(string param_name, vector<T> &param_value, vector<T> default_value)
    {
        if (nh_.getParam(param_name, param_value))
        {
            printf("\033[0;32m Load param %s success: \033[0;0m", param_name.c_str());
            for (int i = 0; i < param_value.size(); i++)
            {
                cout << param_value[i] << " ";
            }
            cout << endl;
            return true;
        }
        else
        {
            printf("\033[0;33m Load param %s failed, use default value: \033[0;0m", param_name.c_str());
            param_value = default_value;
            for (int i = 0; i < param_value.size(); i++)
            {
                cout << param_value[i] << " ";
            }
            cout << endl;
            return false;
        }
    }

    void ComputeAverageTime(DeviceState &ds)
    {
        double all = 0;
        for (int i = 0; i < cfg_.sample_num; i++)
        {
            all += ds.dt_queue[i];
        }

        ds.hz = 1.0 / (all / cfg_.sample_num);
    }

public:
    DeviceMonitor(const ros::NodeHandle &nh)
    {
        nh_ = nh;
        // Load params
        LoadParam("camera1_topic", cfg_.camera1_topic, string("cam1"));
        LoadParam("camera2_topic", cfg_.camera2_topic, string("cam2"));
        LoadParam("camera3_topic", cfg_.camera3_topic, string("cam3"));
        LoadParam("camera4_topic", cfg_.camera4_topic, string("cam4"));
        LoadParam("camera5_topic", cfg_.camera5_topic, string("cam5"));

        LoadParam("lidar1_topic", cfg_.lidar1_topic, string("lidar1"));
        LoadParam("lidar2_topic", cfg_.lidar2_topic, string("lidar2"));
        LoadParam("lidar3_topic", cfg_.lidar3_topic, string("lidar3"));
        LoadParam("lidar4_topic", cfg_.lidar4_topic, string("lidar4"));

        LoadParam("apx_imu_topic", cfg_.apx_imu_topic, string("apx"));
        LoadParam("apx_state_topic", cfg_.apx_state_topic, string("apx"));

        LoadParam("camera_desired_hz", cfg_.camera_desired_hz, 30.0);
        LoadParam("lidar_desired_hz", cfg_.lidar_desired_hz, 10.0);
        LoadParam("apx_desired_hz", cfg_.apx_desired_hz, 10.0);

        LoadParam("tolerance", cfg_.tolerance, 0.8);

        LoadParam("print_hz", cfg_.print_hz, 5.0);
        LoadParam("sample_num", cfg_.sample_num, 20);
        LoadParam("sync_timeout", cfg_.sync_timeout, 5.0);
        LoadParam("msg_timeout", cfg_.msg_timeout, 0.2);

        for (int i = 0; i < 11; i++)
        {
            devices_[i].dt_queue = new double[cfg_.sample_num];
        }

        devices_[0].name = "[Camera_B]: ";
        devices_[0].desired_hz = cfg_.camera_desired_hz;
        devices_[1].name = "[Camera_FL]: ";
        devices_[1].desired_hz = cfg_.camera_desired_hz;
        devices_[2].name = "[Camera_FR]: ";
        devices_[2].desired_hz = cfg_.camera_desired_hz;
        devices_[3].name = "[Camera_L]: ";
        devices_[3].desired_hz = cfg_.camera_desired_hz;
        devices_[4].name = "[Camera_R]: ";
        devices_[4].desired_hz = cfg_.camera_desired_hz;

        devices_[5].name = "[Lidar_R]: ";
        devices_[5].desired_hz = cfg_.lidar_desired_hz;
        devices_[6].name = "[Lidar_L]: ";
        devices_[6].desired_hz = cfg_.lidar_desired_hz;
        devices_[7].name = "[Lidar_B]: ";
        devices_[7].desired_hz = cfg_.lidar_desired_hz;
        devices_[8].name = "[Lidar_F]: ";
        devices_[8].desired_hz = cfg_.lidar_desired_hz;
        devices_[9].name = "[Apx_imu]: ";
        devices_[9].desired_hz = cfg_.apx_desired_hz;
        devices_[10].name = "[Apx_state]: ";
        devices_[10].desired_hz = cfg_.apx_desired_hz;

        //        lidar1_sub = nh_.subscribe(cfg_.lidar1_topic, DeviceMonitor::)
        cam1_sub = nh_.subscribe(cfg_.camera1_topic, 10, &DeviceMonitor::Camera1Callback, this);
        cam2_sub = nh_.subscribe(cfg_.camera2_topic, 10, &DeviceMonitor::Camera2Callback, this);
        cam3_sub = nh_.subscribe(cfg_.camera3_topic, 10, &DeviceMonitor::Camera3Callback, this);
        cam4_sub = nh_.subscribe(cfg_.camera4_topic, 10, &DeviceMonitor::Camera4Callback, this);
        cam5_sub = nh_.subscribe(cfg_.camera5_topic, 10, &DeviceMonitor::Camera5Callback, this);

        apx_imu_sub = nh_.subscribe(cfg_.apx_imu_topic, 10, &DeviceMonitor::ApxImuCallback, this);
        apx_state_sub = nh_.subscribe(cfg_.apx_state_topic, 10, &DeviceMonitor::ApxStateCallback, this);

        lidar1_sub = nh_.subscribe(cfg_.lidar1_topic, 10, &DeviceMonitor::Lidar1Callback, this);
        lidar2_sub = nh_.subscribe(cfg_.lidar2_topic, 10, &DeviceMonitor::Lidar2Callback, this);
        lidar3_sub = nh_.subscribe(cfg_.lidar3_topic, 10, &DeviceMonitor::Lidar3Callback, this);
        lidar4_sub = nh_.subscribe(cfg_.lidar4_topic, 10, &DeviceMonitor::Lidar4Callback, this);

        ros::Duration(1.0).sleep();
        print_timer_ = nh_.createTimer(ros::Duration(1.0 / cfg_.print_hz), &DeviceMonitor::PrintTimerCallback, this);
    }
    ~DeviceMonitor() {}

private:
    // Table monitor_;

    bool IsTimeout(const DeviceState & device, double dt = 5)
    {
        double c_dt = (device.current_callback- devices_->last_callback).toSec();
//        cout<<c_dt<<" "<< dt
//        <<endl;
        if (c_dt> dt)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    string ConverDoubleToString(const long double value, const int precision = 0)
    {
        std::stringstream stream{};
        stream << std::fixed << std::setprecision(precision) << value;
        return stream.str();
    }
#ifndef USE_TABULATE
  void PrintTimerCallback(const ros::TimerEvent &e)
    {
        double max_t = -DBL_MAX, min_t = DBL_MAX;
        for (int i = 0; i < 11; i++)
        {

            // if (IsTimeout(devices_[i].last_call_back, cfg_.msg_timeout))
            // {
            //     cout << devices_[i].name << " NC"<<endl;
            //     continue;
            // }
            ComputeAverageTime(devices_[i]);
            if (devices_[i].last_call_back.toSec() > max_t)
            {
                max_t = devices_[i].last_call_back.toSec();
            }
            if (devices_[i].last_call_back.toSec() < min_t)
            {
                min_t = devices_[i].last_call_back.toSec();
            }

            cout << devices_[i].name << " ";
            cout << devices_[i].hz << " " << endl;
        }
        cout << (max_t - min_t) << endl;
    }
#else
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
                 if(devices_[i].current_callback.toSec() > max_t){
                     max_t = devices_[i].current_callback.toSec();
                 }
                 if(devices_[i].current_callback.toSec() < min_t){
                     min_t = devices_[i].current_callback.toSec();
                 }
                 cout<<setprecision(12)<<devices_[i].current_callback.toSec()<<endl;
                 if(IsTimeout(devices_[i], cfg_.msg_timeout)){
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
                 }
                 else{
                     ComputeAverageTime(devices_[i]);
                     tex+= ConverDoubleToString(devices_[i].hz,3);
                     tex+= " Hz";
                     monitor_.row(row).cell(cell).set_text(tex);
                     if(abs(devices_[i].hz - devices_[i].desired_hz) < cfg_.tolerance){
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
             if(dt>cfg_.sync_timeout){
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
#endif
    void UpdateDeviceState(const int device_id, const ros::Time stamp)
    {
        devices_[device_id].current_callback = stamp;
        static int cnt = 0;
        if (cnt++ >= cfg_.sample_num)
        {
            cnt = 0;
        }
        double dt = (stamp -  devices_[device_id].last_callback ).toSec();
        devices_[device_id].last_callback = stamp;
        devices_[device_id].dt_queue[cnt] = dt;
    }

    void Camera1Callback(const sensor_msgs::ImageConstPtr &msg)
    {
        UpdateDeviceState(0, msg->header.stamp);
    }
    void Camera2Callback(const sensor_msgs::ImageConstPtr &msg)
    {
        UpdateDeviceState(1, msg->header.stamp);
    }
    void Camera3Callback(const sensor_msgs::ImageConstPtr &msg)
    {
        UpdateDeviceState(2, msg->header.stamp);
    }
    void Camera4Callback(const sensor_msgs::ImageConstPtr &msg)
    {
        UpdateDeviceState(3, msg->header.stamp);
    }
    void Camera5Callback(const sensor_msgs::ImageConstPtr &msg)
    {
        UpdateDeviceState(4, msg->header.stamp);
    }

    void Lidar1Callback(const livox_ros_driver::CustomMsg::ConstPtr &msg)
    {
        UpdateDeviceState(5, msg->header.stamp);
    }

    void Lidar2Callback(const livox_ros_driver::CustomMsg::ConstPtr &msg)
    {     UpdateDeviceState(6, msg->header.stamp);
    }

    void Lidar3Callback(const livox_ros_driver::CustomMsg::ConstPtr &msg)
    {     UpdateDeviceState(7, msg->header.stamp);
    }

    void Lidar4Callback(const livox_ros_driver::CustomMsg::ConstPtr &msg)
    {     UpdateDeviceState(8, msg->header.stamp);
    }

    void ApxImuCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {     UpdateDeviceState(9, msg->header.stamp);
    }

    void ApxStateCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {     UpdateDeviceState(10, msg->header.stamp);
    }

    void clear()
    {
        // CSI[2J clears screen, CSI[H moves the cursor to top-left corner
        std::cout << "\x1B[2J\x1B[H";
    }

    ros::NodeHandle nh_;
    ros::Timer print_timer_;
    ros::Subscriber cam1_sub, cam2_sub, cam3_sub, cam4_sub, cam5_sub;
    ros::Subscriber lidar1_sub, lidar2_sub, lidar3_sub, lidar4_sub;
    ros::Subscriber apx_imu_sub, apx_state_sub;
    // 0-4 are camera 1-5
    // 5-8 are lidar 1-4
    // 9 10 is Apx
    DeviceState devices_[11];

    struct Config
    {
        string camera1_topic, camera2_topic, camera3_topic, camera4_topic, camera5_topic;
        string lidar1_topic, lidar2_topic, lidar3_topic, lidar4_topic;
        string apx_imu_topic, apx_state_topic;

        double camera_desired_hz, lidar_desired_hz, apx_desired_hz;
        double print_hz;
        double sync_timeout, msg_timeout, tolerance;
        int sample_num{20};
    } cfg_;
};

#endif //SRC_KRRT_PLANNER_H
