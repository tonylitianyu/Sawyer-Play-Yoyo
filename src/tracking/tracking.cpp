#include "ros/ros.h"
#include "sawyer_move/YoyoState.h"
#include "sawyer_move/RobotState.h"
#include "sawyer_move/RobotControl.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime> 
#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <ctype.h>
#include <unistd.h>
#include <math.h>

#include "apriltag.h"
#include "tag25h9.h"


#include "common/getopt.h"
#include "common/image_u8.h"
#include "common/image_u8x4.h"
#include "common/pjpeg.h"
#include "common/zarray.h"

using namespace Eigen;
using namespace std;
using namespace cv;

class MovingAverage
{
    private:
        int n_past;
        double cap_val;
        double cap_diff_val;
        std::string name;
        std::deque<float> buffer;

    public:
        MovingAverage(const std::string name, const int n_past, double init_val, double cap_val, double cap_diff_val) : 
        n_past(n_past),
        name(name),
        cap_val(cap_val),
        cap_diff_val(cap_diff_val)
        {
            buffer.push_back(init_val);
        }

        void add(float new_data){
            // if (abs(new_data) > cap_val){
            //     new_data = get_last();
            // }else if (abs(new_data - get_last()) > cap_diff_val){
            //     new_data = get_last();
            // }

            if (buffer.size() < n_past){
                buffer.push_back(new_data);
            }else{
                buffer.pop_front();
                buffer.push_back(new_data);
            }
        }

        float avg(){
            float sum = 0.0;
            for (int i = 0; i < buffer.size(); i++){
                sum += buffer[i];
            }

            return sum/buffer.size();
        }
    
        float get_last(){
            return buffer[buffer.size() - 1];
        }

};

class Tracking
{
    private:
        ros::Timer timer;
        ros::Publisher yoyo_state_pub;
        ros::Subscriber robot_state_sub; //not started yet
        ros::Subscriber eo_measure_sub;
        ros::Subscriber flir_measure_sub;

        double ee_z_pos;
        double robot_origin_from_ground;


        MovingAverage last_yoyo_z_dis;
        MovingAverage last_yoyo_rot;
        std::chrono::_V2::system_clock::time_point last_tracking_time;


        double eo_yoyo_z_dis;
        double flir_yoyo_z_dis;

    public:
        Tracking(ros::NodeHandle nh, double robot_origin_from_ground) : 
        timer(nh.createTimer(ros::Duration((1.0/100.0)), &Tracking::main_loop, this)),
        yoyo_state_pub(nh.advertise<sawyer_move::YoyoState>("yoyo_state", 1000, true)),
        robot_state_sub(nh.subscribe("robot_state", 1000, &Tracking::robot_state_callback, this)),
        eo_measure_sub(nh.subscribe("eo_measure", 1000, &Tracking::eo_callback, this)),
        flir_measure_sub(nh.subscribe("flir_measure", 1000, &Tracking::flir_callback, this)),
        last_yoyo_z_dis(MovingAverage("z_dis", 10, 1.55, 1.0, 0.1)),
        last_yoyo_rot(MovingAverage("rot", 10, 1.00*0.0055, 150.0, 150.0)),
        last_tracking_time(std::chrono::system_clock::now()),
        ee_z_pos(1.55),
        robot_origin_from_ground(robot_origin_from_ground),
        eo_yoyo_z_dis(0.0),
        flir_yoyo_z_dis(0.0)
        {
        }

        void eo_callback(const std_msgs::Float64 & data){
            eo_yoyo_z_dis = data.data;
        }
        void flir_callback(const std_msgs::Float64 & data){
            flir_yoyo_z_dis = data.data;
        }

        void robot_state_callback(const sawyer_move::RobotState & state_data){
            ee_z_pos = state_data.ee_z_pos;
        }

        /// \brief The main control loop state machine
        void main_loop(const ros::TimerEvent &){
            //auto start = std::chrono::system_clock::now();
            double yoyo_z_dis;
            if (eo_yoyo_z_dis > 0.0){
                yoyo_z_dis = eo_yoyo_z_dis;
            }else{
                yoyo_z_dis = flir_yoyo_z_dis;
            }

            if ((eo_yoyo_z_dis < 0.0) && (flir_yoyo_z_dis < 0.0)){
                cout << "no tag detected from both cameras !!!" << endl;
                return;
            }

        
            // //yoyo dis from ground
            // double yoyo_z_dis = robot_origin_from_ground - (cent.y * dis_per_pixel) + (robot_origin_z*dis_per_pixel);
            

            
            double last_yoyo_z_dis_avg = last_yoyo_z_dis.avg();
            double last_yoyo_rot_avg = last_yoyo_rot.avg();
            

            if(abs(yoyo_z_dis - last_yoyo_z_dis.get_last()) > 0.2){
                yoyo_z_dis = last_yoyo_z_dis.get_last();
            }

            last_yoyo_z_dis.add(yoyo_z_dis);

            double yoyo_rot = (last_yoyo_z_dis.avg() - ee_z_pos + 1.0)/0.0055;

            last_yoyo_rot.add(yoyo_rot);

            auto curr_time = std::chrono::system_clock::now();
            double elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(curr_time-last_tracking_time).count()/1000.0;

            double yoyo_z_vel = (last_yoyo_z_dis.avg() - last_yoyo_z_dis_avg)/elapsed_seconds;

            double yoyo_rot_vel = (last_yoyo_rot.avg() - last_yoyo_rot_avg)/elapsed_seconds;



            sawyer_move::YoyoState yoyo_state;
            //cout << yoyo_z_dis << endl;
            yoyo_state.yoyo_pos = yoyo_z_dis;
            yoyo_state.yoyo_posvel = yoyo_z_vel;
            yoyo_state.yoyo_rot = yoyo_rot;
            yoyo_state.yoyo_rotvel = yoyo_rot_vel;
            yoyo_state_pub.publish(yoyo_state);


            last_tracking_time = curr_time;
            


            // auto end = std::chrono::system_clock::now();

            // std::chrono::duration<double> elapsed_seconds = end-start;
            // std::time_t end_time = std::chrono::system_clock::to_time_t(end);

            // std::cout << "finished computation at " << std::ctime(&end_time)
            //         << "elapsed time: " << 1/(elapsed_seconds.count()) << "hz\n";
        }

};


template <typename T>
void loadROSParam(ros::NodeHandle nh, string name, T&param){
    if (nh.getParam(name, param)){
        ROS_INFO("%s: %s", name.c_str(), to_string(param).c_str());
    }else{
        ROS_ERROR("Unable to get param '%s'", name.c_str());
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracking");
    ros::NodeHandle n;


    double robot_origin_from_ground; //in meter

    loadROSParam<double>(n, "robot_origin_from_ground", robot_origin_from_ground);


    Tracking tracking = Tracking(n, robot_origin_from_ground);
    ros::spin();


    return 0;

}