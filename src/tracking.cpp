#include "ros/ros.h"
#include "sawyer_move/YoyoState.h"
#include "sawyer_move/RobotState.h"
#include "sawyer_move/RobotControl.h"
#include "std_msgs/Int8.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime> 
#include "camera.hpp"
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
using namespace cam;
using namespace std;
using namespace cv;

class MovingAverage
{
    private:
        int n_past;
        double cap_val;
        std::string name;
        std::deque<float> buffer;

    public:
        MovingAverage(const std::string name, const int n_past, double init_val, double cap_val) : 
        n_past(n_past),
        name(name),
        cap_val(cap_val)
        {
            buffer.push_back(init_val);
        }

        void add(float new_data){
            if (new_data > cap_val){
                new_data = get_last();
            }

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

        double ee_z_pos;
        double robot_origin_from_ground;

        double robot_origin_z_eo;
        double robot_origin_z_flir;
        MovingAverage last_yoyo_z_dis;
        MovingAverage last_yoyo_rot;
        std::chrono::_V2::system_clock::time_point last_tracking_time;

        int width;
        int height;
        cam::Camera camera;

        apriltag_family_t *tf_eo;
        apriltag_detector_t *td_eo;
        apriltag_family_t *tf_flir;
        apriltag_detector_t *td_flir;


    public:
        Tracking(ros::NodeHandle nh, int width, int height, double robot_origin_from_ground, 
        double eo_height, double flir_height, double eo_dis, double flir_dis, double robot_origin_z_eo, double robot_origin_z_flir) : 
        timer(nh.createTimer(ros::Duration((1.0/200.0)), &Tracking::main_loop, this)),
        width(width),
        height(height),
        camera(cam::Camera(width, height, eo_dis, flir_dis, eo_height, flir_height)),
        yoyo_state_pub(nh.advertise<sawyer_move::YoyoState>("yoyo_state", 1000, true)),
        robot_state_sub(nh.subscribe("robot_state", 1000, &Tracking::robot_state_callback, this)),
        last_yoyo_z_dis(MovingAverage("z_dis", 10, 1.00, 1.0)),
        last_yoyo_rot(MovingAverage("rot", 10, 1.00*0.0055, 100.0)),
        last_tracking_time(std::chrono::system_clock::now()),
        ee_z_pos(1.55),
        robot_origin_from_ground(robot_origin_from_ground),
        robot_origin_z_eo(robot_origin_z_eo),
        robot_origin_z_flir(robot_origin_z_flir)
        {
            tf_eo = tag25h9_create();
            td_eo = apriltag_detector_create();
            apriltag_detector_add_family(td_eo,tf_eo);
            td_eo->nthreads = 10;
            td_eo->quad_decimate = 1.0;//2.0;
            // td_eo->refine_edges = 1;
            // td_eo->decode_sharpening = 1.0;

            tf_flir = tag25h9_create();
            td_flir = apriltag_detector_create();
            apriltag_detector_add_family(td_flir,tf_flir);
            td_flir->nthreads = 10;
            td_flir->quad_decimate = 1.0;//1.5;
            // td_flir->refine_edges = 1;
            // td_flir->decode_sharpening = 1.0;
        }

        void robot_state_callback(const sawyer_move::RobotState & state_data){
            ee_z_pos = state_data.ee_z_pos;
        }

        /// \brief The main control loop state machine
        void main_loop(const ros::TimerEvent &){
            //auto start = std::chrono::system_clock::now();
            double yoyo_z_dis;


            cv::Mat eo_frame(height, width, CV_8UC1);
            camera.eo.getNextFrame(eo_frame);


            cv::Mat flir_frame(height, width, CV_8UC1);
            camera.flir.getNextFrame(flir_frame);

            image_u8_t img_header = {
                .width = eo_frame.cols,
                .height = eo_frame.rows,
                .stride = eo_frame.cols,
                .buf = eo_frame.data
            };

            zarray_t * detections = apriltag_detector_detect(td_eo, &img_header);
            cv::Mat eo_Kinv;
            camera.eo.getKinv(eo_Kinv);
            if (zarray_size(detections) > 0){
                        // Draw detection outlines
                for (int i = 0; i < zarray_size(detections); i++) {
                    apriltag_detection_t *det;
                    zarray_get(detections, i, &det);

                    Mat worldCord = (Mat_<double>(3,1) << det->c[0], det->c[1], 1.0);
                    worldCord.convertTo(worldCord, CV_64FC1);
                    worldCord = eo_Kinv*worldCord;
                    worldCord *= camera.eo.getDistance(); //1.0414
                    yoyo_z_dis = camera.eo.getGroundHeight() - worldCord.at<double>(0,1);
                    cout << "eo " << yoyo_z_dis << " based on " << det->c[0] << " " << det->c[1] << endl;
                }
                zarray_destroy(detections);
                // imshow("Tag Detections", eo_frame);
                // int key = (cv::waitKey(1) & 0xFF);
            }else{
                image_u8_t flir_img_header = {
                    .width = flir_frame.cols,
                    .height = flir_frame.rows,
                    .stride = flir_frame.cols,
                    .buf = flir_frame.data
                };
                zarray_t * flir_detections = apriltag_detector_detect(td_flir, &flir_img_header);
                        // Draw detection outlines
                cv::Mat flir_Kinv;
                camera.flir.getKinv(flir_Kinv);
                for (int i = 0; i < zarray_size(flir_detections); i++) {
                    apriltag_detection_t *det;
                    zarray_get(flir_detections, i, &det);

                    Mat worldCord = (Mat_<double>(3,1) << det->c[0], det->c[1], 1.0);
                    worldCord.convertTo(worldCord, CV_64FC1);
                    worldCord = flir_Kinv*worldCord;
                    worldCord *= camera.flir.getDistance(); //1.0414
                    yoyo_z_dis = camera.flir.getGroundHeight() - worldCord.at<double>(0,1);
                    cout << "flir "<<yoyo_z_dis << " based on " << det->c[0] << " " << det->c[1] << endl;

                }
                zarray_destroy(flir_detections);
                // imshow("Tag Detections", flir_frame);
                // int key = (cv::waitKey(1) & 0xFF);
            }

            


            // //yoyo dis from ground
            // double yoyo_z_dis = robot_origin_from_ground - (cent.y * dis_per_pixel) + (robot_origin_z*dis_per_pixel);
            

            
            double last_yoyo_z_dis_avg = last_yoyo_z_dis.avg();
            double last_yoyo_rot_avg = last_yoyo_rot.avg();

            last_yoyo_z_dis.add(yoyo_z_dis);

            double yoyo_rot = (last_yoyo_z_dis.avg() - ee_z_pos + 1.0)/0.0055;
            last_yoyo_rot.add(yoyo_rot);

            auto curr_time = std::chrono::system_clock::now();
            double elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(curr_time-last_tracking_time).count()/1000.0;

            double yoyo_z_vel = (last_yoyo_z_dis.avg() - last_yoyo_z_dis_avg)/elapsed_seconds;

            double yoyo_rot_vel = (last_yoyo_rot.avg() - last_yoyo_rot_avg)/elapsed_seconds;


            sawyer_move::YoyoState yoyo_state;
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


    int width;
    int height;
    double robot_origin_from_ground; //in meter
    double eo_height;
    double flir_height;
    double eo_dis;
    double flir_dis;
    double robot_origin_z_eo; //robot origin pixel within the camera
    double robot_origin_z_flir;

    loadROSParam<int>(n, "width", width);
    loadROSParam<int>(n, "height", height);
    loadROSParam<double>(n, "robot_origin_from_ground", robot_origin_from_ground);
    loadROSParam<double>(n, "eo_height", eo_height);
    loadROSParam<double>(n, "flir_height", flir_height);
    loadROSParam<double>(n, "eo_dis", eo_dis);
    loadROSParam<double>(n, "flir_dis", flir_dis);
    loadROSParam<double>(n, "robot_origin_z_eo", robot_origin_z_eo);
    loadROSParam<double>(n, "robot_origin_z_flir", robot_origin_z_flir);

    Tracking tracking = Tracking(n, width, height, robot_origin_from_ground, 
                                        eo_height, flir_height, eo_dis, flir_dis,
                                        robot_origin_z_eo, robot_origin_z_flir);
    ros::spin();


    return 0;

}