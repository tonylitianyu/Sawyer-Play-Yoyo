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
#include <aruco/aruco.h>
#include "camera.hpp"
#include <opencv2/opencv.hpp>

using namespace Eigen;
using namespace cam;
using namespace std;
using namespace cv;

class MovingAverage
{
    private:
        int n_past;
        std::string name;
        std::deque<float> buffer;

    public:
        MovingAverage(const std::string name, const int n_past, double init_val) : 
        n_past(n_past),
        name(name)
        {
            buffer.push_back(init_val);
        }

        void add(float new_data){
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
        aruco::MarkerDetector MDetector;
    public:
        Tracking(ros::NodeHandle nh, int width, int height, double robot_origin_from_ground, 
        double eo_height, double flir_height, double eo_dis, double flir_dis, double robot_origin_z_eo, double robot_origin_z_flir) : 
        timer(nh.createTimer(ros::Duration((1.0/200.0)), &Tracking::main_loop, this)),
        width(width),
        height(height),
        camera(cam::Camera(width, height, eo_dis, flir_dis, eo_height, flir_height)),
        yoyo_state_pub(nh.advertise<sawyer_move::YoyoState>("yoyo_state", 1000, true)),
        robot_state_sub(nh.subscribe("robot_state", 1000, &Tracking::robot_state_callback, this)),
        last_yoyo_z_dis(MovingAverage("z_dis", 10, 1.00)),
        last_yoyo_rot(MovingAverage("rot", 10, 1.00*0.0055)),
        last_tracking_time(std::chrono::system_clock::now()),
        ee_z_pos(1.55),
        robot_origin_from_ground(robot_origin_from_ground),
        robot_origin_z_eo(robot_origin_z_eo),
        robot_origin_z_flir(robot_origin_z_flir)
        {
            MDetector.setDictionary("ARUCO_MIP_16h3");
            MDetector.setDetectionMode(aruco::DM_FAST, 0.02);
        }

        void robot_state_callback(const sawyer_move::RobotState & state_data){
            ee_z_pos = state_data.ee_z_pos;

        }

        /// \brief The main control loop state machine
        void main_loop(const ros::TimerEvent &){
            // auto start = std::chrono::system_clock::now();
            cv::Mat eo_frame(height, width, CV_8UC1);
            camera.eo.getNextFrame(eo_frame);


            cv::Mat flir_frame(height, width, CV_8UC1);
            camera.flir.getNextFrame(flir_frame);


            vector<aruco::Marker> eo_markers = MDetector.detect(eo_frame);
            vector<aruco::Marker> flir_markers = MDetector.detect(flir_frame);

            if ((eo_markers.size() == 0) && (flir_markers.size() == 0)){
                cout << "i'm blind" << endl;
                return;
            }
            cv::Mat frame(height, width, CV_8UC1);
            cv::Mat Kinv;
            double yoyo_z_dis;
            if (eo_markers.size() >= flir_markers.size()){
                frame = eo_frame;
                camera.eo.getKinv(Kinv);
                Point2f cent;
                //double yoyo_z_dis;
                for(size_t i=0;i<eo_markers.size();i++){
                    eo_markers[i].draw(frame);
                    Point2f cent = eo_markers[i].getCenter();
                    //cout << cent << endl;

                    Mat worldCord = (Mat_<double>(3,1) << cent.x, cent.y, 1.0);
                    worldCord.convertTo(worldCord, CV_64FC1);
                    worldCord = Kinv*worldCord;
                    worldCord *= camera.eo.getDistance(); //1.0414
                    yoyo_z_dis = camera.eo.getGroundHeight() - worldCord.at<double>(0,1);
                    cout << yoyo_z_dis << endl;
                    //cout << worldCord.at<double>(0,1) << endl;
                }
            }else{
                frame = flir_frame;
                camera.flir.getKinv(Kinv);
                Point2f cent;
                //double yoyo_z_dis;
                for(size_t i=0;i<flir_markers.size();i++){
                    flir_markers[i].draw(frame);
                    Point2f cent = flir_markers[i].getCenter();
                    //cout << cent << endl;

                    Mat worldCord = (Mat_<double>(3,1) << cent.x, cent.y, 1.0);
                    worldCord.convertTo(worldCord, CV_64FC1);
                    worldCord = Kinv*worldCord;
                    worldCord *= camera.flir.getDistance(); //1.0414
                    yoyo_z_dis = camera.flir.getGroundHeight() - worldCord.at<double>(0,1);
                    cout << yoyo_z_dis << endl;
                    //cout << worldCord.at<double>(0,1) << endl;
                }
            }
            



            //string currCamName = camera.getCurrCamName();
            
            //cout << currCamName << endl;
            // double dis_per_pixel;
            // double robot_origin_z;
            // if (currCamName == "eo"){
            //     dis_per_pixel = dis_per_pixel_eo;
            //     robot_origin_z = robot_origin_z_eo;
            // }else{
            //     dis_per_pixel = dis_per_pixel_flir;
            //     robot_origin_z = robot_origin_z_flir;
            // }


            // //yoyo dis from ground
            // double yoyo_z_dis = robot_origin_from_ground - (cent.y * dis_per_pixel) + (robot_origin_z*dis_per_pixel);
            double yoyo_rot = (yoyo_z_dis - ee_z_pos + 1.0)/0.0055;

            
            double last_yoyo_z_dis_avg = last_yoyo_z_dis.avg();
            double last_yoyo_rot_avg = last_yoyo_rot.avg();

            last_yoyo_z_dis.add(yoyo_z_dis);
            last_yoyo_rot.add(yoyo_rot);

            auto curr_time = std::chrono::system_clock::now();
            double elapsed_seconds = (curr_time-last_tracking_time).count();

            double yoyo_z_vel = (last_yoyo_z_dis.avg() - last_yoyo_z_dis_avg)/elapsed_seconds;
            double yoyo_rot_vel = (last_yoyo_rot.avg() - last_yoyo_rot_avg)/elapsed_seconds;


            sawyer_move::YoyoState yoyo_state;
            yoyo_state.yoyo_pos = yoyo_z_dis;
            yoyo_state.yoyo_posvel = yoyo_z_vel;
            yoyo_state.yoyo_rot = yoyo_rot;
            yoyo_state.yoyo_rotvel = yoyo_rot_vel;
            yoyo_state_pub.publish(yoyo_state);


            auto last_tracking_time = curr_time;
            

            cv::namedWindow("current Image", cv::WINDOW_AUTOSIZE);
            cv::imshow("current Image", frame);
            int key = (cv::waitKey(1) & 0xFF);//otherwise the image will not display...
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