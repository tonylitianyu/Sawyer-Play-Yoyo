#include "ros/ros.h"
#include "sawyer_move/YoyoState.h"
#include "sawyer_move/RobotState.h"
#include "sawyer_move/RobotControl.h"
#include "std_msgs/Int8.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <fstream>
#include <aruco/aruco.h>
#include "camera.hpp"


using namespace Eigen;
using namespace cam;

class MovingAverage
{
    private:
        int n_past;
        std::string name;
        std::deque<float> buffer;

    public:
        MovingAverage(const int n_past, const std::string name) : 
        n_past(n_past),
        name(name)
        {
            buffer.push_back(0.0);
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
        ros::Subscriber robot_state_sub;



        int width;
        int height;
        cam::Camera camera;
        aruco::MarkerDetector MDetector;
    public:
        Tracking(ros::NodeHandle nh, int width, int height) : 
        timer(nh.createTimer(ros::Duration((1.0/100.0)), &Tracking::main_loop, this)),
        width(width),
        height(height),
        camera(cam::Camera(width, height)),
        yoyo_state_pub(nh.advertise<sawyer_move::YoyoState>("yoyo_state", 1000, true))
        {
            MDetector.setDictionary("ARUCO_MIP_16h3");
            MDetector.setDetectionMode(aruco::DM_FAST, 0.02);
        }

        /// \brief The main control loop state machine
        void main_loop(const ros::TimerEvent &){
            cv::Mat frame(height, width, CV_8UC1);
            camera.getNextFrame(frame);

            vector<aruco::Marker> markers = MDetector.detect(frame);
            if (markers.size() == 0){
                camera.switchCam();
            }

            for(size_t i=0;i<markers.size();i++){
                markers[i].draw(frame);
                Point2f cent = markers[i].getCenter();
                cout << cent << endl;
            }

        }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracking");
    ros::NodeHandle n;

    int width = 720;
    int height = 300;

    Tracking tracking = Tracking(n, width, height);
    ros::spin();


    return 0;

}