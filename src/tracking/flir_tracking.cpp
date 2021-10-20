#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime> 
#include "flir.hpp"
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
using namespace FLIR;

class Flir_Tracking{
    private:
        ros::Timer timer;
        ros::Publisher flir_measure_pub;
        Flir camera;

        apriltag_family_t *tf_flir;
        apriltag_detector_t *td_flir;

    public:
        Flir_Tracking(ros::NodeHandle nh, Flir &camera) : 
        timer(nh.createTimer(ros::Duration((1.0/500.0)), &Flir_Tracking::main_loop, this)),
        flir_measure_pub(nh.advertise<std_msgs::Float64>("flir_measure", 1000, true)),
        camera(camera)
        {
            tf_flir = tag25h9_create();
            td_flir = apriltag_detector_create();
            apriltag_detector_add_family(td_flir,tf_flir);
            td_flir->nthreads = 10;
            td_flir->quad_decimate = 1.0;
        }


        /// \brief The main control loop state machine
        void main_loop(const ros::TimerEvent &){
            auto start = std::chrono::system_clock::now();
            cv::Mat flir_frame(camera.getHeight(), camera.getWidth(), CV_8UC1);
            camera.getNextFrame(flir_frame);

            image_u8_t img_header = {
                .width = flir_frame.cols,
                .height = flir_frame.rows,
                .stride = flir_frame.cols,
                .buf = flir_frame.data
            };

            zarray_t * detections = apriltag_detector_detect(td_flir, &img_header);
            cv::Mat flir_Kinv;
            camera.getKinv(flir_Kinv);

            double yoyo_z_dis;

            if (zarray_size(detections) == 0){
                yoyo_z_dis = -1.0;
            }

            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);
                line(flir_frame, Point(det->p[0][0], det->p[0][1]),
                        Point(det->p[1][0], det->p[1][1]),
                        Scalar(0, 0xff, 0), 2);
                line(flir_frame, Point(det->p[0][0], det->p[0][1]),
                        Point(det->p[3][0], det->p[3][1]),
                        Scalar(0, 0, 0xff), 2);
                line(flir_frame, Point(det->p[1][0], det->p[1][1]),
                        Point(det->p[2][0], det->p[2][1]),
                        Scalar(0xff, 0, 0), 2);
                line(flir_frame, Point(det->p[2][0], det->p[2][1]),
                        Point(det->p[3][0], det->p[3][1]),
                        Scalar(0xff, 0, 0), 2);

                stringstream ss;
                ss << det->id;
                String text = ss.str();
                int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
                double fontscale = 1.0;
                int baseline;
                Size textsize = getTextSize(text, fontface, fontscale, 2,
                                                &baseline);
                putText(flir_frame, text, Point(det->c[0]-textsize.width/2,
                                        det->c[1]+textsize.height/2),
                        fontface, fontscale, Scalar(0xff, 0x99, 0), 2);

                Mat worldCord = (Mat_<double>(3,1) << det->c[0], det->c[1], 1.0);
                worldCord.convertTo(worldCord, CV_64FC1);
                worldCord = flir_Kinv*worldCord;
                worldCord *= camera.getDistance(); //1.0414
                yoyo_z_dis = camera.getGroundHeight() - worldCord.at<double>(0,1);
                //cout << "flir " << yoyo_z_dis << " based on " << det->c[0] << " " << det->c[1] << endl;
            }

            // auto end = std::chrono::system_clock::now();

            // std::chrono::duration<double> elapsed_seconds = end-start;
            // std::time_t end_time = std::chrono::system_clock::to_time_t(end);

            // std::cout << "finished computation at " << std::ctime(&end_time)
            //         << "elapsed time: " << (elapsed_seconds.count()) << "s\n";


            zarray_destroy(detections);
            // imshow("Flir Tag Detections", flir_frame);
            // int key = (cv::waitKey(1) & 0xFF);

            std_msgs::Float64 z_dis_msg = std_msgs::Float64();
            z_dis_msg.data = yoyo_z_dis;
            flir_measure_pub.publish(z_dis_msg);
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
    ros::init(argc, argv, "flirtracking");
    ros::NodeHandle n;


    int width;
    int height;
    double flir_height;
    double flir_dis;


    loadROSParam<int>(n, "width", width);
    loadROSParam<int>(n, "height", height);
    loadROSParam<double>(n, "flir_height", flir_height);
    loadROSParam<double>(n, "flir_dis", flir_dis);

    
    Flir camera = Flir(0, width, height, flir_dis, flir_height);
    Flir_Tracking tracking = Flir_Tracking(n, camera);
    ros::spin();


    return 0;

}