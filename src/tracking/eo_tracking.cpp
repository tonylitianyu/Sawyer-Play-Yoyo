#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime> 
#include "eo.hpp"
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
using namespace UEYE;

class EO_Tracking{
    private:
        ros::Timer timer;
        ros::Publisher eo_measure_pub;
        EO camera;

        apriltag_family_t *tf_eo;
        apriltag_detector_t *td_eo;

    public:
        EO_Tracking(ros::NodeHandle nh, EO &camera) : 
        timer(nh.createTimer(ros::Duration((1.0/100.0)), &EO_Tracking::main_loop, this)),
        eo_measure_pub(nh.advertise<std_msgs::Float64>("eo_measure", 1000, true)),
        camera(camera)
        {
            tf_eo = tag25h9_create();
            td_eo = apriltag_detector_create();
            apriltag_detector_add_family(td_eo,tf_eo);
            td_eo->nthreads = 14;
            td_eo->quad_decimate = 1.0;
        }

        /// \brief The main control loop state machine
        void main_loop(const ros::TimerEvent &){
            cv::Mat eo_frame(camera.getHeight(), camera.getWidth(), CV_8UC1);
            camera.getNextFrame(eo_frame);

            image_u8_t img_header = {
                .width = eo_frame.cols,
                .height = eo_frame.rows,
                .stride = eo_frame.cols,
                .buf = eo_frame.data
            };

            zarray_t * detections = apriltag_detector_detect(td_eo, &img_header);
            cv::Mat eo_Kinv;
            camera.getKinv(eo_Kinv);

            double yoyo_z_dis;

            if (zarray_size(detections) == 0){
                yoyo_z_dis = -1.0;
            }

            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);
                // line(eo_frame, Point(det->p[0][0], det->p[0][1]),
                //         Point(det->p[1][0], det->p[1][1]),
                //         Scalar(0, 0xff, 0), 2);
                // line(eo_frame, Point(det->p[0][0], det->p[0][1]),
                //         Point(det->p[3][0], det->p[3][1]),
                //         Scalar(0, 0, 0xff), 2);
                // line(eo_frame, Point(det->p[1][0], det->p[1][1]),
                //         Point(det->p[2][0], det->p[2][1]),
                //         Scalar(0xff, 0, 0), 2);
                // line(eo_frame, Point(det->p[2][0], det->p[2][1]),
                //         Point(det->p[3][0], det->p[3][1]),
                //         Scalar(0xff, 0, 0), 2);

                Mat worldCord = (Mat_<double>(3,1) << det->c[0], det->c[1], 1.0);
                worldCord.convertTo(worldCord, CV_64FC1);
                worldCord = eo_Kinv*worldCord;
                worldCord *= camera.getDistance(); //1.0414
                yoyo_z_dis = camera.getGroundHeight() - worldCord.at<double>(0,1);
                //cout << "eo " << yoyo_z_dis << " based on " << det->c[0] << " " << det->c[1] << endl;
            }
            zarray_destroy(detections);
            // imshow("EO Tag Detections", eo_frame);
            // int key = (cv::waitKey(1) & 0xFF);

            std_msgs::Float64 z_dis_msg = std_msgs::Float64();
            z_dis_msg.data = yoyo_z_dis;
            eo_measure_pub.publish(z_dis_msg);
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
    ros::init(argc, argv, "eotracking");
    ros::NodeHandle n;


    int width;
    int height;
    double eo_height;
    double eo_dis;


    loadROSParam<int>(n, "width", width);
    loadROSParam<int>(n, "height", height);
    loadROSParam<double>(n, "eo_height", eo_height);
    loadROSParam<double>(n, "eo_dis", eo_dis);


    EO camera = EO(width, height, eo_dis, eo_height);
    EO_Tracking tracking = EO_Tracking(n, camera);
    ros::spin();


    return 0;

}