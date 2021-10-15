#ifndef CAMERA_HPP_
#define CAMERA_HPP_

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <iostream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <chrono>
#include <ctime>  
#include <aruco/aruco.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

#include "flir.hpp"
#include "eo.hpp"
#include <unistd.h>



using namespace std;
using namespace cv;


namespace cam{

    class Camera{
        public:
            Camera(int width, int height, double eo_dis, double flir_dis, double eo_height, double flir_height);
            void getNextFrame(Mat &frame);
            void switchCam();
            void getKinv(Mat &Kinverse);
            double getDistance();
            string getCurrCamName();
            double getGroundHeight();


        private:
            FLIR::Flir flir;
            UEYE::EO eo;
            bool backup;
            int width, height;

    };



}







#endif