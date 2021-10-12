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
            Camera(int width, int height);
            void getNextFrame(Mat &frame);
            string getCurrCamName();
            void switchCam();


        private:
            FLIR::Flir flir;
            UEYE::EO eo;
            string currCamName;
            bool backup;
            int width, height;

    };



}







#endif