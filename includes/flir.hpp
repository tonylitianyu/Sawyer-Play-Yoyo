#ifndef FLIR_HPP_
#define FLIR_HPP_

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <chrono>
#include <ctime>  
#include <aruco/aruco.h>
#include <opencv2/highgui.hpp>



using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

namespace FLIR{
    class Flir
    {
        public:
            Flir(int index,int width, int height);
            ~Flir();
            void getNextFrame(cv::Mat & frame);

        private:
            CameraPtr pCam = nullptr;
            CameraList camList;
            SystemPtr system;
            cv::Mat map1, map2;
            int width, height;

            void createUndistortMap();


    };
}



#endif