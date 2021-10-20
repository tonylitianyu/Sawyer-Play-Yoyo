#ifndef EO_HPP_
#define EO_HPP_

#include <stdio.h>
#include <stddef.h>
#include <ueye.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <chrono>
#include <ctime>  
#include <aruco/aruco.h>
#include <opencv2/highgui.hpp>


namespace UEYE{
    class EO{
        public:
            EO(int width, int height, double dis, double ground_height);
            ~EO();
            void getNextFrame(cv::Mat & frame);
            void getKinv(cv::Mat & Kinverse);
            double getDistance();
            double getGroundHeight();

            int getWidth();
            int getHeight();

        private:
            HIDS hCam = 1;
            char* pMem = NULL;
            int memID = 0;
            cv::Mat map1, map2;
            int width, height;
            cv::Mat Kinv;
            double dis;
            double ground_height;

            void createUndistortMap();
    };
}


#endif