/// \file  eo.cpp
/// \brief class for managing the edmund optics camera
///


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

            /// \brief initialze an object for managing camera
            /// \param width width of the desired view
            /// \param height height of the desired view
            /// \param dis distance from the robot
            /// \param ground_height height from the ground
            EO(int width, int height, double dis, double ground_height);

            /// \brief destructor for the object
            ~EO();

            /// \brief get the latest frame
            /// \param frame the return frame
            void getNextFrame(cv::Mat & frame);

            /// \brief get camera matrix inverse
            /// \param Kinverse the return matrix
            void getKinv(cv::Mat & Kinverse);

            /// \brief get distance from the camera
            double getDistance();

            /// \brief get height from the ground of the camera
            double getGroundHeight();

            /// \brief get width of the view
            int getWidth();

            /// \brief get height of the view
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

            /// \brief undistort the camera view
            void createUndistortMap();
    };
}


#endif