#include "flir.hpp"
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

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace FLIR;
using namespace std;
using namespace cv;

Flir::Flir(int index, int width, int height) : 
width(width),
height(height)
{
    pCam = nullptr;
    system = System::GetInstance();
    camList = system->GetCameras();
    const unsigned int numCameras = camList.GetSize();
    cout << "Number of cameras detected: " << numCameras << endl << endl;
    if (numCameras == 0){
        cout << "No camera" << endl;
    }else{
        //configure camera and prepare for streaming
        pCam = camList.GetByIndex(0);

        pCam->Init();
        pCam->BeginAcquisition();
        cout << "Acquiring images..." << endl;
    }


    createUndistortMap();
}




void Flir::getNextFrame(Mat&frame){
    ImagePtr pResImage = pCam->GetNextImage();
    ImagePtr convertedImage = pResImage;
    frame = Mat(height, width, CV_8UC1, convertedImage->GetData(), convertedImage->GetStride());
    frame = frame.t();
    flip(frame, frame, 1);
    remap(frame, frame, map1, map2, INTER_LINEAR, BORDER_CONSTANT);
    pResImage->Release();
}


Flir::~Flir(){
    pCam->EndAcquisition();
    pCam->DeInit();
    pCam = nullptr;

    camList.Clear();
    system->ReleaseInstance();
}



void Flir::createUndistortMap(){
    FileStorage fs;
    Mat K, D;
    fs.open("/home/tianyu/Documents/Developer/camera_cpp/src/flir_calibration.yml", FileStorage::READ);
    fs["camera_matrix"] >> K;
    fs["distortion_coefficients"] >> D;

    cout << "K = " << K << endl;
    cout << "D = " << D << endl;
    
    cv::initUndistortRectifyMap(K, D, 
                        Mat_<double>::eye(3,3), K, cv::Size(height,width), CV_8UC1, map1, map2 );
}