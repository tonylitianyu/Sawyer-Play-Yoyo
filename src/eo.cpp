#include "eo.hpp"
#include <ueye.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <chrono>
#include <ctime>  
#include <aruco/aruco.h>
#include <opencv2/highgui.hpp>
#include <iostream>



using namespace UEYE;
using namespace std;
using namespace cv;

EO::EO(int width, int height) : 
width(width),
height(height)
{
    cout << "Launching EO camera" << endl;
    INT init_ret = is_InitCamera(&hCam, 0);
    cout << "init ret " << init_ret << endl;

    const wchar_t* settingsFile = L"/home/tianyu/Documents/Developer/camera_cpp/src/fps500.ini";
    INT param_ret = is_ParameterSet(hCam, IS_PARAMETERSET_CMD_LOAD_FILE, (void*)settingsFile, 0);
    cout << "param ret " << param_ret << endl;


    IS_RECT aoi;
    INT aoi_ret = is_AOI(hCam, IS_AOI_IMAGE_GET_AOI, (void*)&aoi, sizeof(aoi));
    cout << "aoi ret " << aoi_ret << endl; 


    pMem = NULL;
    memID = 0;
    int nPixelPerBit = 8;
    int pitch = 0;
    INT alloc_ret = is_AllocImageMem(hCam, width, height, nPixelPerBit, &pMem, &memID);
    cout << "alloc ret " << alloc_ret << endl;

    
    INT set_ret = is_SetImageMem(hCam, pMem, memID);
    cout << "set ret " << set_ret << endl;

    INT cap_ret = is_CaptureVideo(hCam, IS_DONT_WAIT);
    cout << "capture ret " << cap_ret << endl;

    //is_InquireImageMem(hCam, pMem, memID, &width, &height, &nPixelPerBit, &pitch);

    createUndistortMap();
}

void EO::getNextFrame(cv::Mat & frame){
    //is_InquireImageMem(hCam, pMem, memID, &width, &height, &nPixelPerBit, &pitch);
    VOID* img_mem;
    int getImageIntRet = is_GetImageMem(hCam, &img_mem);
	if (getImageIntRet != IS_SUCCESS) {
		cout << "Image data could not be read from memory!" << endl;
	}

    memcpy(frame.ptr(), img_mem, width*height);
    frame = frame.t();
    flip(frame, frame, 1);
    remap(frame, frame, map1, map2, INTER_LINEAR, BORDER_CONSTANT);


}    



EO::~EO(){
    is_FreeImageMem(hCam, pMem, memID);
}


void EO::createUndistortMap(){
    FileStorage fs;
    Mat K, D;
    fs.open("/home/tianyu/Documents/Developer/camera_cpp/src/eo_calibration.yml", FileStorage::READ);
    fs["camera_matrix"] >> K;
    fs["distortion_coefficients"] >> D;

    cout << "K = " << K << endl;
    cout << "D = " << D << endl;
    
    cv::initUndistortRectifyMap(K, D, 
                        Mat_<double>::eye(3,3), K, cv::Size(height,width), CV_8UC1, map1, map2 );
}
