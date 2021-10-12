#include "camera.hpp"
#include "flir.hpp"
#include "eo.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

using namespace FLIR;
using namespace UEYE;



cam::Camera::Camera(int width, int height) :
flir(Flir(0, width, height)),
eo(EO(width, height)),
backup(false),
width(width),
height(height),
currCamName("eo")
{
}


void cam::Camera::getNextFrame(cv::Mat &frame){
    if (backup){
        currCamName = "flir";
        flir.getNextFrame(frame);
    }else{
        currCamName = "eo";
        eo.getNextFrame(frame);
    }
}

void cam::Camera::switchCam(){
    backup = !backup;
}

string cam::Camera::getCurrCamName(){
    return currCamName;
}