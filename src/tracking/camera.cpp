#include "camera.hpp"
#include "flir.hpp"
#include "eo.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

using namespace FLIR;
using namespace UEYE;



cam::Camera::Camera(int width, int height, double eo_dis, double flir_dis, double eo_height, double flir_height) :
flir(Flir(0, width, height, flir_dis, flir_height)),
eo(EO(width, height, eo_dis, eo_height)),
backup(false),
width(width),
height(height)
{
}


void cam::Camera::getNextFrame(cv::Mat &frame){
    if (backup){
        flir.getNextFrame(frame);
    }else{
        eo.getNextFrame(frame);
    }
}

void cam::Camera::switchCam(){
    backup = !backup;
}

void cam::Camera::getKinv(cv::Mat &Kinverse){
    if (backup){
        flir.getKinv(Kinverse);
    }else{
        eo.getKinv(Kinverse);
    }
}


string cam::Camera::getCurrCamName(){
    if (backup){
        return "flir";
    }else{
        return "eo";
    }
}


double cam::Camera::getDistance(){
    if (backup){
        return flir.getDistance();
    }else{
        return eo.getDistance();
    }
}


double cam::Camera::getGroundHeight(){
    if (backup){
        return flir.getGroundHeight();
    }else{
        return eo.getGroundHeight();
    }
}