#pragma once

#include <iostream>
#include <vector>
// Kinect for Windows SDK Header
#include <Kinect.h>
#include <opencv2/opencv.hpp>

class kinect2Sensor {

private:
    IFrameDescription* pFrameDescription = nullptr;
    //Color configuration
    IColorFrameSource* colorFrameSource = nullptr;
    IColorFrame* colorFrame = nullptr;

    //Depth configuration
    IDepthFrameSource* depthFrameSource = nullptr;
    IDepthFrame* depthFrame = nullptr;

    //Infrared configuration
    IInfraredFrameSource* infraFrameSource = nullptr;
    IInfraredFrame* infraFrame = nullptr;
    //Coordinate Mapper configuration

    // DepthCamIntrinsics.
    // USE CoordinateMapper.getDepthCameraIntrinsics to obtain with a little trick,
    const float principalPointX = 256.473;
    const float principalPointY = 215.277;

    const float focalX = 365.492;
    const float focalY = 365.492;


public:
    IKinectSensor* pSensor = nullptr;

    //These attributes should be exposed to user
    IColorFrameReader* colorFrameReader = nullptr;
    IDepthFrameReader* depthFrameReader = nullptr;
    IInfraredFrameReader* infraFrameReader = nullptr;
    ICoordinateMapper* coordinateMapper = nullptr;

    int colorFrameWidth = 0;
    int colorFrameHeight = 0;
    int depthFrameWidth = 0;
    int depthFrameHeight = 0;
    int infraFrameWidth = 0;
    int infraFrameHeight = 0;

    kinect2Sensor:: kinect2Sensor();

    void kinect_close();

    void initKinectColor();

    void releaseColor();

    void initKinectDepth();

    void releaseDepth();

    void initKinectInfra();

    void releaseInfra();

    void initCoordinateMapper();

    void closeCoordinateMapper();

    //Map Entire Frame
    //UINT16* depthbuffer = new UINT16[depthFrameWidth*depthFrameHeight];
    //UINT16* buffer = new UINT16[colorFrameWidth*colorFrameHeight];
    void getDepthINColorFrame(UINT16* depthBuffer, UINT16* D2C_buffer);
    //Map points
    void getDepthINColorPoints(UINT16* depthBuffer, std::vector<int> &POI_Dx, std::vector<int> &POI_Dy, std::vector<int> &POI_Cx, std::vector<int> & POI_Cy);

    ///Overload ver2
    void getDepthINColorPoints(UINT16* depthBuffer, std::vector<cv::Point2i> &POI_D, std::vector<cv::Point2i> &POI_C);

    //Overload ver3
    void getDepthINColorPoints(UINT16* depthBuffer, cv::Mat &pointsMask, std::vector<cv::Point2i> &POI_C);

    //Map Entire Frame
    //UINT16* depthBuffer = new UINT16[depthFrameWidth*depthFrameHeight];
    //BYTE* colorBuffer = new BYTE[colorFrameWidth*colorFrameHeight*colorBytesPerPixel];
    void getColorINDepthFrame(UINT16* depthBuffer, BYTE* colorBuffer);

    void getColorINDepthRect(cv::Rect &depthRect, cv::Rect &colorRect, UINT16* depthFrame);

    void getColorINDepthPoints(std::vector<cv::Point2i> &depthPoints, std::vector<cv::Point2i>&colorPoints, UINT16* depthFrame);

    void depthToCamSpace(cv::Point3i inputP, cv::Point3f camSpPoint);

    void CamTodepthSpace(cv::Point3f inputP, cv::Point3i depthSpPoint);

};