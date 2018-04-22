#include <iostream>
#include <opencv2/opencv.hpp>
#include "kinect2Sensor.h"
#include "algo.h"

using namespace std;
using namespace cv;

int main(){

    kinect2Sensor kinect_sensor;

    kinect_sensor.initKinectDepth();
    kinect_sensor.initKinectColor();

    const int depthWidth = kinect_sensor.depthFrameWidth;
    const int depthHeight = kinect_sensor.depthFrameHeight;
    IDepthFrame* depthFrame = nullptr;
    UINT depthBufferSize = depthHeight*depthWidth;

    const int depthListLen = 10;

    static Mat depthFrameList[depthListLen];
    int curDepthFrame = 0;
    int lstDepthFrame = 0;

    for (int i = 0; i < depthListLen; i++)
    {
        depthFrameList[i] = Mat::zeros(depthHeight, depthWidth, CV_16UC1);
    }
    Mat depDiffMat0 = Mat::zeros(depthHeight, depthWidth, CV_16UC1);
    Mat depDiffMat1 = Mat::zeros(depthHeight, depthWidth, CV_16UC1);
    Mat BFSLabelMat = Mat::zeros(depthHeight, depthWidth, CV_8UC1);
    Mat depNonZero0 = Mat::zeros(depthHeight, depthWidth, CV_8UC1);
    Mat depNonZero1 = Mat::zeros(depthHeight, depthWidth, CV_8UC1);

    Mat dep8bit = Mat::zeros(depthHeight, depthWidth, CV_8UC1);

    Mat depHistory = Mat::zeros(depthHeight, depthWidth, CV_8UC1);

    const int colorWidth = kinect_sensor.colorFrameWidth;
    const int colorHeight = kinect_sensor.colorFrameHeight;
    IColorFrame* colorFrame = nullptr;
    UINT colorBufferSize = colorHeight*colorWidth;
    Mat colorMat = Mat::zeros(colorHeight, colorWidth, CV_8UC3);
    Mat colorSplit[4];
    //Enter main loop
    while (true)
    {
        if (kinect_sensor.depthFrameReader->AcquireLatestFrame(&depthFrame) == S_OK)
        {

            curDepthFrame++;
            curDepthFrame %= depthListLen;
            depthFrame->CopyFrameDataToArray(depthBufferSize, (UINT16*)depthFrameList[curDepthFrame].data);
            depthFrame->Release();

            depDiffMat0 = depthFrameList[curDepthFrame] - depthFrameList[ (curDepthFrame - 1 + depthListLen) % depthListLen];
            depNonZero0 = (depDiffMat0 < 2000 & depDiffMat0 > 30);
            depthFrameList[curDepthFrame].convertTo(dep8bit, CV_8UC1, 255.0f / 5000);
            depDiffMat1 = depthFrameList[(curDepthFrame - 1 + depthListLen) % depthListLen] - depthFrameList[(curDepthFrame - 2 + depthListLen) % depthListLen];
            depNonZero1 = (depDiffMat1 < 2000 & depDiffMat1 > 30);
            erode(depNonZero0, depNonZero0, cv::Mat::ones(3, 3, CV_8UC1));
            bitwise_and(depNonZero0, depNonZero1, depNonZero1);
            if (countNonZero(depNonZero1)>0)
            {
                vector<Point2i> startPoints;
                cv::findNonZero(depNonZero1, startPoints);
                for (int i = 0; i < startPoints.size(); ++i)
                {
                    Point3i objCenter;
                    BFS_Online(depthFrameList[curDepthFrame], BFSLabelMat, startPoints[i], objCenter, 30);
                    if (objCenter.x != -1)
                    {
                        circle(depHistory, Point2i(objCenter.x, objCenter.y), 5, Scalar(255), 2);

                        circle(depNonZero1, Point2i(objCenter.x, objCenter.y), 20, Scalar(255), 2);

                    }
                }
            }

            imshow("Find Object", depHistory);
            imshow("Differ Mat", depthFrameList[curDepthFrame]);
            BFSLabelMat.setTo(0);

        }
        char keyInput = waitKey(30);

        if ( 27 == keyInput )
        {
            break;
        }
        else if ( 'c' == keyInput ){
            depHistory.setTo(0);
        }


    }

    kinect_sensor.releaseDepth();
    kinect_sensor.releaseColor();

    cout << "Press Enter to EXIT " << endl;

    cin.get();
    return 0;
}