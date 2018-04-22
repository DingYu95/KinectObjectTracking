#include "kinect2Sensor.h"
using namespace std;
using namespace cv;
kinect2Sensor:: kinect2Sensor(){
    cout << "Try to get default sensor" << endl;

    if (GetDefaultKinectSensor(&pSensor) != S_OK)
    {
        cerr << "Get Sensor failed" << endl;
    }

    //  Open sensor
    cout << "Try to open sensor" << endl;
    if (pSensor->Open() != S_OK)
    {
        cerr << "Can't open sensor" << endl;
    }

}

void kinect2Sensor::kinect_close(){
    releaseColor();
    releaseDepth();
    releaseInfra();
    closeCoordinateMapper();
    pSensor->Close();
    pSensor->Release();
    pSensor = nullptr;
}

void kinect2Sensor:: initKinectColor(){
    pSensor->get_ColorFrameSource(&colorFrameSource);
    colorFrameSource->get_FrameDescription(&pFrameDescription);
    pFrameDescription->get_Width(&colorFrameWidth);
    pFrameDescription->get_Height(&colorFrameHeight);

    pFrameDescription->Release();
    pFrameDescription = nullptr;

    colorFrameSource->OpenReader(&colorFrameReader);
    colorFrameSource->Release();
    colorFrameSource = nullptr;
}

void kinect2Sensor::releaseColor(){
    colorFrameReader->Release();
    colorFrameReader = nullptr;
}

void kinect2Sensor :: initKinectDepth(){
    pSensor->get_DepthFrameSource(&depthFrameSource);
    depthFrameSource->get_FrameDescription(&pFrameDescription);
    pFrameDescription->get_Width(&depthFrameWidth);
    pFrameDescription->get_Height(&depthFrameHeight);

    pFrameDescription->Release();
    pFrameDescription = nullptr;

    depthFrameSource->OpenReader(&depthFrameReader);
    depthFrameSource->Release();
    depthFrameSource = nullptr;

}

void kinect2Sensor::releaseDepth(){
    depthFrameReader->Release();
    depthFrameReader = nullptr;
}

void kinect2Sensor::initKinectInfra()
{
    pSensor->get_InfraredFrameSource(&infraFrameSource);
    infraFrameSource->get_FrameDescription(&pFrameDescription);
    pFrameDescription->get_Width(&infraFrameWidth);
    pFrameDescription->get_Height(&infraFrameHeight);

    pFrameDescription->Release();
    pFrameDescription = nullptr;

    infraFrameSource->OpenReader(&infraFrameReader);
    infraFrameSource->Release();
    infraFrameSource = nullptr;
}

void kinect2Sensor::releaseInfra(){
    infraFrameReader->Release();
    infraFrameReader = nullptr;
}

void kinect2Sensor::initCoordinateMapper()
{
    if (pSensor->get_CoordinateMapper(&coordinateMapper) == S_OK)
    {
        cout << "Get coordinate Mapper succeed" << endl;
    }
}

void kinect2Sensor::closeCoordinateMapper()
{
    coordinateMapper->Release();
    coordinateMapper = nullptr;
}

void kinect2Sensor::getDepthINColorFrame(UINT16* depthBuffer, UINT16* D2C_buffer)
{
    DepthSpacePoint* colorBuffer = new DepthSpacePoint[colorFrameWidth* colorFrameHeight];

    if (coordinateMapper->MapColorFrameToDepthSpace(depthFrameHeight*depthFrameWidth, depthBuffer, colorFrameHeight*colorFrameWidth, colorBuffer) == S_OK)
    {
        for (int colorY = 0; colorY < colorFrameHeight; ++colorY){
            for (int colorX = 0; colorX < colorFrameWidth; ++colorX)
            {
                const unsigned int colorIndex = colorY * colorFrameWidth + colorX;
                const int depthX = static_cast<int>(colorBuffer[colorIndex].X + 0.5f);
                const int depthY = static_cast<int>(colorBuffer[colorIndex].Y + 0.5f);
                if ((0 <= depthX) && (depthX < depthFrameWidth) && (0 <= depthY) && (depthY < depthFrameHeight)){
                    const unsigned int depthIndex = depthY * depthFrameWidth + depthX;
                    D2C_buffer[colorIndex] = depthBuffer[depthIndex];
                }
            }
        }
    }
    delete[] colorBuffer;
    // e.g. Mapped Depth Buffer to cv::Mat
    //cv::Mat depthMat = cv::Mat( colorFrameHeight, colorFrameWidth, CV_16UC1, &D2C_buffer[0] ).clone();
}

void kinect2Sensor::getDepthINColorPoints(UINT16* depthBuffer, vector<int> &POI_Dx, vector<int> &POI_Dy, vector<int> &POI_Cx, vector<int> & POI_Cy)
{
    DepthSpacePoint* colorBuffer = new DepthSpacePoint[colorFrameHeight*colorFrameWidth];
    if (coordinateMapper->MapColorFrameToDepthSpace(depthFrameHeight*depthFrameWidth, depthBuffer, colorFrameHeight*colorFrameWidth, colorBuffer) == S_OK){
        vector<int>::iterator iter_x = POI_Cx.begin();
        vector<int>::iterator iter_y = POI_Cy.begin();
        for (; iter_x != POI_Cx.end(); ++iter_x, ++iter_y){
            const unsigned int colorIdx = (*iter_y) * colorFrameWidth + (*iter_x);
            const unsigned int depthX = static_cast<int>(colorBuffer[colorIdx].X + 0.5f);
            const unsigned int depthY = static_cast<int>(colorBuffer[colorIdx].Y + 0.5f);

            if ((depthX >= 0) && (depthX < depthFrameWidth) && (depthY >= 0) && (depthY < depthFrameHeight))
            {
                POI_Dx.push_back(depthX);
                POI_Dy.push_back(depthY);
            }
        }
    }
    delete[] colorBuffer;
}


void kinect2Sensor::getDepthINColorPoints(UINT16* depthBuffer, vector<cv::Point2i> &POI_D, vector<cv::Point2i> &POI_C)
{
    DepthSpacePoint* colorBuffer = new DepthSpacePoint[colorFrameHeight*colorFrameWidth];
    if (coordinateMapper->MapColorFrameToDepthSpace(depthFrameHeight*depthFrameWidth, depthBuffer, colorFrameHeight*colorFrameWidth, colorBuffer) == S_OK){
        vector<cv::Point2i>::iterator iter_xy = POI_C.begin();

        for (; iter_xy != POI_C.end(); ++iter_xy){
            const unsigned int colorIdx = (iter_xy->y) * colorFrameWidth + (iter_xy->x);
            const unsigned int depthX = static_cast<int>(colorBuffer[colorIdx].X + 0.5f);
            const unsigned int depthY = static_cast<int>(colorBuffer[colorIdx].Y + 0.5f);

            if ((depthX >= 0) && (depthX < depthFrameWidth) && (depthY >= 0) && (depthY < depthFrameHeight))
            {
                POI_D.push_back(cv::Point2i(depthX, depthY));
            }
        }
    }
    delete[] colorBuffer;
}

void kinect2Sensor::getDepthINColorPoints(UINT16* depthBuffer, cv::Mat &pointsMask, vector<cv::Point2i> &POI_C)
{
    static DepthSpacePoint* colorBuffer = new DepthSpacePoint[colorFrameHeight*colorFrameWidth];

    if (coordinateMapper->MapColorFrameToDepthSpace(depthFrameHeight*depthFrameWidth, depthBuffer, colorFrameHeight*colorFrameWidth, colorBuffer) == S_OK){
        unsigned int colorIdx(0);
        unsigned int depthX(0);
        unsigned int depthY(0);

        // A fast but not safe version
        uchar* temp_ptr = nullptr;
        for (int i = 0; i != POI_C.size(); ++i)
        {
            colorIdx = POI_C[i].y * colorFrameWidth + POI_C[i].x;
            depthX = static_cast<int>(colorBuffer[colorIdx].X + 0.5f);
            depthY = static_cast<int>(colorBuffer[colorIdx].Y + 0.5f);

            if ((depthX > 0) && (depthX < depthFrameWidth) && (depthY > 0) && (depthY < depthFrameHeight))
            {
                temp_ptr = pointsMask.ptr<uchar>(depthY);
                temp_ptr[depthX] = 255;
            }
        }

    }
    delete[] colorBuffer;
}

void kinect2Sensor::getColorINDepthFrame(UINT16* depthBuffer, BYTE* colorBuffer)
{
    int colorBytesPerPixel = 4;


    std::vector<ColorSpacePoint> colorSpacePoints(depthFrameWidth * depthFrameHeight);
    if (coordinateMapper->MapDepthFrameToColorSpace(depthFrameWidth * depthFrameHeight, &depthBuffer[0], colorSpacePoints.size(), &colorSpacePoints[0]) == S_OK){

        // Mapped Color Buffer

        std::vector<BYTE> buffer(depthFrameWidth * depthFrameHeight * colorBytesPerPixel);

        // Mapping Color Data to Depth Resolution
        for (int depthY = 0; depthY < depthFrameHeight; depthY++){
            for (int depthX = 0; depthX < depthFrameWidth; depthX++){
                const unsigned int depthIndex = depthY * depthFrameWidth + depthX;
                const int colorX = static_cast<int>(colorSpacePoints[depthIndex].X + 0.5f);
                const int colorY = static_cast<int>(colorSpacePoints[depthIndex].Y + 0.5f);
                if ((0 <= colorX) && (colorX < colorFrameWidth) && (0 <= colorY) && (colorY < colorFrameHeight)){
                    const unsigned int colorIndex = colorY * colorFrameWidth + colorX;
                    buffer[depthIndex * colorBytesPerPixel + 0] = colorBuffer[colorIndex * colorBytesPerPixel + 0];
                    buffer[depthIndex * colorBytesPerPixel + 1] = colorBuffer[colorIndex * colorBytesPerPixel + 1];
                    buffer[depthIndex * colorBytesPerPixel + 2] = colorBuffer[colorIndex * colorBytesPerPixel + 2];
                    buffer[depthIndex * colorBytesPerPixel + 3] = colorBuffer[colorIndex * colorBytesPerPixel + 3];
                }
            }
        }

    }
}

void kinect2Sensor::getColorINDepthRect(cv::Rect &depthRect, cv::Rect &colorRect, UINT16* depthFrame)
{
    DepthSpacePoint depthPointsBuffer[4];
    ColorSpacePoint colorPointsBuffer[4];
    UINT16 depthOfPoints[4];

    int depthX[4];
    int depthY[4];

    if (pSensor->get_CoordinateMapper(&coordinateMapper) != S_OK)
    {
        return;
    }


    depthX[0] = static_cast<int>(depthRect.x + 0.5f);
    depthY[0] = static_cast<int>(depthRect.y + 0.5f);

    depthX[1] = static_cast<int>(depthRect.x + depthRect.width + 0.5f);
    depthY[1] = static_cast<int>(depthRect.y + 0.5f);

    depthX[2] = static_cast<int>(depthRect.x + 0.5f);
    depthY[2] = static_cast<int>(depthRect.y + depthRect.height + 0.5f);

    depthX[3] = static_cast<int>(depthRect.x + depthRect.width + 0.5f);
    depthY[3] = static_cast<int>(depthRect.y + depthRect.height + 0.5f);

    for (int i = 0; i < 4; ++i)
    {
        depthPointsBuffer[i].X = depthX[i];
        depthPointsBuffer[i].Y = depthY[i];
        depthOfPoints[i] = depthFrame[depthY[i] * depthFrameWidth + depthX[i]];
        /*if (coordinateMapper->MapDepthPointToColorSpace(depthPointsBuffer[i], 4245, &colorPointsBuffer[i]) == S_OK);
        {
        colorRect.x = colorPointsBuffer[0].X;
        colorRect.y = colorPointsBuffer[0].Y;

        colorRect.width = colorPointsBuffer[1].X - colorRect.x;
        colorRect.height = colorPointsBuffer[2].Y - colorRect.y;
        }*/
    }

    coordinateMapper->MapDepthPointsToColorSpace(4, &depthPointsBuffer[0], \
        4, &depthOfPoints[0], 4, &colorPointsBuffer[0]);

}

void kinect2Sensor::getColorINDepthPoints(vector<cv::Point2i> &depthPoints, vector<cv::Point2i>&colorPoints, UINT16* depthFrame)
{
    static std::vector<DepthSpacePoint> depthPointsBuffer(depthPoints.size());
    static std::vector<ColorSpacePoint> colorPointsBuffer(colorPoints.size());
    static std::vector<UINT16> depthOfPoints(depthPoints.size());
    unsigned int depthX(0);
    unsigned int depthY(0);

    //Convert Points from opencv to points in Kinect
    //Looking for faster version

    for (int i = 0; i < depthPoints.size(); ++i)
    {
        depthX = static_cast<int>(depthPoints[i].x + 0.5f);
        depthPointsBuffer[i].X = depthX;
        depthY = static_cast<int>(depthPoints[i].y + 0.5f);
        depthPointsBuffer[i].Y = depthY;
        depthOfPoints[i] = depthFrame[depthY *depthFrameWidth + depthX];
    }


    coordinateMapper->MapDepthPointsToColorSpace(depthPointsBuffer.size(), &depthPointsBuffer[0], \
        depthPointsBuffer.size(), &depthOfPoints[0], colorPointsBuffer.size(), &colorPointsBuffer[0]);

    for (int i = 0; i < depthPoints.size(); ++i)
    {
        cv::Point2i temp(colorPointsBuffer[i].X, colorPointsBuffer[i].Y);
        colorPoints.push_back(temp);
    }
}

void kinect2Sensor::depthToCamSpace(cv::Point3i inputP, cv::Point3f camSpPoint)
{
    camSpPoint.x = (inputP.x - principalPointX) / focalX * inputP.z;
    camSpPoint.y = (inputP.y - principalPointY) / focalY * inputP.z;
    camSpPoint.z = inputP.z;
}

void kinect2Sensor::CamTodepthSpace(cv::Point3f inputP, cv::Point3i depthSpPoint)
{
    depthSpPoint.x = inputP.x / inputP.z * focalX + principalPointX;
    depthSpPoint.y = inputP.y / inputP.z * focalY + principalPointY;
    depthSpPoint.z = inputP.z;
}


