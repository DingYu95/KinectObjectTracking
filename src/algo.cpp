#include "algo.h"
using namespace std;
using namespace cv;

void BFS_Online(cv::Mat &InputMat, cv::Mat &LabelMat, cv::Point2i cur_SP, cv::Point3i &cCenter, int Thresh){

    cCenter = cv::Point3i(-1, -1, -1);

    UINT16* const InputArray = (UINT16* const)InputMat.data;
    UINT8* LabelArray = LabelMat.data;
    ushort cur_Label = 255;
    //Return if already visited
    if (LabelArray[cur_SP.y * LabelMat.cols + cur_SP.x])
    {
        return;
    }
    UINT BFS_Queue_Array[10000][2];

    long sumX(0), sumY(0), sumZ(0);

    int left_most(999), right_most(0), up_most(999), down_most(0);

    const int width_max = 120;
    const int height_max = 120;

    //Start the BFS

    long cur_Array_Index = 0;
    long lst_Array_Index = 0;

    ++cur_Array_Index;
    BFS_Queue_Array[lst_Array_Index][0] = cur_SP.x;
    BFS_Queue_Array[lst_Array_Index][1] = cur_SP.y;
    BFS_Queue_Array[cur_Array_Index][0] = cur_SP.x;
    BFS_Queue_Array[cur_Array_Index][1] = cur_SP.y;
    cv::Point2i this_Point;

    while (cur_Array_Index< 9000 && lst_Array_Index < cur_Array_Index){

        this_Point.x = BFS_Queue_Array[lst_Array_Index][0];
        this_Point.y = BFS_Queue_Array[lst_Array_Index][1];

        if (this_Point.x > right_most){ right_most = this_Point.x; }
        if (this_Point.x < left_most){ left_most = this_Point.x; }
        if (this_Point.y < up_most){ up_most = this_Point.y; }
        if (this_Point.y > down_most){ down_most = this_Point.y; }

        if (this_Point.y >1 && this_Point.y < (InputMat.rows - 2) && this_Point.x > 1 && this_Point.x < (InputMat.cols - 2)){
            UINT16 this_Point_dep = InputArray[this_Point.y * InputMat.cols + this_Point.x];
            if (!this_Point_dep)
            {
                ++lst_Array_Index;
                continue;
            }

            UINT16 temp_dep = 0;
            //Left Point
            if (!LabelArray[this_Point.y * LabelMat.cols + this_Point.x - 1]){
                temp_dep = InputArray[this_Point.y * InputMat.cols + this_Point.x - 1];
                if (std::abs(this_Point_dep - temp_dep) < Thresh){
                    ++cur_Array_Index;
                    //left_most = this_Point.x - 1;
                    cv::Point2i left_Point(this_Point.x - 1, this_Point.y);
                    BFS_Queue_Array[cur_Array_Index][0] = left_Point.x;
                    BFS_Queue_Array[cur_Array_Index][1] = left_Point.y;
                    LabelArray[this_Point.y * LabelMat.cols + this_Point.x - 1] = cur_Label;
                    sumX += left_Point.x;
                    sumY += left_Point.y;
                    sumZ += temp_dep;
                }
            }
            //Right Point
            if (!LabelArray[this_Point.y * LabelMat.cols + this_Point.x + 1]){
                temp_dep = InputArray[this_Point.y * InputMat.cols + this_Point.x + 1];
                if (std::abs(this_Point_dep - temp_dep) < Thresh){
                    ++cur_Array_Index;
                    //right_most = this_Point.x + 1;
                    cv::Point2i right_Point(this_Point.x + 1, this_Point.y);
                    BFS_Queue_Array[cur_Array_Index][0] = right_Point.x;
                    BFS_Queue_Array[cur_Array_Index][1] = right_Point.y;
                    LabelArray[this_Point.y * LabelMat.cols + this_Point.x + 1] = cur_Label;
                    sumX += right_Point.x;
                    sumY += right_Point.y;
                    sumZ += temp_dep;
                }
            }
            //Up Point
            if (!LabelArray[(this_Point.y - 1) * LabelMat.cols + this_Point.x]) {
                temp_dep = InputArray[(this_Point.y - 1) * InputMat.cols + this_Point.x];
                if (std::abs(this_Point_dep - temp_dep) < Thresh){
                    ++cur_Array_Index;
                    //up_most = this_Point.y - 1;
                    cv::Point2i up_Point(this_Point.x, this_Point.y - 1);
                    BFS_Queue_Array[cur_Array_Index][0] = up_Point.x;
                    BFS_Queue_Array[cur_Array_Index][1] = up_Point.y;
                    LabelArray[(this_Point.y - 1) * LabelMat.cols + this_Point.x] = cur_Label;
                    sumX += up_Point.x;
                    sumY += up_Point.y;
                    sumZ += temp_dep;
                }
            }
            //Down Point
            if (!LabelArray[(this_Point.y + 1) * LabelMat.cols + this_Point.x]){
                temp_dep = InputArray[(this_Point.y + 1) * InputMat.cols + this_Point.x];
                if (std::abs(this_Point_dep - temp_dep) < Thresh){
                    ++cur_Array_Index;
                    //down_most = this_Point.y + 1;
                    cv::Point2i down_Point(this_Point.x, this_Point.y + 1);
                    BFS_Queue_Array[cur_Array_Index][0] = down_Point.x;
                    BFS_Queue_Array[cur_Array_Index][1] = down_Point.y;
                    LabelArray[(this_Point.y + 1) * LabelMat.cols + this_Point.x] = cur_Label;
                    sumX += down_Point.x;
                    sumY += down_Point.y;
                    sumZ += temp_dep;
                }
            }
        }

        /*if ((down_most - right_most) > height_max)
        {
        return;
        }
        if ((right_most - left_most) > width_max)
        {
        return ;
        }
        if (abs(abs(down_most - up_most) - abs(right_most - left_most)) > 10)
        {
        return;
        }*/
        ++lst_Array_Index;
    }


    if (cur_Array_Index <60 || (right_most - left_most) > width_max || (down_most - up_most) > height_max )
    {
        return;
    }

    cCenter.x = (int)sumX * 1.0 / cur_Array_Index;
    cCenter.y = (int)sumY * 1.0 / cur_Array_Index;
    cCenter.z = (int)sumZ * 1.0 / cur_Array_Index;

    return;
}


void Gen_CoMat1(double CoeMat[4], double addTi, double addDi){
    double Ti1 = addTi;
    double Ti2 = Ti1* addTi;

    CoeMat[0] = CoeMat[0] + 1;		CoeMat[1] = CoeMat[1] + Ti1;
    CoeMat[2] = CoeMat[2] + Ti1;		CoeMat[3] = CoeMat[3] + Ti2;

    CoeMat[4] = CoeMat[4] + addDi;
    CoeMat[5] = CoeMat[5] + addDi * Ti1;

}

void Gen_CoMat2(double CoeMat[12], double addTi, double addDi){
    double Ti1 = addTi;
    double Ti2 = Ti1* addTi;
    double Ti3 = Ti2*addTi;
    double Ti4 = Ti3 * addTi;
    CoeMat[0] = CoeMat[0] + 1;		CoeMat[1] = CoeMat[1] + Ti1;		CoeMat[2] = CoeMat[2] + Ti2;
    CoeMat[3] = CoeMat[3] + Ti1;		CoeMat[4] = CoeMat[4] + Ti2;		CoeMat[5] = CoeMat[5] + Ti3;
    CoeMat[6] = CoeMat[6] + Ti2;		CoeMat[7] = CoeMat[7] + Ti3;		CoeMat[8] = CoeMat[8] + Ti4;

    CoeMat[9] = CoeMat[9] + addDi;
    CoeMat[10] = CoeMat[10] + addDi * Ti1;
    CoeMat[11] = CoeMat[11] + addDi* Ti2;


}

double forwardPred1(double timePoint, vector<double>&Input_CoMat){

    vector<double>poly1Coe;
    vector<double> adjuMat;
    adjuMat.push_back(Input_CoMat[3]);
    adjuMat.push_back(-Input_CoMat[2]);
    adjuMat.push_back(-Input_CoMat[1]);
    adjuMat.push_back(Input_CoMat[0]);

    double detInput_Co = Input_CoMat[0] * Input_CoMat[3] - Input_CoMat[1] * Input_CoMat[2];

    poly1Coe.push_back((adjuMat[0] * Input_CoMat[4] + adjuMat[2] * Input_CoMat[5]) / detInput_Co);
    poly1Coe.push_back((adjuMat[1] * Input_CoMat[4] + adjuMat[3] * Input_CoMat[5]) / detInput_Co);

    return poly1Coe[0] + poly1Coe[1] * timePoint;
}


double forwardPred2(double timePoint, vector<double>&Input_CoMat){

    double adjuMat[12];
    adjuMat[0] = (Input_CoMat[4] * Input_CoMat[8] - Input_CoMat[5] * Input_CoMat[7]);
    adjuMat[1] = -(Input_CoMat[3] * Input_CoMat[8] - Input_CoMat[5] * Input_CoMat[6]);
    adjuMat[2] = (Input_CoMat[3] * Input_CoMat[7] - Input_CoMat[4] * Input_CoMat[6]);
    adjuMat[3] = -(Input_CoMat[1] * Input_CoMat[8] - Input_CoMat[2] * Input_CoMat[7]);
    adjuMat[4] = (Input_CoMat[0] * Input_CoMat[8] - Input_CoMat[2] * Input_CoMat[6]);
    adjuMat[5] = -(Input_CoMat[0] * Input_CoMat[7] - Input_CoMat[1] * Input_CoMat[6]);
    adjuMat[6] = (Input_CoMat[1] * Input_CoMat[5] - Input_CoMat[2] * Input_CoMat[4]);
    adjuMat[7] = -(Input_CoMat[0] * Input_CoMat[5] - Input_CoMat[2] * Input_CoMat[3]);
    adjuMat[8] = (Input_CoMat[0] * Input_CoMat[4] - Input_CoMat[1] * Input_CoMat[3]);

    //Obtain Determinant of Input_Coefficient Matrix
    double det_InputCo;
    det_InputCo = \
        Input_CoMat[0] * (Input_CoMat[4] * Input_CoMat[8] - Input_CoMat[5] * Input_CoMat[7]) + \
        Input_CoMat[1] * -(Input_CoMat[3] * Input_CoMat[8] - Input_CoMat[5] * Input_CoMat[6]) + \
        Input_CoMat[2] * (Input_CoMat[3] * Input_CoMat[7] - Input_CoMat[4] * Input_CoMat[6]);

    //Obtain Reverse
    double poly2Coe[3];
    poly2Coe[0] = (adjuMat[0] * Input_CoMat[9] + adjuMat[3] * Input_CoMat[10] + adjuMat[6] * Input_CoMat[11]) / det_InputCo;
    poly2Coe[1] = (adjuMat[1] * Input_CoMat[9] + adjuMat[4] * Input_CoMat[10] + adjuMat[7] * Input_CoMat[11]) / det_InputCo;
    poly2Coe[2] = (adjuMat[2] * Input_CoMat[9] + adjuMat[5] * Input_CoMat[10] + adjuMat[8] * Input_CoMat[11]) / det_InputCo;


    return poly2Coe[0] + poly2Coe[1] * timePoint + poly2Coe[2] * timePoint*timePoint;

}
