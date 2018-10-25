//
// Created by howstar on 18-6-9.
//
#include "stdafx.h"

using namespace std;

vector<float> g_threshold_const_positive[VELO_LINE_DOWN + 1]; //点云障碍物阈值，正
vector<float> g_threshold_const_negative[VELO_LINE_DOWN + 1]; //点云障碍物阈值，负

// double VELO_ANG_32[32] = {-30.67, -9.33, -29.33, -8.00, -28.00,
//                           -6.67, -26.67, -5.33, -25.33, -4.00,
//                           -24.00, -2.67, -22.67, -1.33, -21.33,
//                           0.00, -20.00, 1.33, -18.67, 2.67,
//                           -17.33, 4.00, -16.00, 5.33, -14.67,
//                           6.67, -13.33, 8.00, -12.00, 9.33,
//                           -10.67, 10.67};

double VELO_ANG_32[32] = {-30.67, -29.33, -28.00, -26.67, -25.33,
                          -24.00, -22.67, -21.33, -20.00, -18.67,
                          -17.33, -16.00, -14.67, -13.33, -12.00,
                          -10.67, -9.33, -8.00, -6.67, -5.33, -4.00,
                          -2.67, -1.33, 0.00, 1.33, 2.67, 4.00,
                          5.33, 6.67, 8.00, 9.33, 10.67};

double velo_ang_rad[32];

//LiDAR pos information
double g_LiDAR_pos[6] = {0, 1.3, 2.99, 2.12, 0.55, 180.0};

//RFans LiDAR offset per line of four lines
//int off_idx[4]={0,55,14,70};

//std::string g_file_dir="/home/howstar/文档/Testdata-001_0/";

float calculatePAngleThrePitch(float obstacleHeightThre, float laserHeight, float tan_pitchR, float angle2centerR, int laserID)
{
    float h_d = obstacleHeightThre;
    float h_v = laserHeight;

    float pitchCompensationD = float(180.0 / M_PI * atan(tan_pitchR * cos(angle2centerR)));

    float laserVerticalAngleD = pitchCompensationD + 20.5f - laserID;
    float DO_g = (h_v - h_d) / tan((laserVerticalAngleD - 1.0) * M_PI / 180.0);
    float BO_g = h_v / tan(laserVerticalAngleD * M_PI / 180.0);
    float angleThreR = atan(h_d / (DO_g - BO_g));

    float minTh = 0.035;
    if (angleThreR < 0 && h_d > 0)
    {
        angleThreR = M_PI / 2.0;
    }
    else if (angleThreR > 0 && h_d < 0)
    {
        angleThreR = -M_PI / 2.0;
    }
    if (angleThreR < minTh && angleThreR > 0)
    {
        angleThreR = minTh;
    }
    if (angleThreR > -minTh && angleThreR < 0)
    {
        angleThreR = -minTh;
    }

    return angleThreR;
}

void filterThresholdOfObstacle(double laserHeightM, double laserPitchD, double addConstant, double mulConstant, double obsHeightThred)
{
    for (unsigned i = 0; i < VELO_LINE; i++)
    {
        velo_ang_rad[i] = VELO_ANG_32[i] / 180.0 * M_PI;
    }

    //parameters initialization
    ///count
    int numLine = VELO_LINE_DOWN;
    int numAng = ANG_THRESHOLD_COUNT / 2;
    //constant threshold
    //    float obstacleHeightThre = 0.1;
    float laserPitchR = laserPitchD / 180.0 * M_PI;
    float tan_pitchR = tan(laserPitchR);

    float delt_centerR = M_PI / numAng;
    //    float obstacleH = 0.1;
    for (unsigned ibeam = 0; ibeam <= numLine; ibeam++)
    {
        vector<float> threshold_positive_LineList(ANG_THRESHOLD_COUNT + 1);
        vector<float> threshold_negative_LineList(ANG_THRESHOLD_COUNT + 1);
        float angle2centerR = M_PI;
        for (unsigned jAngD = 0; jAngD <= numAng; jAngD++)
        {
            double changeConstant = addConstant + mulConstant * ibeam;

            float positive = calculatePAngleThrePitch(obsHeightThred, laserHeightM, tan_pitchR, angle2centerR, ibeam) + changeConstant;
            float negative = calculatePAngleThrePitch(-obsHeightThred, laserHeightM, tan_pitchR, angle2centerR, ibeam) - changeConstant;
            threshold_positive_LineList[jAngD] = positive;
            threshold_negative_LineList[jAngD] = negative;
            threshold_positive_LineList[ANG_THRESHOLD_COUNT - jAngD] = positive;
            threshold_negative_LineList[ANG_THRESHOLD_COUNT - jAngD] = negative;
            angle2centerR -= delt_centerR;
        }
        g_threshold_const_positive[ibeam] = threshold_positive_LineList;
        g_threshold_const_negative[ibeam] = threshold_negative_LineList;
    }
}

pcl::PointXYZI transform_point(pcl::PointXYZI pt)
{
    double pitchR = g_LiDAR_pos[3] * M_PI / 180;
    double rollR = g_LiDAR_pos[4] * M_PI / 180;
    double yawR = g_LiDAR_pos[5] * M_PI / 180;
    double dx = g_LiDAR_pos[0];
    double dy = g_LiDAR_pos[1];
    double dz = g_LiDAR_pos[2];

    //rotate the points from lidar to vehicle
    pt.y = pt.y * cos(pitchR) - pt.z * sin(pitchR);
    pt.z = pt.y * sin(pitchR) + pt.z * cos(pitchR);

    pt.x = pt.x * cos(rollR) + pt.z * sin(rollR);
    pt.z = -pt.x * sin(rollR) + pt.z * cos(rollR);

    pt.x = pt.x * cos(yawR) - pt.y * sin(yawR);
    pt.y = pt.x * sin(yawR) + pt.y * cos(yawR);

    pt.x += dx;
    pt.y += dy;
    pt.z += dz;

    return pt;
}

void readCaliFile(std::string path)
{
    ifstream ifstream1(path);
    string head;
    ifstream1 >> head >> g_LiDAR_pos[3] >> g_LiDAR_pos[4] >> g_LiDAR_pos[5];
    ifstream1 >> head >> g_LiDAR_pos[0] >> g_LiDAR_pos[1] >> g_LiDAR_pos[2];

    ifstream1.close();
}
