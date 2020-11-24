//
//  locoGenerator.hpp
//  efficient CPG model
//
//  Created by Azhar Aulia Saputra on 2020/11/24.
//  Copyright © 2020 Azhar Aulia Saputra. All rights reserved.
//

#ifndef locoGenerator_hpp
#define locoGenerator_hpp

#include <stdio.h>

extern double ModeLad_x[4];
extern double ModeLad_y[4];
extern double ModeLad_z[4];

extern float timeStep;
extern float timeFSP[4];
extern float timeFSP2[4];
extern int stepAdd[4];
extern float Offset[3][4];

extern int dstep,tStep[4];
extern int stepTCount[4], stepTCount0[4], stepDisCount[4], stepDisCountRev[4], stepTiming[4];
extern float b,f;

extern float torsoX;
extern float bTorsoX[4];
extern float bTorsoX2[4];
extern float dir_foot_x[4];
extern float dir_foot_z[4];

extern float dTorsoP;
extern float torsoP;
extern float dTorsoZ;
extern float torsoZ;
extern float bTorsoZ[4];

extern float lCenterX[4];
extern float lCenterZ[4];

extern float torsoDirX;
extern float torsoDirZ;
extern float torsoDir;
extern float bTorsoDir[4];
extern float bTorsoDirZ[4];
extern float bTorsoDirX[4];
extern float tDir;

extern double timeInc[4], timeInc2[4], timeInc3[4];
extern int tTime[4], time_1[4];

extern float bZ,fZ,fPelvis;
extern float bZDir, bXDir;
extern float bDir,fDir[4], fZDir, fXDir;
extern float tr_dir_x[4];
extern float tr_dir_z[4];

extern int Touch[4];
extern float PelvisOscillation;
extern float Angle[4];
extern float stepLengthX;//-200;
extern float stepLengthZ;

extern float lengthStepX;
extern float lengthStepY;
extern float lengthStepZ;
extern float heightStep;
extern bool finishStep;
extern double h_leg[4];
extern float DirectionValue;


extern double x_foot[4];
extern double x_foot2[4];
extern double z_foot[4];
extern double z_foot2[4];
extern double y_foot[4];
extern double y_foot2[4];

extern double tr_x[4], tr_x_[4], tr_x0[4], tr_x1[4], tr_y[4], tr_y0[4], tr_z[4], tr_z0[4],  tr_z0[4], tr_z1[4],tr_z_[4]; //x trajectory
extern double tr_ye[4], tr_p, tr_dir[4];

extern float CurrentPosePos[3];
extern float CurrentPoseAngle[3];
extern float DesiredPoseAngle[3];
extern float DesiredPosePos[3];

extern int RESETPOSE;
extern int h_leg2[4];

void initTrajectory();
void Trajectory_walking();
void posRecovery3(int step);
void poseControl(float x, float y, float z, float ax, float ay, float az);
void timingControl(int leg, float fXX, float fZ, float bX, float bZ, float fDir);
//float getRotationalStep(int step, float theta, float xFoot[4], float zFoot[4], float lCenterX[4], float lCenterZ[4], int mode);

#endif /* locoGenerator_hpp */
