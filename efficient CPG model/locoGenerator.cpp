//
//  locoGenerator.cpp
//  efficient CPG model
//
//  Created by Azhar Aulia Saputra on 2020/11/24.
//  Copyright © 2020 Azhar Aulia Saputra. All rights reserved.
//

#include "locoGenerator.hpp"
//#include "locoMode2.h"
//#include "trajectory.h"
#include "main.h"
#include "CPG.h"
#include "projection.hpp"

#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include <math.h>

int stepAdd[4] = {0,0,0,0};

#define TIMEFSP 6
#define TIMEFSP2 0
#define TIMESTEP 16
#define timeStep2 12


double ModeLad_x[4] = {0,0,0,0};
double ModeLad_y[4] = {0,0,0,0};
double ModeLad_z[4] = {0,0,0,0};

float timeSwing = 0;
float timeStep = TIMESTEP;
float timeFSP[4] = {TIMEFSP, TIMEFSP, TIMEFSP,TIMEFSP};
float timeFSP2[4] = {TIMEFSP2, TIMEFSP2, TIMEFSP2,TIMEFSP2};
int tStep[4] = {0,0,0,0};
int stepTCount[4] = {0,0,0,0};
int stepTCount0[4] = {0,0,0,0};
int stepTiming[4] = {0,0,0,0};
int stepDisCount[4] = {0,0,0,0};
int stepDisCountRev[4] = {0,0,0,0};
int dstep = 0;
float bX=0,f=0, fX = 0;

float torsoX = 0;
float bTorsoX[4] = {0,0,0,0};
float bTorsoX2[4] = {0,0,0,0};
float bTorsoZ2[4] = {0,0,0,0};
float dir_foot_x[4] = {0,0,0,0};
float dir_foot_z[4] = {0,0,0,0};

float dTorsoP;
float torsoP = 0;
float dTorsoZ;
float torsoZ = 0;
float bTorsoZ[4] = {0,0,0,0};

float lCenterX[4];
float lCenterX2[4];
float lCenterZ[4];
float Offset[3][4];

float torsoDirX = 0;
float torsoDirZ= 0;
float torsoDir = 0;
float bTorsoDir[4] = {0,0,0,0};
float bTorsoDirZ[4] = {0,0,0,0};
float bTorsoDirX[4] = {0,0,0,0};
float tDirLeg[4] = {0,0,0,0};
float btDirLeg[4] = {0,0,0,0};
float tDir = 0;
float tDir0 = 0;
float dtDir = 0;

float bZ=0,fZ=0, fPelvis=0;
float bZDir=0, bXDir=0;
float bDir=0,fDir[4]={0,0,0,0},fDir2=0,fDir3=0, fZDir=0, fXDir=0;
float tr_dir_x[4] = {0,0,0,0};
float tr_dir_z[4] = {0,0,0,0};

int Touch[4];
float PelvisOscillation = 50;
float Angle[4];
float stepLengthX = 150;//-200;
float stepLengthZ = 0;
float center2base_x = 0, center2base_z = 0;

float pTime = 0;
float pTime1 = 10;
float xTime = 0;
float xTime1 = 10;
float a0=0,a1=0,a2=0,a3=0;


float lengthStepX = 0;
float lengthStepY = 0;
float lengthStepZ = 0;
float heightStep = 0;
double h_leg[4];
int h_leg2[4] = {0,0,0,0};
float DirectionValue;

double x_foot[4] = {0,0,0,0};
double x_foot2[4] = {0,0,0,0};
double z_foot[4] = {0,0,0,0};
double z_foot2[4] = {0,0,0,0};
double y_foot[4] = {0,0,0,0};
double y_foot2[4] = {0,0,0,0};

double timeInc[4];
double timeInc2[4];
double timeInc3[4];
int tTime[4] = {1,1,1,1},
time_1[4] = {static_cast<int>(timeStep),static_cast<int>(timeStep),static_cast<int>(timeStep),static_cast<int>(timeStep)};

double tr_x[4] = {0,0,0,0}, tr_x_[4] = {0,0,0,0}, tr_x0[4] = {0,0,0,0}, tr_x1[4] = {0,0,0,0}; //x trajectory
double tr_y[4] = {0,0,0,0}, tr_y0[4] = {0,0,0,0}; //x trajectory
double tr_z[4] = {0,0,0,0}, tr_z_[4] = {0,0,0,0}, tr_z0[4] = {0,0,0,0}, tr_z1[4] = {0,0,0,0}; //x 


float CurrentPosePos[3];
float CurrentPoseAngle[3];
float DesiredPoseAngle[3];
float DesiredPosePos[3];

void initTrajectory(){
    h_leg[4] = heightStep;
    THETA[0][3] = -phi/4;
    THETA[1][3] = -phi/4;
    THETA[2][3] = -phi/4;
    THETA[3][3] = -phi/4;
    
    
        DirectionValue = 0.0;
        stepLengthZ = 0;
        stepLengthX = 100;
    
    for(int i = 0; i < NumberOfLeg; i++){
        x_foot[i] = 0;
        y_foot[i] = 0;
        z_foot[i] = 0;
        x_foot2[i] = 0;
        bTorsoX[i] = 0;
        bTorsoZ[i] = 0;
        bTorsoX2[i] = 0;
        torsoP = 0;
    }
}
double* getRotationalStep(int step, float theta, double xFoot[4], double zFoot[4], float lCenterX[4], float lCenterZ[4], float stepLengthX, float stepLengthZ){
    double out=0;
    
    double *pos0 = (double *) malloc(sizeof (double) * 3);
    double *pos1 = (double *) malloc(sizeof (double) * 3);
    double *pos2 = (double *) malloc(sizeof (double) * 3);
//    x_foot[0] = 0;
//    x_foot[1] = 0;
//    x_foot[2] = 0;
//    x_foot[3] = 0;
    if(step == 0){
        double point2center_x = stepLengthX - lCenterX[0];
        center2base_x = ((lCenterX[2] - x_foot[2]) + (lCenterX[3] - x_foot[3]))/2;
        double point2base_x = point2center_x + center2base_x;
        
        double point2center_z = stepLengthZ + lCenterZ[0];
        center2base_z = 0;
        double point2base_z = point2center_z + center2base_z;
        
        pos0[0] = point2base_x;
        pos0[1] = point2base_z;
        pos0[2] = 0;
        pos1 = rotation_z(theta*2, pos0);
        
        double a = pos1[1];
        pos2[0] = pos1[0] - point2base_x;
        pos2[1] = pos1[1] - point2base_z;
        
    }else if(step == 1){
        double point2center_x = stepLengthX - lCenterX[1];
        center2base_x = ((lCenterX[2] - x_foot[2]) + (lCenterX[3] - x_foot[3]))/2;
        double point2base_x = point2center_x + center2base_x;
        
        double point2center_z = stepLengthZ + lCenterZ[1];
        center2base_z = 0;
        double point2base_z = point2center_z + center2base_z;
        
        pos0[0] = point2base_x;
        pos0[1] = point2base_z;
        pos0[2] = 0;
        pos1 = rotation_z(theta*2, pos0);
        double a = pos1[0];
        double b = pos1[1];
        pos2[0] = pos1[0] - point2base_x;
        pos2[1] = pos1[1] - point2base_z;
        
    }else if(step == 2){
        double point2center_x = stepLengthX - lCenterX[2];
        center2base_x = ((lCenterX[0] - x_foot[0]) + (lCenterX[1] - x_foot[1]))/2;
        double point2base_x = point2center_x + center2base_x;
        
        double point2center_z = stepLengthZ + lCenterZ[2];
        center2base_z = 0;
        double point2base_z = point2center_z + center2base_z;
        
        pos0[0] = point2base_x;
        pos0[1] = point2base_z;
        pos0[2] = 0;
        pos1 = rotation_z(theta*2, pos0);
        
        double a = pos1[1];
        pos2[0] = pos1[0] - point2base_x;
        pos2[1] = pos1[1] - point2base_z;
        
    }else if(step == 3){
        double point2center_x = stepLengthX - lCenterX[3];
        center2base_x = ((lCenterX[0] - x_foot[0]) + (lCenterX[1] - x_foot[1]))/2;
        double point2base_x = point2center_x + center2base_x;
        
        double point2center_z = stepLengthZ + lCenterZ[3];
        center2base_z = 0;
        double point2base_z = point2center_z + center2base_z;
        
        pos0[0] = point2base_x;
        pos0[1] = point2base_z;
        pos0[2] = 0;
        pos1 = rotation_z(theta*2, pos0);
        
        
        double a = pos1[0];
        double b = pos1[1];
        pos2[0] = pos1[0] - point2base_x;
        pos2[1] = pos1[1] - point2base_z;
        
    }else{
        out = 0;
    }
    
    return pos2;
}
float lStep[4] = { 0,0,0,0 };
float lStepZ[4] = { 0,0,0,0 };
int tPeriod[4] = { 70,70,70,70 };
float xPelvis = 0;
float xP = 0;
static int tP = 0;

float dSpeed = -1;
void patternControl() {
    float xPz = 0;
    if(fZ > 0) xPz = 20;
    if(fZ < 0) xPz = -20;
    
    if(tStep[0] == 1){
        xPelvis = xP-xPz;
    }
    else if(tStep[1] == 1){
        xPelvis = xP-xPz;
    }
    else if(tStep[2] == 1){
        xPelvis = -xP-xPz;
    }
    else if(tStep[3] == 1){
        xPelvis = -xP-xPz;
    }
    //if(stepAdd[0] != 0 && stepAdd[1] != 0 && stepAdd[2] != 0 && stepAdd[3] != 0)
        tP++;
}
bool finishStep = 0;
void Trajectory_walking(){
    int leg = 0, i;
    
    lCenterX[0] = c_x[1][0]*SCALE + ModeLad_x[0];
    lCenterX[1] = c_x[2][0]*SCALE + ModeLad_x[1];
    lCenterX[2] = c_x[0][0]*SCALE + ModeLad_x[2];
    lCenterX[3] = c_x[3][0]*SCALE + ModeLad_x[3];
    lCenterZ[0] = c_y[1][0]*SCALE - ModeLad_z[0];
    lCenterZ[1] = c_y[2][0]*SCALE - ModeLad_z[1];
    lCenterZ[2] = c_y[0][0]*SCALE - ModeLad_z[2];
    lCenterZ[3] = c_y[3][0]*SCALE - ModeLad_z[3];
    
    torsoX = 0;
    patternControl();
    for(leg=0; leg<NumberOfLeg; leg++){

        swingPhase[leg] = 0;
        
        if(stepAdd[leg] == 1){
            timeInc[leg] = (float)(tTime[leg]) / (float)(time_1[leg]);
            if (tTime[leg] >(timeFSP[leg]) && tTime[leg] <= (time_1[leg] - timeFSP2[leg]))
                timeInc3[leg] = (float)(tTime[leg] - timeFSP[leg]) / (float)(time_1[leg] - timeFSP[leg] - timeFSP2[leg]);
            else
                timeInc3[leg] = 0;
            
            tr_x0[leg] = 0;
            tr_z0[leg] = 0;
            tr_y[leg] = 0;
            
            tr_x0[leg] = tr_x1[leg];
            tr_x_[leg] = tr_x[leg];
            tr_x1[leg] = (float)(lStep[leg])*(sin((phi / 2)*(timeInc[leg])));
            tr_x[leg] = (float)(lStep[leg])*(sin((phi / 2)*(timeInc3[leg])));
            
            tr_z0[leg] = tr_z1[leg];
            tr_z_[leg] = tr_z[leg];
            tr_z1[leg] = (float)(lStepZ[leg])*(sin((phi / 2)*(timeInc[leg])));
            tr_z[leg] = (float)(lStepZ[leg])*(sin((phi / 2)*(timeInc3[leg])));
            
            tr_y[leg] = ((2 - (cos(1.9 * phi * timeInc3[leg])+1))* h_leg[leg]/2);
            
            btDirLeg[leg] = tDirLeg[leg];
            tDirLeg[leg] = (float)(fDir[leg])*(1 + sin(1.5 * phi + (phi/2)*(timeInc[leg])));
            
            tTime[leg] = tTime[leg] + 1;
            
            swingPhase[leg] = 1;
            
        }else if(stepAdd[leg] == 0){

            tr_x0[leg] = 0;//tr_x1[leg];
            tr_x1[leg] = 0;
            tr_x_[leg] = 0;
            tr_x[leg] = 0;
            tr_z0[leg] = 0;//tr_x1[leg];
            tr_z1[leg] = 0;
            tr_z_[leg] = 0;
            tr_z[leg] = 0;
            tDirLeg[leg] = 0;
            btDirLeg[leg] = 0;
            
            if(touch[leg] != 1)
                y_foot[leg] -= 3;
            else
                y_foot[leg] = y_foot[leg];
        }
        else if (stepAdd[leg] == 2) {
            tr_x0[leg] = 0;//tr_x1[leg];
            tr_x1[leg] = 0;
            tr_x_[leg] = 0;
            tr_x[leg] = 0;
            tr_z0[leg] = 0;//tr_x1[leg];
            tr_z1[leg] = 0;
            tr_z_[leg] = 0;
            tr_z[leg] = 0;
            tDirLeg[leg] = 0;
            btDirLeg[leg] = 0;
            
            if (y_foot[leg] < 1) y_foot[leg] = y_foot[leg] + 1;
            else if (y_foot[leg] > 1) y_foot[leg] = y_foot[leg] - 1;
            else y_foot[leg] = 0;
        }
        
        torsoX += ((tr_x1[leg] - tr_x0[leg]) + bTorsoX2[leg]) / 4;
        torsoZ += ((tr_z1[leg] - tr_z0[leg]) + bTorsoZ2[leg]) / 4;
    }
    
    float ptInc = pTime/pTime1;
    if(pTime1 != 0)torsoP = a3*ptInc*ptInc*ptInc+ a2*ptInc*ptInc+ a1*ptInc+ a0;
    torsoZ = ((tr_z[0] + bTorsoZ[0]) + (tr_z[1] + bTorsoZ[1]) + (tr_z[2] + bTorsoZ[2]) + (tr_z[3] + bTorsoZ[3]))/4;
    tDir0 = tDir;
    tDir = tDirLeg[0] + tDirLeg[1] + tDirLeg[2] + tDirLeg[3];
    dtDir = (tDirLeg[0] - btDirLeg[0]) + (tDirLeg[1] - btDirLeg[1]) + (tDirLeg[2] - btDirLeg[2]) + (tDirLeg[3] - btDirLeg[3]);
    //dtDir = 0;
    
    float A, B, C, D;
//    DirectionValue = 0.0;
    
    lCenterX2[0] = - center2base_x + lCenterX[0];
    lCenterX2[1] = - center2base_x + lCenterX[1];
    lCenterX2[2] = - center2base_x + lCenterX[2];
    lCenterX2[3] = - center2base_x + lCenterX[3];
    
    for(int i=0; i<NumberOfLeg; i++){
        A = + torsoX ;
        B = - torsoZ + torsoP;
        C = ((-A + lCenterX2[i]) * cos(dtDir) - lCenterX2[i] - (B - lCenterZ[i]) * sin(dtDir));
        D = ((B - lCenterZ[i]) * cos(dtDir) + lCenterZ[i] + (-A + lCenterX2[i]) * sin(dtDir));
        
        x_foot[i] = (tr_x[i] - tr_x_[i]) + C + bTorsoX[i] ;
        x_foot2[i] = (tr_x1[i] - tr_x0[i]) + C + bTorsoX2[i] ;
        z_foot[i] = (tr_z[i] - tr_z_[i]) + D + bTorsoZ[i] ;
        z_foot2[i] = (tr_z1[i] - tr_z0[i]) + D + bTorsoZ2[i];
        
        y_foot[i] = tr_y[i];
            
        dir_foot_x[i] = tr_dir_x[i] + bTorsoDirX[i] - torsoDirX;
        dir_foot_z[i] = tr_dir_z[i] + bTorsoDirZ[i] - torsoDirZ;
            
    }
    
    
    for(leg=0; leg<NumberOfLeg; leg++){
        if(tStep[leg] == 1 && swingPhase[leg] == 0){
            
            if(tStep[0] == 1){
                printf(" ");
            }
            finishStep = 0;
            stepAdd[leg] = 1;
            tStep[leg] = 0;
            
            
            float aP = torsoP;
            float bP = torsoP - dTorsoP;
            float cP = xPelvis;
            float dP = 0;
            if (cP > 50) cP = 50;
            if (cP < -50) cP = -50;
            pTime = 0;
            int tAa = tPeriod[1];
            if (tAa < 40) tAa = 40;
            pTime1 = ((float)(tAa)/75.0) * (abs(cP - torsoP));
            if(pTime1 > 30) pTime1 = 30;
            
            a0 = aP;
            a1 = bP;
            a2 = 3 * cP - 3 * aP - 2 * bP - dP;
            a3 = dP - 2 * cP + 2 * aP + bP;
            
            bX = -x_foot[leg];
            bZ = -z_foot[leg] +torsoP;
//            stepLengthX = 132;
            float a = stepLengthX / 40;
            if (a > 1) a = 1;
            else if (a < -1) a = -1;
            if (leg == 0 || leg == 1) {
                fDir[leg] = DirectionValue * (1 + a);
                if (leg == 1 && fDir[leg] > 0.5) fDir[leg] = 0.5;
            }
            else if (leg == 2 || leg == 3) {
                fDir[leg] = DirectionValue * (1 - a);
                if (leg == 2 && fDir[leg] > 0.5) fDir[leg] = 0.5;
            }
            
            fXDir = 0; fZDir = 0;
            //printf("%d\t%.3f\t%.3f", step, fXDir, fZDir);
            //if(stepLengthX != dSpeed)
            timingControl(leg, stepLengthX, stepLengthZ, bX, bZ, fDir[leg]);
            pTime1 = timeFSP[leg];
            
            f = fX + fXDir;
            fZ = stepLengthZ + fZDir;
            
            lStep[leg] = bX + f;
            if (leg == 0 && fZ < -60)fZ = -60;
            else if (leg == 2 && fZ < -60)fZ = -60;
            else if (leg == 1 && fZ > 60)fZ = 60;
            else if (leg == 3 && fZ > 60)fZ = 60;
            lStepZ[leg] = bZ + fZ;
            
            
            tTime[leg] = 1;
            time_1[leg] = timeStep;
            
        }
        else if((tTime[leg] > (time_1[leg]))){
            stepAdd[leg] = 2;
            tTime[leg] = 1;
            finishStep = 1;
        }
    }
    
    for(int leg=0; leg<NumberOfLeg; leg++){
        bTorsoX2[leg] = x_foot2[leg];
        bTorsoX[leg] = x_foot[leg];
        bTorsoZ2[leg] = z_foot2[leg];
        bTorsoZ[leg] = z_foot[leg];
    }
    
    poseControl(CurrentPosePos[0], CurrentPosePos[1], CurrentPosePos[2], CurrentPoseAngle[0], CurrentPoseAngle[1], CurrentPoseAngle[2]);

    pTime ++;
    if(pTime >= pTime1){
        pTime = pTime1;
    }
    dTorsoP = torsoP;
}
void timingControl(int leg, float fxx, float fZ, float bX, float bZ, float fDir) {
    float fXX = fmax(abs(fZ), fxx);
    if (fZ != 0) {
        
        if (fXX < 60 && fXX > 0) fXX = 60;
        else if (fXX > -60 && fXX < 0) fXX = -60;
        
    }
    
    if (fXX > 0 || fZ != 0 || fDir != 0 || bX != 0 || bZ != 0) {
        if(fDir != 0){
            if(fXX < 100) fXX = 100;
        }
        timeSwing = (150 - fXX) / 4.5;
        if (timeSwing < 25) timeSwing = 25;
        else if (timeSwing > 30) timeSwing = 30;
//        timeSwing = int(timeSwing/2);
        
        if (fxx == 0) fX = 0;
        else fX = 60 + fxx;
        if (fX > 180) fX = 180;
        
        if (fDir != 0) {
            double *Dir = (double *)malloc(sizeof(double) * 3);
            Dir = getRotationalStep(leg, fDir, x_foot, z_foot, lCenterX, lCenterZ, fX, 0);
            fXDir = Dir[0];
            fZDir = Dir[1];
        }
//        if(timeSwing > timeStep) timeSwing = timeStep;
//        timeStep = (150 - fXX / 2)/4;
        timeStep = timeSwing + timeSwing * (100/(fXX));
        timeFSP[leg] = timeStep - timeSwing;
        if (fXX > 130 && (leg == 2 || leg == 3)) {
//            if (timeStep < 40) timeStep = 40;
            tPeriod[0] = 150 - fXX;
            tPeriod[1] = 150 - fXX - 5;
            tPeriod[2] = 150 - fXX;
            tPeriod[3] = 150 - fXX - 5;
            
            if (tPeriod[0] < 30) tPeriod[0] = 30;
            if (tPeriod[1] < 2) tPeriod[1] = 2;
            if (tPeriod[2] < 30) tPeriod[2] = 30;
            if (tPeriod[3] < 2) tPeriod[3] = 2;
            
            xP = 0;
            dSpeed = stepLengthX;
        }
        else if (fXX < 130) {
//            if (timeStep < 40) timeStep = 40;
            tPeriod[0] = 140 - fXX*3;
            tPeriod[1] = 140 - fXX*3;
            tPeriod[2] = 140 - fXX*3;
            tPeriod[3] = 140 - fXX*3;
            
            if (tPeriod[0] < 20) tPeriod[0] = 20;
            if (tPeriod[1] < 40) tPeriod[1] = 40;
            if (tPeriod[2] < 20) tPeriod[2] = 20;
            if (tPeriod[3] < 40) tPeriod[3] = 40;
            
            xP = tPeriod[0] / 1.1;
            if (xP > 30) xP = 30;
            if (xP < 20) xP = 20;
            dSpeed = stepLengthX;
        }
        
        h_leg[leg] = 100;
        if(h_leg2[leg] == 1){
            h_leg2[leg] = 0;
            h_leg[leg] = 200;
            timeFSP[leg] -= 5;
            tPeriod[leg] += 10;
        }
        
        if (leg == 0) tP = tPeriod[0] + 1;
        else if (leg == 3) tP = tPeriod[0] + tPeriod[1] + 1;
        else if (leg == 1) tP = tPeriod[0] + tPeriod[1] + tPeriod[2] + 1;
        else if (leg == 2) tP = 0;
        
        
    }
    else {
        xP = 0;
        fX = 0;
        h_leg[leg] = 0;
        stepAdd[leg] = 2;
    }
    patternControl();
//    tP -= 1;
}
void posRecovery3(int step){
    float ite = 0.1;
    int i=0;
    for(i=0; i < 4; i++)
    {
        if(i != step){
            if(y_foot[i] > ite){
                y_foot[i]-=ite;
            }else if(y_foot[i] < -ite ){
                y_foot[i]+=ite;
            }else{
                if(y_foot[i] < ite && y_foot[i] > -ite ){
                    y_foot[i] =0.0;
                }
            }
        }
    }
    
}

int RESETPOSE = 0;

void poseControl(float x, float y, float z, float ax, float ay, float az){
    int leg;
    
    double *pos0 = (double *) malloc(sizeof (double) * 3);
    double *pos1 = (double *) malloc(sizeof (double) * 3);
    double *pos = (double *) malloc(sizeof (double) * 3);
    
    for(leg=0; leg<4; leg++){
        pos0[0] = 0;pos0[1] = 0;pos0[2] = 0;
        
        pos1[0] = -x_foot[leg] + lCenterX[leg] + x;
        pos1[1] = z_foot[leg] + lCenterZ[leg] + y;
        pos1[2] = y_foot[leg] - ModeLad_y[leg] + z;
        //            0.5*sin(2*phi*timeInc)
        pos = outputMatrixRotation(pos1, pos0, ax, ay, az);
        Offset[0][leg] = pos[0] - pos1[0] + x;
        Offset[2][leg] = pos[1] - pos1[1] + y;
        Offset[1][leg] = pos[2] - pos1[2] + z;
    }
    
    if(RESETPOSE == 1){
        for(int i=0; i<3; i++){
            CurrentPoseAngle[i] = 0;
            CurrentPosePos[i] = 0;
            DesiredPoseAngle[i] = 0;
            DesiredPosePos[i] = 0;
        }
        
        for(leg=0; leg<4; leg++){
            x_foot[leg] = x_foot[leg] - Offset[0][leg];
            y_foot[leg] = y_foot[leg] + Offset[1][leg];
            z_foot[leg] = z_foot[leg] + Offset[2][leg];
            
            Offset[0][leg] = 0;
            Offset[1][leg] = 0;
            Offset[2][leg] = 0;
        }
        RESETPOSE = 0;
        
    }
}
