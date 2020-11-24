//
//  main.h
//  efficient CPG model
//
//  Created by Azhar Aulia Saputra on 2020/11/24.
//  Copyright © 2020 Azhar Aulia Saputra. All rights reserved.
//

#ifndef main_h
#define main_h

#include <OpenGL/OpenGL.h>
#include <GLUT/GLUT.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "CPG.h"

#define rad 57.295779513082320876798154814105
#define phi 3.1415926535897932384626433832795
#define sp2 1.4142135623730950488016887242097
#define NumberOfLeg 4


#define LINK_NUM 5  // ëSÉäÉìÉNêî (total number of links)
#define JT_NUM   3  // ëSÉWÉáÉCÉìÉgêî (total number of joints)
#define LEG_NUM  4  // ëSãrêî (total number of legs)
#define laddernum 6
#define SCALE 717.4

#define num_obj 20
#define PI  3.141592653589793238462643383279

extern double desireDistance, fitMoveDistance, fitEnergy, fitStepLength, fitCountStep, fitPhase, fitMoveSpeed, fitStability;
extern double THETA[LEG_NUM][LINK_NUM];
extern FILE *OutputFile;

extern float data_pos[3];
extern float data_rot[3];
extern float angvel[3];

extern int touch[4];
extern int forceTouch[4];
extern double  c_x[LEG_NUM][LINK_NUM];
extern double  c_y[LEG_NUM][LINK_NUM];
extern int countTrain;

typedef double *VECTOR;
typedef VECTOR *MATRIX;

extern FILE *ParamFile;
extern FILE *OutputFile2;

//extern double gamma_ref,        // gamma refractriness
//gamma_syn,          // gamma spike output
//R,
//threshold;        // threshold

extern MATRIX out_n,                // internal state
out_y, out_m;            // input

extern VECTOR GAweight,    // individu in GA
t_out;

extern float wMap[num_neuron_max][num_neuron_max];
extern int flag;
void VectorAllocate(VECTOR *vector, int nCols);
void AllocateCols(VECTOR matrix[], int nRows, int nCols);
void MatrixAllocate(MATRIX *pmatrix, int nRows, int nCols);
void MatrixFree(MATRIX matrix,  int nRows);

void OpenOutputFile();
void ReadParamFile();
int movementControl(double target_x, double target_y, double direction);
void GetPositionObject();
void setDrawStuff();

void headingRobotAndPosition();
extern dReal  SX, SY, SZ;
extern double fitMoveDistance_pos[7];
extern double fitMoveSpeed_pos[7];
extern double fitPhase_pos[7];
extern double fitPattern;
extern double fitTiming;
extern double fitPattern_pos[7];
extern double fitTiming_pos[7];
extern int ModeRunning;


#endif /* main_h */
