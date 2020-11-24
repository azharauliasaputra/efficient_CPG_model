//
//  projection.hpp
//  efficient CPG model
//
//  Created by Azhar Aulia Saputra on 2020/11/24.
//  Copyright © 2020 Azhar Aulia Saputra. All rights reserved.
//

#ifndef projection_hpp
#define projection_hpp

#include <stdio.h>

double **MatrixMultiplication(double **m1, double **m2, int c);
double dotProduct(double v1[], double v2[]);
double dotProductPar(double v1[], double v2[], int p);
double vectorValue(double v1[]);
double vectorValue2(double v1[], int p);
double *vectorRed(double v1[], double v2[]);
double *vectorUnit(double v1[]);
double *redProductPar(double v1[], double v2[], int p);
double *addProductPar(double v1[], double v2[], int p);
double cosValue(double v1[], double v2[], int p);
double *outputMatrixRotation(double *pos, double *pos0, double ax,double ay,double az);

double *vectorSubtraction(double *v1, double *v2, int j);
float *vectorSubtraction_f(float *v1, float *v2, int j);
double *vectorAdd(double v1[], double v2[], int j);
double *vectorProduct(double v1[], double v2[], int j);
double *vectorScale(double a, double v2[], int j);
float *vectorScale_f(float a, float v2[], int j);
double *Angle3DVector(double *x1, double *x2, double *y1, double *y2, double *z1, double *z2);
double *Rotation3D(double *R, double *pos0);
double norm(double V[], int j);
float norm_f(float V[], int j);
double *crossProduct(double *v1, double *v2);
float *vectorUnit(float v1[]);
float *vectorUnit_f(float v1[]);
float *centerPointTriangle(double v1[], double v2[], double v3[]);
float *vectorAdd_f(float v1[], float v2[], int j);
double *rotation_z(double a, double *pos);


#endif /* projection_hpp */
