#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "CPG.h"
#include "locoGenerator.hpp"
#include "malloc.hpp"
#include "main.h"

MATRIX out_n,                // internal state
out_y, out_m;            // input

VECTOR GAweight,    // individu in GA
t_out;

#define rad 57.295779513082320876798154814105
#define phi 3.1415926535897932384626433832795
#define sp2 1.4142135623730950488016887242097

#define Tr    1
#define Ta    12
int MaxTime;
int NumberOfNeurons;
float b    = 1.5;
float Tf = 2;

float OutNeuron[8]={0,0,0,0,0,0,0,0}; // data nilai output neuron untuk parameter global, biar bisa dibaca dari fungsi penjumlahan dari data output dikalikan weight.

float S_i[8];
float Sensor[8];
float t = 0;
float speedSTIM = 0;

float wMapNO[num_neuron_max][num_neuron_max] = {
    {2.5,1.8,2.5,2.5,2.5,2.5,2.5,2.5},    //1
    {1.8,2.5,2.5,2.5,2.5,2.5,2.5,2.5},    //2
    {2.5,2.5,2.5,2.5,2.5,2.5,2.5,2.5},    //3
    {2.5,2.5,2.5,2.5,2.5,2.5,2.5,2.5},    //1
    {2.5,2.5,2.5,2.5,2.5,2.5,2.5,2.5},    //1
    {2.5,2.5,2.5,2.5,2.5,2.5,2.5,2.5},    //1
    {2.5,2.5,2.5,2.5,2.5,2.5,2.5,2.5},    //1
    {2.5,2.5,2.5,2.5,2.5,2.5,2.5,2.5},    //1
};
float wMapCMD[num_neuron_max] = {
    2.5,1.8,2.5,2.5,2.5,2.5,2.5,2.5
};
float wMapSN[num_neuron_max][num_neuron_max] = {
    {2.5,1.8,2.5,2.5,2.5,2.5,2.5,2.5},    //1
    {1.8,2.5,2.5,2.5,2.5,2.5,2.5,2.5},    //2
    {2.5,2.5,2.5,2.5,2.5,2.5,2.5,2.5},    //3
    {2.5,2.5,2.5,2.5,2.5,2.5,2.5,2.5},    //1
    {2.5,2.5,2.5,2.5,2.5,2.5,2.5,2.5},    //1
    {2.5,2.5,2.5,2.5,2.5,2.5,2.5,2.5},    //1
    {2.5,2.5,2.5,2.5,2.5,2.5,2.5,2.5},    //1
    {2.5,2.5,2.5,2.5,2.5,2.5,2.5,2.5},    //1
};
float Gamma[4];

int P[4] = {0,1,2,3};
float thres[num_neuron] = {0.0,0.0,0.0,0.0};
float impulse[num_neuron] ={0,0,0,0};
void GlobalNeuronOutput(int index, float *a){
    int j = 0;
    for(j=0; j<index; j++){
        OutNeuron[j] = a[j];
    }
}
float WeightCalculation(int num){
    
    
    double  G[3];
    G[0] = 2.431;
    G[1] = 2.431;
    G[2] = 1.32;
    
    wMapNO[0][0] = 0.00;    wMapNO[0][1] = G[0];     wMapNO[0][2] = G[2];    wMapNO[0][3] = G[1];
    wMapNO[1][0] = G[2];    wMapNO[1][1] = 0.00;     wMapNO[1][2] = G[1];    wMapNO[1][3] = G[0];
    wMapNO[2][0] = G[0];    wMapNO[2][1] = G[1];     wMapNO[2][2] = 0.00;    wMapNO[2][3] = G[2];
    wMapNO[3][0] = G[1];    wMapNO[3][1] = G[2];     wMapNO[3][2] = G[0];    wMapNO[3][3] = 0.00;
    
    float x=0;int i;
    for(i=0; i<num_neuron; i++){
        x+=OutNeuron[i]*wMapNO[num][i];//*connection1[num][i];
    }
    return x;
}
double sX = 0;

double Noticeptor[4] = {1,1,1,1};
double swingPhase[4] = {0,0,0,0};
float NeuOscMatsuoka_x(int node_num, float xi, float vi){
    float x, w;
    w = WeightCalculation(node_num);
    
    double aff[4][4] ={
        {0,1.002,0.960,1.023},
        {0.960,0,1.023,1.002},
        {1.002,1.023,0,0.960},
        {1.023,0.960,1.002,0}
    };
    
    double aff_[4][4] ={
        {0, 0, 2.059, 0},
        {0.02, 0, 0, 0},
        {0, 0, 0, 0.02},
        {0, 1.987, 0, 0},
    };
    double aff_2[4][4] ={
        {0, 0.001, 0, 0},
        {1.875, 0, 0, 0},
        {0, 0, 0, 1.94},
        {0, 0, 0.001, 0},
    };

    S_i[node_num] = 1;// + jointFeed;
    double jointFeed = 0;
    
    
    double A = 3/(1 + exp(-8 * sX + 15));
    double B = 2*exp(log10(0.5) * (2 * sX - 4) * (2 * sX - 4));
    for(int i=0; i<4; i++){
        if(i == 0){
            jointFeed = THETA[1][1];
        }else if(i == 1){
            jointFeed = THETA[2][1];
        }else if(i == 2){
            jointFeed = THETA[0][1];
        }else if(i == 3){
            jointFeed = THETA[3][1];
        }
        if(jointFeed < 0.4) jointFeed = 0;
        
        S_i[node_num] += 1.5 * (double)(forceTouch[i]) * Noticeptor[i] * aff[node_num][i]
        - B*((double)(swingPhase[i]))* Noticeptor[i] * aff_2[node_num][i]
        - A*((double)(swingPhase[i]))* Noticeptor[i] * aff_[node_num][i];
        
    }
    x = (-xi - b*vi - w + S_i[node_num])/(Tr*Tf);
    return x;
    
}
float NeuOscMatsuoka_v(float vi, float yi){
    float v;
    v = (-vi + yi)/(Ta*Tf);
    return v;
}
float NeuOsc_output(int i, float a){
    float x;
    if(a >= thres[i])x = a;
    else x = 0.0;
    return x;
}

float x0[24] =  {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0,0,0,0,0,0,0,0},
v0[24] =  {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0,0,0,0,0,0,0,0};
float h1[16] =  {0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3},
h2[16] =  {0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3,0.3};
double DataTheta[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
float GAIN_[10] = {1,1,1,1,1,1,1,1,1,1};
float alphaGain = 0;
float global_x[num_neuron],global_x0[num_neuron],global_x1[num_neuron],global_x2[num_neuron],global_x3[num_neuron],
global_v[num_neuron],global_y[num_neuron];


int spike[num_neuron] = {0,0,0,0};
float href[num_neuron] = {0,0,0,0};
float spikeThres[num_neuron] = {0.2, 0.2, 0.2, 0.2};

void neuralOscInit(){
    int j;
    for(j=0; j<num_neuron; j++){
        global_x[j]    = x0[j]; // inner state neuron 1
        global_x0[j] = 0;
        global_x1[j] = 0;
        global_x2[j] = 0;
        global_x3[j] = 0;
        
        global_v[j]    = v0[j]; // degree of adaptation neuron 1
        
        global_y[j]    = NeuOsc_output(j,global_x[j]); // output neuron 1
        t_out[j] = 0.0;
    }
    GlobalNeuronOutput(num_neuron,global_y);
    speedSTIM = 0.0 + (float)(1 + ModeRunning) * 0.1;
    
    for(int i=0; i<1000; i++){
        RungeKuttaGill_NeuOscMatsuoka(i);
        tStep[0] = 0;tStep[1] = 0;tStep[2] = 0;tStep[3] = 0;
        stepTCount[0] = 0;stepTCount[1] = 0;stepTCount[2] = 0;stepTCount[3] = 0;
    }
    alphaGain = 0;
    fitPhase = 0;
//    initSNN();
}
#define Alpha1 30
void RungeKuttaGill_NeuOscMatsuoka(int t){
    float k1[8][2];
    float k2[8][2];
    float k3[8][2];
    float k4[8][2];
    
    
    float x1[8]    ,x2[8]    ,x3[8]    ,v1[8]    ,v2[8]    ,v3[8]    ;
    int j;
    
    sX = (double)(t)/600;
    stepLengthX = 40 + sX*6;
    
    GlobalNeuronOutput(num_neuron, global_y);
    for(j=0; j<num_neuron; j++){
        k1[j][0]= NeuOscMatsuoka_x(j,global_x[j],global_v[j]);
        x1[j]    = global_x[j] + k1[j][0]*h1[j]/2;
        global_y[j]    = NeuOsc_output(j,x1[j]);
        k1[j][1]= NeuOscMatsuoka_v(global_v[j],global_y[j]);
        v1[j]    = global_v[j] + k1[j][1]*h2[j]/2;
    }
    GlobalNeuronOutput(num_neuron, global_y);
    for(j=0; j<num_neuron; j++){
        k2[j][0]= NeuOscMatsuoka_x(j,x1[j],v1[j]);
        x2[j]    = global_x[j] + (sp2 - 0.5)*k1[j][0]*h1[j] + (1 - sp2)*k2[j][0]*h1[j];
        global_y[j]    = NeuOsc_output(j,x2[j]);
        k2[j][1]= NeuOscMatsuoka_v(v1[j],global_y[j]);
        v2[j]    = global_v[j] + (sp2 - 0.5)*k1[j][1]*h2[j] + (1 - sp2)*k2[j][1]*h2[j];
    }
    GlobalNeuronOutput(num_neuron, global_y);
    for(j=0; j<num_neuron; j++){
        k3[j][0]= NeuOscMatsuoka_x(j,x2[j],v2[j]);
        x3[j]    = global_x[j] - sp2*h1[j]*k2[j][0] + (1.0 + sp2)*h1[j]*k3[j][0];
        global_y[j]    = NeuOsc_output(j,x3[j]);
        k3[j][1]= NeuOscMatsuoka_v(v2[j],global_y[j]);
        v3[j]    = global_v[j] - sp2*h2[j]*k2[j][1] + (1.0 + sp2)*h2[j]*k3[j][1];
    }
    GlobalNeuronOutput(num_neuron, global_y);
    for(j=0; j<num_neuron; j++){
        k4[j][0]= NeuOscMatsuoka_x(j,x3[j],v3[j]);
        global_x[j]    = global_x[j] + (1.0/6.0)*(k1[j][0] + (1.0-sp2)*2*k2[j][0] + (1.0+sp2)*2*k3[j][0] + k4[j][0])*h1[j];
        global_y[j]    = NeuOsc_output(j,global_x[j]);
        k4[j][1]= NeuOscMatsuoka_v(v3[j],global_y[j]);
        global_v[j]    = global_v[j] + (1.0/6.0)*(k1[j][1] + (1.0-sp2)*2*k2[j][1] + (1.0+sp2)*2*k3[j][1] + k4[j][1])*h2[j];
    }
    for(j=0; j<num_neuron; j++){
        if((href[j] + global_y[j]) > spikeThres[j]){
            spike[j] = 1;
        }else{
            spike[j] = 0;
        }
        if(spike[j] == 1) tStep[P[j]] = 1;
        else tStep[P[j]] = 0;
        

        float gamma_ref = 0.98;
        float R = 30;
        if(spike[j]==1)
        {
            href[j] = gamma_ref*href[j]-R*Tf;
            spike[j] = 0;
        }
        else
        {
            href[j]=gamma_ref*href[j];
        }
        
        stepTCount[j]++;
    }

    printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n",tStep[P[0]], tStep[P[1]], tStep[P[2]], tStep[P[3]],
           touch[0], touch[1], touch[2], touch[3],
           forceTouch[0], forceTouch[1], forceTouch[2], forceTouch[3],
           global_y[0], global_y[1], global_y[2], global_y[3],
           href[0], href[1], href[2], href[3]);
    
}
void runNeuralOscillator(){
    int t;
    neuralOscInit();
    for(t = 0; t < MaxTime; t++){
        RungeKuttaGill_NeuOscMatsuoka(t);
    }
}

