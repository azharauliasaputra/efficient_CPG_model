//
//  CPG.h
//  efficient CPG model
//
//  Created by Azhar Aulia Saputra on 2020/11/24.
//  Copyright © 2020 Azhar Aulia Saputra. All rights reserved.
//

#ifndef CPG_h
#define CPG_h

#define num_neuron 4
#define num_neuron_max 12

extern int NumberOfNeurons;
extern int P[4];
extern float Tf;

extern int MaxTime;
extern float impulse[num_neuron];

extern float global_x[num_neuron],global_x0[num_neuron],global_v[num_neuron],global_y[num_neuron];

extern float wMapNO[num_neuron_max][num_neuron_max];
extern float wMapCMD[num_neuron_max];
extern float Gamma[4];
extern float speedSTIM;

extern double **dataweight;

void TrajectoryWalking(int t);
void RungeKuttaGill_NeuOscMatsuoka(int t);
void GlobalNeuronOutput(int index, float *a);
float WeightCalculation(int num);
float NeuOscMatsuoka_x(int node_num, float xi, float vi);
float NeuOscMatsuoka_v(float vi, float yi);
float NeuOsc_output(float a);
void neuralOscInit();
void runNeuralOscillator();
void ReadWeightMain(int s, int n);
void initReadWeight(int n);
extern float global_x[num_neuron],global_v[num_neuron],global_y[num_neuron];
extern double DataTheta[12];
extern float GAIN[10], alphaGain;
extern double swingPhase[4];

#endif /* CPG_h */
