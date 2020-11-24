//
//  IO.cpp
//  efficient CPG model
//
//  Created by Azhar Aulia Saputra on 2020/08/24.
//  Copyright © 2020 Azhar Aulia Saputra. All rights reserved.
//

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "main.h"
#include "CPG.h"
//#include "SNN.h"

FILE *OutputFile;
FILE *OutputFile2;
FILE *ParamFile;
void ReadParamFile()
{
    int i,j;
    
    NumberOfNeurons = 4;
    MaxTime = 6000;
    
    
    MatrixAllocate(&out_n,num_neuron_max,MaxTime);
    MatrixAllocate(&out_y,num_neuron_max,MaxTime);
    MatrixAllocate(&out_m,num_neuron_max,MaxTime);
    
    VectorAllocate(&t_out,num_neuron_max);
    VectorAllocate(&GAweight,num_neuron_max * num_neuron_max);
    
}
void OpenOutputFile()
{
    
}
