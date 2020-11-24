//
//  memory_alloc.cpp
//  efficient CPG model

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>


typedef double *VECTOR;
typedef VECTOR *MATRIX;

/* Allocate space for vector of double cells for one dimensional dynamic vector[cols] */
void VectorAllocate(VECTOR *vector, int nCols)
{
   if ((*vector = (VECTOR) calloc(nCols, sizeof(double))) == NULL)
   {
      fprintf(stderr, "Sorry! Not enough memory\n");
      exit(1);
   }
}

/* Allocate space for columns (double cells) for dynamic two dimensional matrix[rows][cols] */
void AllocateCols(VECTOR matrix[], int nRows, int nCols)
{
   int  i;

   for (i = 0;  i < nRows;  i++)
      VectorAllocate(&matrix[i], nCols);
}

/* Allocate space for a two dimensional dynamic matrix[rows][cols] */
void MatrixAllocate(MATRIX *pmatrix, int nRows, int nCols)
{
   if ( (*pmatrix  =  (MATRIX) calloc(nRows,  sizeof(VECTOR) ) )   ==  NULL)
   {
      fprintf(stderr, "Sorry! Not enough memory\n");
      exit(1);
   }

   AllocateCols(*pmatrix, nRows, nCols);
}

/* Free space for a two dimensional dynamic matrix */
void MatrixFree(MATRIX matrix,  int nRows)
{
   int   i;
   for (i = 0;  i < nRows;  i++)
      free(matrix[i]);
   free(matrix);
}

