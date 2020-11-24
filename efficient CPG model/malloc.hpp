//
//  malloc.hpp
//  efficient CPG model
//
//  Created by Azhar Aulia Saputra on 2020/11/24.
//  Copyright © 2020 Azhar Aulia Saputra. All rights reserved.
//

#ifndef malloc_hpp
#define malloc_hpp

#include <stdio.h>


void free2d_double(double ** a);
double **malloc2d_double(int x, int y);

void free2d_int(int ** a);
int **malloc2d_int(int x, int y);

char **malloc2d_char(int x, int y);


#endif /* malloc_hpp */
