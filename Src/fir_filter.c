#include "fir_filter.h"

/* Digital filter designed by mkfilter/mkshape/gencode   A.J. Fisher
   Command line: /www/usr/fisher/helpers/mkfilter -Bu -Bp -o 4 -a 1.9230769231e-02 2.8846153846e-01 -l */

#define NZEROS 4
#define NPOLES 4
#define GAIN   1.186459913e+00f

static float xv[NZEROS+1], yv[NPOLES+1];

float filterfunction(float input)
{
				xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3]; xv[3] = xv[4]; 
        xv[4] = input / GAIN;
        yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3]; yv[3] = yv[4]; 
        yv[4] =   (xv[0] + xv[4]) - 2 * xv[2]
                     + ( -0.7105395412f * yv[0]) + (  0.0000000000f * yv[1])
                     + (  1.6608343709f * yv[2]) + ( -0.0000000000f * yv[3]);
        return yv[4];   
}

