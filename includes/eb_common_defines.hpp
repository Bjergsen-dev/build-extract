#ifndef EB_COMMON_DEFINES_H
#define EB_COMMON_DEFINES_H

#include <stdio.h>
#include <time.h>

#define MIN_PIXEL_VAL 0
#define MAX_PIXEL_VAL 250
#define EB_LOG(...) printf(__VA_ARGS__)
#define EB_MAX(a,b) ((a) < (b) ? (b) : (a))
#define EB_MIN(a,b) ((a) > (b) ? (b) : (a))
#define MAT_SIZE 500
#define MIN_DOUBLE 1E-6
#define EB_DEBUG
//#define EB_PCL_VISUAL

#endif