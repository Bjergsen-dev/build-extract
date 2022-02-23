#ifndef EB_COMMON_DEFINES_H
#define EB_COMMON_DEFINES_H

#include <stdio.h>
#include <time.h>

#define MIN_PIXEL_VAL 10.
#define MAX_PIXEL_VAL 250.
#define MIN_DISPARITY 0.000005
#define EB_LOG(...) printf(__VA_ARGS__)
#define EB_MAX(a,b) ((a) < (b) ? (b) : (a))
#define EB_MIN(a,b) ((a) > (b) ? (b) : (a))
#define EB_LENGTH(a,b,c,d) sqrt(pow(a-b,2)+pow(c-d,2))
#define MAT_SIZE 500
#define INVALID_VAL -1000.0
#define MIN_DOUBLE 1E-6
#define EB_DEBUG
//#define EB_PCL_VISUAL
#define RB_NO_ROOF
//#define RB_NO_BASE
#define NO_FIT_GROUND
#define TEST_1
#define TEST_2
//#define TEST_NICE_GROUND
//#define TEST_DELAUNAY


#define TEST_8_SOBEL

//#define TEST_CANNY

//#define TEST_LoG

#define TEST_EVALUATION

//#define MID_RESULT

//#define TEST_IOU
//#define IOU_BRY

//#define TEST_LIDAR_ORIGIN_AND_DP

#endif