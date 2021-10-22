#ifndef EB_FEATURES_H
#define EB_FEATURES_H
#include "opencv2/opencv.hpp"

typedef struct eb_point
{
    double point_x;
    double point_y;
    double point_z;

    float dx;
    float dy;

    bool is_delaunay;

}eb_point_t;

typedef struct eb_points
{
    eb_point_t * points;
    int point_size;
}eb_points_t;

typedef struct eb_absorbent
{
    int adsorbent_num;
    int *adsorbent_index;
    double *adsorbent_dis;
    eb_point_t *adsorbent_foot;

}eb_absorbent_t;

typedef struct eb_line
{
    eb_point_t point_beg;
    eb_point_t point_end;  

    eb_absorbent_t adsorbent;

}eb_line_t;




typedef struct eb_lines
{
    eb_line_t *lines;
    int line_size;
}eb_lines_t;

typedef struct eb_mats
{
    int image_width;
    int image_height;
    cv::Mat input_image;
    cv::Mat canny_image;
    cv::Mat hough_image;
    cv::Mat buffer_image;
    cv::Mat buf_filter_image;
    cv::Mat adsorb_filter_image;
}eb_mats_t;

typedef struct eb_features
{
    eb_points_t boundary_points;
    eb_points_t delau_boundary_pois;
    eb_lines_t hough_lines;
    eb_lines_t buffer_filter_lines;

    eb_mats_t eb_mats;

}eb_features_t;


#endif