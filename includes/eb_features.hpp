#ifndef EB_FEATURES_H
#define EB_FEATURES_H
#include "opencv2/opencv.hpp"
#include "eb_common_defines.hpp"

typedef struct eb_point
{
    double point_x;
    double point_y;
    double point_z;

    float dx;
    float dy;

    bool is_delaunay;
    bool is_adsorbed;

    int adsorb_line_idx;
    int delau_pois_idx;
    #ifdef TEST_EVALUATION
    int absorb_num;
    #endif

}eb_point_t;

typedef struct eb_points
{
    eb_point_t * points;
    int point_size;
}eb_points_t;

typedef struct eb_ins_points
{
    eb_point_t * points;
    int point_size;
    bool *insert_index;
}eb_ins_points_t;

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
    cv::Mat origin_image;
    cv::Mat input_image;
    cv::Mat canny_image;
    cv::Mat hough_image;
    cv::Mat contours_image;
    cv::Mat boundary_image;
    cv::Mat buffer_image;
    cv::Mat buf_filter_image;
    cv::Mat adsorb_filter_image;
    cv::Mat adsorb_update_image;
    cv::Mat simplify_lines_image;
    cv::Mat reset_lines_image;
    cv::Mat refine_liens_image;
    cv::Mat close_lines_image;
    cv::Mat roofs_image;
    cv::Mat roofs_lidar_image;
    cv::Mat org_roofs_lidar_image;
    cv::Mat ndvi_image;
    #ifdef MID_RESULT
    cv::Mat all_buffer_image;
    cv::Mat all_buf_filter_image;
    cv::Mat all_simply_image;
    cv::Mat all_refine_image;
    #endif
}eb_mats_t;

typedef struct eb_features
{
    eb_points_t org_palnar_pois;
    eb_points_t palnar_pois;
    eb_points_t boundary_points;
    eb_points_t delau_boundary_pois;
    eb_ins_points_t insert_pois;
    eb_lines_t hough_lines;
    eb_lines_t buffer_filter_lines;

    eb_mats_t eb_mats;

}eb_features_t;


typedef struct eb_final_line
{
    eb_point_t point_beg;
    eb_point_t point_end;

    double direct[2];  

    int poly_index;

}eb_final_line_t;

typedef struct eb_polygon
{
    eb_final_line_t *lines;  

    int line_size;

}eb_polygon_t;

typedef struct eb_roof
{
    eb_polygon_t *polygons;  
    int poly_size;

    eb_polygon_t basic_poly;
    double roof_direct[2];
}eb_roof_t;


#endif