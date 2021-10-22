#ifndef EB_CONFIG_H
#define EB_CONFIG_H

#include <iostream>

typedef struct file_config
{
    char image_path[256];
    char lidar_path[256];
    char oesm_path[256];
    char dem_path[256];
    char out_path[256];
}file_config_t;


typedef struct eb_config
{
    file_config_t file_config;

    double canny_thd_1;
    double canny_thd_2;

    double hough_thd_1;
    double hough_thd_2;
    int hough_thd_3;
    double hough_thd_4;
    double hough_thd_5;


    double pcl_filter_z1;
    double pcl_filter_z2;


    double pcl_estimate_thd;


    double transform_x;
    double transform_y;
    double transform_z;


    int buffer_matrix_size;
    float buffer_filter_f;

    int min_adsorb_num;
    double min_adsorb_dis;



    

}eb_config_t;





void read_eb_config(eb_config_t * eb_config_ptr, const char * file_path);


#endif