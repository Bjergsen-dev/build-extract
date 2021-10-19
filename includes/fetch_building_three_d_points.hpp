/**************************************************************************

Copyright:Best_Intelligence in LiesMars

Author: ZZC_Bjergsen

Date:2020-08-18

Description:Provide  functions  of GDAL

**************************************************************************/
#pragma once

#include "gdal_methods.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/core/core.hpp"
#include  <vector>
#include <fstream>
#include <iostream>
#include <iomanip>


///create a struct for x_y_z  point
typedef struct zzc_point{
public:
    float geo_x;
    float geo_y;
    float elevation;

    
    zzc_point(float x, float y, float ele){
        
        geo_x = x;
        geo_y = y;
        elevation = ele;
    }
}z_Point;


///create a struct for x_y_z  point
typedef struct zzc_plane{
public:
    std::vector<int> points_index_vec;
    std::vector<int> context_index_vec;
    int nor_index;
    
    zzc_plane(std::vector<int> &pps_index_vec, int n_index,std::vector<int> &con_index_vec){
        
        nor_index = n_index;
        points_index_vec = pps_index_vec;
        context_index_vec = con_index_vec;
    }
}z_Plane;

//change the pixel coodinate of building corner points to geo_X geo_Y elevation
/*
 @prama: vector<cv::Point> &points_vec -- input 2d points vec;
 @prama: const char * dom_filepath -- input dom file path;
 @prama: const char * oesm_filepath -- input oesm file path;
 @prama: const char * dem_filepath -- input dem file path;
 @prama: vector<z_Point>&z_point_vec -- vec to record result 3d points;
 @prama: bool is_ground -- check if the plane is a single ground;
 */
void fetch_td_points(vector<cv::Point> &points_vec,const char * dom_filepath, const char * oesm_filepath,const char * dem_filepath,vector<z_Point>&z_point_vec,bool is_ground);

//get the normal vector with three points in plane
/*
 @prama: z_Point &v1 v2 v3 -- three points construct a plane
 @return: the normal vector
 */
z_Point Cal_Normal_3D(z_Point &v1,z_Point &v2,z_Point &v3);


//reset the elevation of groof and floor to the average value and get the normalization vector of each plane
/*
 @pram: vector<z_Point>&z_point_vec -- the vec records the 3d points of building corner points in order (groof -> floor)
 @prama: vector<z_Point>&normal_est_vec -- the vec records the normal vector of each planes
 @prama: vector<z_Plane>&z_planes_vec -- the vec record the idex of each plane(normal vector index  and   corner points index)
 @prama: int former_point_count,int former_normal_count -- there are many buildings ,add the former count before cal the index
 @prama: bool is_ground -- check if the plane is a single ground;
 */
void resize_ele_and_get_normal_vec(vector<z_Point>&z_point_vec, vector<z_Point>&normal_est_vec,vector<z_Plane>&z_planes_vec,int former_point_count,int former_normal_count,bool is_ground);




//create the obj file
/*
 @prama: const char * obj_file_path -- out file path
 @pram: vector<<z_Point>>&z_point_vec -- the v_vec records the 3d points of building corner points in order (groof -> floor)
 @prama: vector<<z_Point>>&normal_est_vec -- the v_vec records the normal vector of each planes
 @prama: vector<z_Plane>&z_planes_vec -- the vec record the idex of each plane(normal vector index  and   corner points index) 
 @prama: cv::Size size -- the input dom size
 */
void write_to_objfile(const char * obj_file_path, vector<vector<z_Point>>&z_point_v_vec, vector<vector<z_Point>>&normal_est_v_vec,vector<z_Plane>&z_planes_vec,cv::Size size);
