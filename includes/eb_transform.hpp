#ifndef EB_TRANSFORM_H
#define EB_TRANSFORM_H

#include"gdal_priv.h"
#include "ogrsf_frmts.h"
#include <pcl/point_cloud.h> 
#include <pcl/point_types.h>
#include "eb_common_defines.hpp"
#include "eb_features.hpp"
#include "eb_config.hpp"
#include "opencv2/opencv.hpp"

#ifdef MID_RESULT
void trans_dx_dy_to_all(cv::Point poi,eb_config_t *config_ptr,double *res);
#endif
//get the row,column subnum in tiff file from spacial loacation geoX geoY
/*
 @prama: double *trans -- array record thr trans data
 @prama: double geoX -- input the spacial location_X
 @prama: double geoY -- input the spacial location_Y
 @prama: int type -- decide the return type 2--row other--column
 return : the row or column for input spacial XY
 */
float get_row_column_frm_geoX_geoY(double *trans,double geoX, double geoY,int type);

//get the geoX,geoY subnum in tiff file from row column
/*
 @prama: double *trans -- array record thr trans data
 @prama: double x -- input the clomn
 @prama: double y -- input the row
 @prama: int type -- decide the return type 2--geo_X other--geo_Y
 return : the spacial XY for input row or column
 */
void get_geoX_geoY_frm_row_column(double*trans, double x,double y,double *geo_X, double *geo_Y);

float get_elevation_frm_geo_XY(GDALDataset* poDataset,double geo_x, double geo_y, double *trans);

void open_Gdal(GDALDataset *poDataset,char *file_path_name);

//get the elevation of raster tif in paticular row_column
/*
 @prama: double *trans -- array record thr trans data
 @prama: double x -- input the clomn
 @prama: double y -- input the row
 return : the elevation for input row or column
 */
float get_elevation_frm_row_column(GDALDataset* poDataset, float* inBuf,int x, int y);
void save_Trans_to_File(eb_config_t * config_ptr);

//get trans of tiff file
/*
 @prama: const char * file_path_name -- file path to load the tif
 @prama: double* trans -- array to record the trans
 */
void getTrans_of_TiffFile(const char * file_path_name, double* trans);

void shp_reader(char * shp_file,std::vector<std::vector<cv::Vec2f>> &polygons_vec);
void pcd_to_mat(pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_cloud,double *trans,eb_config_t *eb_config_ptr,eb_points_t  *boundary_pois);
#if 0
void lidar_planar_to_image(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,cv::Mat &image,double *trans,double transform_x,double transform_y);
#endif
#endif