#ifndef EB_TRANSFORM_H
#define EB_TRANSFORM_H

#include"gdal_priv.h"
#include "ogrsf_frmts.h"
#include <pcl/point_cloud.h> 
#include <pcl/point_types.h>
#include "eb_common_defines.hpp"
#include "eb_features.hpp"
#include "eb_config.hpp"


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

//get the elevation of raster tif in paticular row_column
/*
 @prama: double *trans -- array record thr trans data
 @prama: double x -- input the clomn
 @prama: double y -- input the row
 return : the elevation for input row or column
 */
float get_elevation_frm_row_column(GDALDataset* poDataset, float* inBuf,int x, int y);

//get trans of tiff file
/*
 @prama: const char * file_path_name -- file path to load the tif
 @prama: double* trans -- array to record the trans
 */
void getTrans_of_TiffFile(const char * file_path_name, double* trans);


void pcd_to_mat(pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_cloud,double *trans,eb_config_t *eb_config_ptr,eb_points_t  *boundary_pois);
#if 0
void lidar_planar_to_image(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,cv::Mat &image,double *trans,double transform_x,double transform_y);
#endif
#endif