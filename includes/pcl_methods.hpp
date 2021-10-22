/**************************************************************************

Copyright:Best_Intelligence in LiesMars

Author: ZZC_Bjergsen

Date:2020-08-18

Description:Provide  functions  of PCD

**************************************************************************/
#pragma once

#if 0
#include <iostream>
#include <math.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <vector>
#include <string>
#include <pcl/sample_consensus/method_types.h>   //随机参数估计方法头文件
#include <pcl/sample_consensus/model_types.h>   //模型定义头文件
#include <pcl/segmentation/sac_segmentation.h>   //基于采样一致性分割的类的头文件
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/boundary.h>
#include <math.h>
#include <boost/make_shared.hpp>
#include <pcl/point_cloud.h> 
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/covariance_sampling.h>
#include <pcl/filters/normal_space.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
using namespace pcl;

//show cloud in PCLviwer
/*
 @pram visualization::PCLVisualizer &viewer --PCLviewer for output cloud
 @pram std::string ss --string description of  each output cloud in viewer
 @pram PointCloud<pcl::PointXYZ>::Ptr &cloud -- cloud should be showed in viwer
 */
void D3_view(visualization::PCLVisualizer &viewer, std::vector<PointCloud<PointXYZ>::Ptr> &cloud_vec,PointCloud<PointXYZ>::Ptr &all_cloud);

//get the cloud points Resolution
/*
 @pram const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud -- input cloud
 @pram int k -- neighbor radius
 */
float computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, int k);

//get the boundarys of palanars
/*
 @pram std::vector<PointCloud<PointXYZ>::Ptr> &planars_cloud_vec --input cloud vec
 @pram std::vector<PointCloud<PointXYZ>::Ptr> &boundaries_cloud_vec --output boundarycloud vec
 */
int estimateBorders(std::vector<PointCloud<PointXYZ>::Ptr> &planars_cloud_vec,std::vector<PointCloud<PointXYZ>::Ptr> &boundaries_cloud_vec);


//check the plane is top or not
/*
 @pram std::vector<float> &tmp) -- vector includes the ModelCoefficients of planars
 */
int check_plane(std::vector<float> &tmp);

//get the cloud average height(z)
/*
 @pram PointCloud<PointXYZ>::Ptr &cloud -- input cloud
 */
float get_planar_height(PointCloud<PointXYZ>::Ptr &cloud);


//ransac method to get the biggest plane in input cloud
/* 
 @prama: PointCloud<PointXYZ>::Ptr &cloud -- InputCloud
 @prama: PointCloud<PointXYZ>::Ptr &output -- output cloud to record the biggest plane points
 @prama: PointCloud<PointXYZ>::Ptr &cloud_other -- out cloud to record the points except the biggest plane points
 @prama: std::vector<float> &tmp -- the parameters to record the output biggest plane
 @prama: int maxIterrations,float distanceThreshold -- ransac parameters 
 @prama: SacModel model -- ransac seg model type 
 */
void ransac(PointCloud<PointXYZ>::Ptr &cloud,int maxIterrations,float distanceThreshold,PointCloud<PointXYZ>::Ptr &output,PointCloud<PointXYZ>::Ptr &cloud_other,std::vector<float> &tmp,SacModel model);

//get the building top planars
/*
 @pram PointCloud<PointXYZ>::Ptr &cloud -- input cloud
 @pram std::vector<std::vector<float>> &Coffis -- vector records the ModelCoefficients of planars
 @pram int threshold -- min points num in cloud for planar_seg
 @pram int maxIterrations,float distanceThreshold,int minSizeofPlanar -- max Iterrations,distanceThreshold, of RANSAC 
 */
void forcircle_ransac_seg( PointCloud<PointXYZ>::Ptr &cloud, std::vector<std::vector<float>> &Coffis,int threshold,int maxIterrations,float distanceThreshold,int minSizeofPlanar, std::vector<PointCloud<PointXYZ>::Ptr> &res_vec, SacModel model);

//create pcd form tiff data(x-col y-row z-tiff data)
/*@prama: float* paf -- array to store input tiff data
 *@prama: int nImgSizeX -- the length(x direction) of pcd ready to create
 *@prama: int nImgSizeY -- the height(y direction) of pcd ready to create
 *@prama: pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud -- empty pcd to record the points will be created 
 *@param: float col -- the begin col of tiff data which decide the x of points in cloud
 *@prama: float row -- the begin row of tiff data which decide the y of points in cloud
 */
void row_col_oesmZ_2_pcd(float* paf,int nImgSizeX, int nImgSizeY,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,float col, float row);

//show cloud in PCLviwer
/*
 @pram visualization::PCLVisualizer &viewer --PCLviewer for output cloud
 @pram std::string ss --string description of  each output cloud in viewer
 @pram std::vector<std::vector<PointCloud<PointXYZ>::Ptr>> &cloud_vec -- lines of each boundary should be showed in viwer
 */
void L3_view(visualization::PCLVisualizer &viewer, std::vector<std::vector<PointCloud<PointXYZ>::Ptr>> &cloud_vec);


//filter the points in cloud with z
void filter_pcl_with_z(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,float z1,float z2);

//get the planes with region grow method 
/*
 @prama:pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud -- input cloud
 @prama:std::vector<PointCloud<PointXYZ>::Ptr> &res_vec -- vector record the planes output
 @prama:general_planar_height -- input cloud's gerneral elevation
 */
int region_grow (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,std::vector<PointCloud<PointXYZ>::Ptr> &res_vec);


#endif