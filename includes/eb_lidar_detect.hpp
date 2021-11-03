#ifndef EB_LIDAR_DETECT_H
#define EB_LIDAR_DETECT_H
#pragma once
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
#include <pcl/common/common.h>
#include "eb_config.hpp"
using namespace pcl;

void filter_pcl_with_z(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,eb_config_t *eb_config_ptr);


//show cloud in PCLviwer
/*
 @pram visualization::PCLVisualizer &viewer --PCLviewer for output cloud
 @pram std::string ss --string description of  each output cloud in viewer
 @pram PointCloud<pcl::PointXYZ>::Ptr &cloud -- cloud should be showed in viwer
 */
void D3_view(visualization::PCLVisualizer &viewer, std::vector<PointCloud<PointXYZ>::Ptr> &cloud_vec,PointCloud<PointXYZ>::Ptr &all_cloud);


//get the boundarys of palanars
/*
 @pram std::vector<PointCloud<PointXYZ>::Ptr> &planars_cloud_vec --input cloud vec
 @pram std::vector<PointCloud<PointXYZ>::Ptr> &boundaries_cloud_vec --output boundarycloud vec
 */
int estimateBorders(std::vector<PointCloud<PointXYZ>::Ptr> &planars_cloud_vec,std::vector<PointCloud<PointXYZ>::Ptr> &boundaries_cloud_vec,eb_config_t *eb_config_ptr);


//get the cloud points Resolution
/*
 @pram const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud -- input cloud
 @pram int k -- neighbor radius
 */
float computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, int k);

void get_min_max_z(double *res,const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);

int region_grow (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,std::vector<PointCloud<PointXYZ>::Ptr> &res_vec);
#endif