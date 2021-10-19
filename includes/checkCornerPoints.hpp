/**************************************************************************

Copyright:Best_Intelligence in LiesMars

Author: ZZC_Bjergsen

Date:2020-08-18

Description:Provide  functions  to find the corner points of build groof

**************************************************************************/
#pragma once

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/core/core.hpp"
#include "gdal_methods.hpp"
#include "pcl_methods.hpp"
#include <jsoncpp/json/json.h>
#include <stack>
#include <cmath>
#include <deque>
#include <algorithm>
#include <iterator>
#include <array>
#include <Eigen/Core>
#include <list>
#include <fstream>
#include <math.h>

//#include "edge.hpp"

#include <vector>
using namespace cv; 



//dictance between two points
/*
 prama: Point p_point1-- input point 1
 prama: Point p_point2-- input point 2
 return: float distance -- distance between two points 
 */
float distance_btw_two_points(Point p_point1, Point p_point2);

//decide two lines 's similarity
/*
 prama: Vec4f &vec1 -- input line 1
 prama: Vec4f &vec2 -- input line 2
 prama: float distance_limit -- the limit to describe the distance btween two lines
 prama: float line_cos_limit -- the limit to describe the angle between two lines
 prama: float line_b_limit -- the limit to describe the nodal increment of two lines
 return: bool decide_two_lines_similarity -- true:is_similary false:not_similary
 */
bool decide_two_lines_similarity(Vec4f &vec1, Vec4f &vec2,float distance_limit,float line_cos_limit,float line_b_limit);

//get the similary lines together(combine the similary lines)
/*
 prama: std::vector<Vec4f> &similar_lines -- the input similary lines vector
 return: the line resulting in input lines combined
 */
Vec4f  get_lines_togethor(std::vector<Vec4f> &similar_lines);

//decide which line is within the pcd boundary area(extended by blur)
/*
 prama: Mat &pcd_boudary_mat -- input mat describe the boundary points area with OpenCV method(instead of pcd method)
 prama: Vec4f &vec -- input line
 prama: float prameter -- prameter decide in or out
 return: bool decide_line_within_pcdBoudaryArea_or_not -- true:line within boundary area false: without
 */
bool decide_line_within_pcdBoudaryArea_or_not(Mat &pcd_boudary_mat,Vec4f &vec,float prameter);

//pcd to mat chansform
/*
@prama: vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &boundary_cloud_vec -- the vector record bundary points in pcd
@prama: double *trans -- localization transform prameters of tiff file (used in gdal)
@prama: float translation_x,float translation_y -- the translation of x,y between pcd and tif file
 */
template<class T>
void pcd_to_mat(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &boundary_cloud_vec,double *trans,float translation_x,float translation_y,vector<vector<T>> &contours){
    
    int i = 0;
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_cloud : boundary_cloud_vec){
        vector<T> point_vec;
        for(pcl::PointXYZ point : boundary_cloud->points){
            float dcol = get_row_column_frm_geoX_geoY(trans,(point.x + translation_x),(point.y + translation_y),1);
            float drow = get_row_column_frm_geoX_geoY(trans,(point.x + translation_x),(point.y + translation_y),2);
            point_vec.push_back(T(dcol,drow));
        }
        i++;
        contours.push_back(point_vec);
    }
    
    
}

//mat to pcd (to use the pcd methods)
/*
 @prama: Mat &inputmat -- input mat
 @prama: pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud -- output point cloud
 @prama: int multiple_num -- zoom the z value(defualt 1)
 */
void mat_to_pcd(Mat &inputmat,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,int multiple_num);

//GaussianBlur and ctgray the input img
/*
 @prama: Mat &img -- input mat
 @prama: Mat &gray -- output mat after  GaussianBlur and ctgray
 */
int gaussianBlur_and_ctgray(Mat &img , Mat &gray);

//find the boundary in mat with sobel kernals in 8 directions
/*
 @prama: Mat &img -- input mat
 @prama: Mat &sobel_out -- output mat after convolution with sobel kernals 
 */
int sobel_find_boundary(Mat &img, Mat &sobel_out);

//find the boundaries in input mat with canny methods
/*
@prama: Mat &img -- input mat
@prama: Mat &sobel_out -- output mat after convolution with canny kernals 
 */
int canny_find_boundary(Mat &img , Mat &canny_out );

 
//get each boundary in img_out after canny or sobel convolution and regular the boundary  with DP method
/*
@prama: Mat &img -- the original input mat read from image file waiting for DPdraw the lines
@prama: vector<vector<Point>> &contours -- vector to record the contours of each boundary
@prama: int level -------dp level
 */
int dp_find_boundary_keypoints(vector<Point> &contours,int level, vector<Vec4f> &res_vec);

//hough methd to find lines(not all boundaries) in boundaries given the canny or sobel way
/*
 @prama: vector<Vec4f> &lines -- vector to record each line's begin point and end point
 @prama: Mat &img -- the original input mat read from image file waiting for Houghdraw the lines
 @prama; Mat &img_out -- the binaryzation mat contains the boundaries and it will be the input for hough method
 */
int hough_find_lins(vector<Vec4f> &lines,Mat &img, Mat &img_out);

//find the similar lines in input lines vector and group them
/*
 prama: float length_limit_each_line -- min length limit of lines, lines shorter than it will be ignored
 prama: float distance_limit -- the limit to describe the distance btween two lines
 prama: float line_cos_limit -- the limit to describe the angle between two lines
 prama: float line_b_limit -- the limit to describe the nodal increment of two lines
 prama: vector<Vec4f> &within_lines -- input lines vec
 prama: vector<vector<Vec4f>> &lines_v_vec -- output lines vec's vec(similar lines in groups)
 */
void find_the_similar_lines_in_lines_vec(float length_limit_each_line,float length_limit_between_two_lines,float lines_cos_limit,float lines_b_limit,vector<Vec4f> &within_lines,vector<vector<Vec4f>> &lines_v_vec);

//quick sort 
/*
 prama: vector<int> &index -- input index array record the lines_vec's line index
 prama: vector<Vec4f> &lines -- input lines vec
 prama: int start, int last -- quick sort pramaters
  prama: vector<vector<Point>> &sorted_lines_foot -- input vector record the foots of each line
 */
template <class T,class H>
void quick_sort(vector<H> index,vector<T> &lines,int start, int last)
  {
      
      
      for(int i = 0; i < index.size() - 1; i++)
    {
        for(int j = 0; j < index.size() - i -1; j++)
        {
            if(index[j] > index[j+1]){
                 H temp = index[j];
                T tem_vec = lines[j];
                
                index[j] = index[j+1];
                index[j+1] = temp;
                lines[j] = lines[j+1];
                lines[j+1] = tem_vec;
            }
                
        }
    }
      
      
    
 }

 //get the cross over point of two lines
 /*
  prama: Vec4f vec1 -- input line 1
  prama: Vec4f vec2 -- input line 2
  return: the crossover point 
  */
 
Point get_common_point_of_two_lines(Vec4f vec1,Vec4f vec2);


//order the line's begin and end point with foot_vec_points's order
void order_line_begin_end(Vec4f &line_vec,Point begin_foot,Point end_foot);

//Mat show method 
/*
 @prama const char* name -- window name 
 @prama Mat &mat -- input mat be shown later
 @prama int size -- window size
 */
void mat_show(const char* name , Mat &mat, int size);

//release the vector room
template<class T>
void release_vector(vector<T> &vec){
    
    vector<T>().swap(vec);
    
}


//get the longest line index in a lines vec
/*
 @prama: std::vector<Vec4f> lines_vec : input lines vector
 @return: longest line's index in vector
 */
int get_longest_line_index_in_vec(std::vector<Vec4f> lines_vec);

//get the cos angle of two lines 
/*
 @prama: Vec4f line1 : input line 1
 @prama: Vec4f line2 : input line 2
 @return: cos angle value btw line1 and line2
 */
float get_cos_btw_two_lines(Vec4f line1,Vec4f line2);


//read json
/*
 @prama:std::string json_file_path -- json file path
 @prama:std::vector<std::string> &str_vec -- json string vec
 */

void readFileJson(std::string json_file_path, std::vector<std::string> &str_vec);

//get the wrong direction and short lines indexes in a lines vec
/*
 @prama: vector<Vec4f> lines_vec -- input lines vec
 @prama: float k_difrence -- a line regarded as wrong line when it's k value is diffrent more than k-limit with other lines
 @prama: float length_limit -- a wrong line can't longer than length-limit
 @prama: vector<int> &delete_index_vec -- vector record the wrong lines' indexes
 */
void get_wrong_lines_indexes(vector<Vec4f> lines_vec,float k_difrence,float length_limit,vector<int> &delete_index_vec);


//according to the wrong lines' indexes , remove the wrong lines and wrong lines' information
template<class T>
void remove_wrong_lines(vector<int> wrong_indexes_vec, vector<T> &lines_vec){
    int count = 0;
    for(typename  vector<T>::iterator it = lines_vec.begin(); it!=lines_vec.end();it++){
        
       for(int index : wrong_indexes_vec){
           if(index == count){
               it = lines_vec.erase(it);
               it--;
               break;
        }
    }
    
    count++;
  }
}
