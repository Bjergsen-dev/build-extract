#ifndef EB_IMAGES_DETECT_H
#define EB_IMAGES_DETECT_H


#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/core/core.hpp"
#include "eb_features.hpp"
#include "eb_config.hpp"




//Mat show method 
/*
 @prama const char* name -- window name 
 @prama Mat &mat -- input mat be shown later
 @prama int size -- window size
 */
void mat_show(const char* name , cv::Mat &mat, int size);

//find the boundaries in input mat with canny methods
/*
@prama: Mat &img -- input mat
@prama: Mat &sobel_out -- output mat after convolution with canny kernals 
 */
int canny_find_boundary(cv::Mat &img , cv::Mat &canny_out ,eb_config_t *eb_config_ptr);



//hough methd to find lines(not all boundaries) in boundaries given the canny or sobel way
/*
 @prama: vector<Vec4f> &lines -- vector to record each line's begin point and end point
 @prama: Mat &img -- the original input mat read from image file waiting for Houghdraw the lines
 @prama; Mat &img_out -- the binaryzation mat contains the boundaries and it will be the input for hough method
 */
int hough_find_lins(eb_lines_t *hough_lines,cv::Mat &canny_image, cv::Mat &hough_img,eb_config_t *eb_config_ptr);

void init_mats(eb_mats_t *mats,const char *image_path);

void set_buffer_mat(cv::Mat &buffer_mat ,eb_points_t *delau, eb_config_t *config_ptr);


void init_features(eb_features_t *features);

#endif