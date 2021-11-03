#include "eb_images_detect.hpp"
#include "eb_common_defines.hpp"
using namespace cv;

//GaussianBlur and ctgray the input img
/*
 @prama: Mat &img -- input mat
 @prama: Mat &gray -- output mat after  GaussianBlur and ctgray
 */
static int gaussianBlur_and_ctgray(cv::Mat &img , cv::Mat &gray){
    
    cv::Mat gass_img;
    cv::GaussianBlur(img,gass_img,cv::Size(3,3),0);
    cv::cvtColor(gass_img,gray,COLOR_RGB2GRAY);
    return 0;
}

//Mat show method 
/*
 @prama const char* name -- window name 
 @prama Mat &mat -- input mat be shown later
 @prama int size -- window size
 */
void mat_show(const char* name , Mat &mat, int size){
    namedWindow(name,0);
    resizeWindow(name,size,size);
    imshow(name, mat);
    waitKey(0);
}


void rotate(cv::Mat &src, cv::Mat &dst,double angle,cv::Point2f center)
{
    cv::Mat M = cv::getRotationMatrix2D(center,angle,1);
    cv::warpAffine(src,dst,M,cv::Size(src.cols,src.rows));
}

//find the boundaries in input mat with canny methods
/*
@prama: Mat &img -- input mat
@prama: Mat &sobel_out -- output mat after convolution with canny kernals 
 */
int canny_find_boundary(cv::Mat &img , cv::Mat &canny_out ,eb_config_t *eb_config_ptr){
    
    
    Mat gray;
    gaussianBlur_and_ctgray(img, gray);
	Canny(gray, canny_out, eb_config_ptr->canny_thd_1, eb_config_ptr->canny_thd_2, 3, true);
	mat_show("canny_out",canny_out,MAT_SIZE);
    return 0;
} 


//hough methd to find lines(not all boundaries) in boundaries given the canny or sobel way
/*
 @prama: vector<Vec4f> &lines -- vector to record each line's begin point and end point
 @prama: Mat &img -- the original input mat read from image file waiting for Houghdraw the lines
 @prama; Mat &img_out -- the binaryzation mat contains the boundaries and it will be the input for hough method
 */
int hough_find_lins(eb_lines_t *hough_lines,cv::Mat &canny_image, cv::Mat &hough_img,eb_config_t *eb_config_ptr){

    

	//'1'生成极坐标时候的像素扫描步长，'CV_PI/180'生成极坐标时候的角度步长，'10'最小直线长度，'0'最大间隔（能构成一条直线） 
    static std::vector<Vec4f> tmp_out_lines;
	HoughLinesP(canny_image,
                tmp_out_lines,
                eb_config_ptr->hough_thd_1,
                eb_config_ptr->hough_thd_2,
                eb_config_ptr->hough_thd_3,
                eb_config_ptr->hough_thd_4,
                eb_config_ptr->hough_thd_5);
	Scalar color = cv::Scalar(255);
    hough_lines->line_size = tmp_out_lines.size();
    hough_lines->lines = (eb_line_t *)malloc(sizeof(eb_line_t) * hough_lines->line_size);

	for (size_t i = 0; i < tmp_out_lines.size(); i++)
	{
        hough_lines->lines[i].point_beg.dx =  tmp_out_lines[i][0];
        hough_lines->lines[i].point_beg.dy =  tmp_out_lines[i][1];
        hough_lines->lines[i].point_end.dx =  tmp_out_lines[i][2];
        hough_lines->lines[i].point_end.dy =  tmp_out_lines[i][3];                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
	    line(hough_img,Point(tmp_out_lines[i][0],tmp_out_lines[i][1]),Point(tmp_out_lines[i][2],tmp_out_lines[i][3]),color,1,LINE_AA);
	}
	mat_show("hough_img",hough_img,MAT_SIZE);

    return 0;
}


void init_mats(eb_mats_t *mats,const char *image_path)
{

    mats->input_image = cv::imread(image_path);
    mats->image_width = mats->input_image.cols;
    mats->image_height = mats->input_image.rows;
    mats->hough_image = Mat(mats->image_height,mats->image_width,CV_8UC1,Scalar(0));
    mats->boundary_image = Mat(mats->image_height,mats->image_width,CV_8UC1,Scalar(0));
    mats->buffer_image = Mat(mats->image_height,mats->image_width,CV_8UC1,Scalar(0));
    mats->buf_filter_image = Mat(mats->image_height,mats->image_width,CV_8UC1,Scalar(0));
    mats->adsorb_filter_image = Mat(mats->image_height,mats->image_width,CV_8UC3,Scalar(255,255,255));
    mats->adsorb_update_image = Mat(mats->image_height,mats->image_width,CV_8UC3,Scalar(255,255,255));
    mats->simplify_lines_image = Mat(mats->image_height,mats->image_width,CV_8UC3,Scalar(255,255,255));
    mats->reset_lines_image = Mat(mats->image_height,mats->image_width,CV_8UC3,Scalar(255,255,255));
    mats->close_lines_image = Mat(mats->image_height,mats->image_width,CV_8UC3,Scalar(255,255,255));
    mats->roofs_image = Mat(mats->image_height,mats->image_width,CV_8UC3,Scalar(255,255,255));
    mats->roofs_lidar_image = Mat(mats->image_height,mats->image_width,CV_8UC1,Scalar(255));
    

    EB_LOG("[CV::INFO] mats init completed！\n");
}

void init_features(eb_features_t *features)
{
    features->boundary_points.point_size = 0;
    features->buffer_filter_lines.line_size = 0;
    features->delau_boundary_pois.point_size = 0;
    features->hough_lines.line_size = 0;
    EB_LOG("[EB::INFO] features size init completed！\n");
}

void set_buffer_mat(cv::Mat &boundary_mat,cv::Mat &buffer_mat ,eb_points_t *delau, eb_config_t *config_ptr)
{
    Mat dilateElement = getStructuringElement(MORPH_RECT, 
                                            Size(config_ptr->buffer_matrix_size, 
                                                config_ptr->buffer_matrix_size));

    for(int i = 0 ; i<delau->point_size; i++)
    {
        int next = i== delau->point_size - 1? 0:i+1;
        line(buffer_mat,
            cv::Point(delau->points[i].dx,delau->points[i].dy),
            cv::Point(delau->points[next].dx,delau->points[next].dy),
            cv::Scalar(255),
            1,
            CV_AA);

        circle(boundary_mat,
                cv::Point(delau->points[i].dx,delau->points[i].dy),
                2,
                cv::Scalar(255),
                2,
                CV_AA);
    }

    dilate(buffer_mat, buffer_mat, dilateElement);

    mat_show("boundary_image",boundary_mat,MAT_SIZE);
    mat_show("boundary_buffer",buffer_mat,MAT_SIZE);
    
}



