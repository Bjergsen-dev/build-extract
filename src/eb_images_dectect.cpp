#include "eb_images_detect.hpp"
#include "eb_common_defines.hpp"
#include "eb_transform.hpp"
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
void mat_show(const char* name , cv::Mat &mat, int size,eb_config_t *config_ptr){
    
    char tmp[256];
    snprintf(tmp,strlen(config_ptr->file_config.out_path)+1,config_ptr->file_config.out_path);
    snprintf(tmp+strlen(tmp),strlen(name)+6,"/%s.bmp",name);
    imwrite(tmp,mat);

    #if 0
    #ifdef MID_RESULT
    #else
    namedWindow(name,0);
    resizeWindow(name,size,size);
    imshow(name, mat);
    waitKey(0);
    #endif
    #endif
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
    cv::threshold(canny_out, canny_out, 1, 255,THRESH_BINARY);
	mat_show("canny_out",canny_out,MAT_SIZE,eb_config_ptr);
    return 0;
} 
#if 0
src = imread("Lenna.jpg");
	GaussianBlur(src, src, Size(3, 3), 0, 0, BORDER_DEFAULT);	//先通过高斯模糊去噪声
	cvtColor(src, src_gray, CV_RGB2GRAY);
	namedWindow(window_name, CV_WINDOW_AUTOSIZE);
 
	Mat dst, abs_dst;
	Laplacian(src_gray, dst, CV_16S, kernel_size);	//通过拉普拉斯算子做边缘检测
	convertScaleAbs(dst, abs_dst);
 
	imshow(window_name, abs_dst);
————————————————
版权声明：本文为CSDN博主「陈胃痛」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/primetong/article/details/79589620
#endif

int LoG_find_boundary(cv::Mat &img , cv::Mat &canny_out ,eb_config_t *eb_config_ptr)
{
    Mat gray;
    //gaussianBlur_and_ctgray(img, gray);
    cv::GaussianBlur( img, img, Size(3,3), 0, 0, BORDER_DEFAULT );
    cv::cvtColor( img, gray, CV_RGB2GRAY );
    Laplacian(gray, canny_out, CV_16S, 3);	//通过拉普拉斯算子做边缘检测
	convertScaleAbs(canny_out, canny_out);
    cv::threshold(canny_out, canny_out, 35, 255,THRESH_BINARY);
    mat_show("LoG_out",canny_out,MAT_SIZE,eb_config_ptr);
    return 0;
}

int sobel_find_boundary(cv::Mat &img, cv::Mat &sobel_out,eb_config_t *eb_config_ptr){
    
    Mat gray;
    gaussianBlur_and_ctgray(img, gray);
    
    Mat gray_out_1;
    Mat kernel_1 = (Mat_<int>(5,5)<<0,0,0,0,0,-1,-2,-4,-2,-1,0,0,0,0,0,1,2,4,2,1,0,0,0,0,0);
    filter2D(gray,gray_out_1,-1,kernel_1,Point(-1,-1),0,0);
    
    Mat gray_out_2;
    Mat kernel_2 = (Mat_<int>(5,5)<<0,0,0,0,0,0,-2,-4,-2,0,-1,-4,0,4,1,0,2,4,2,0,0,0,0,0,0);
    filter2D(gray,gray_out_2,-1,kernel_2,Point(-1,-1),0,0);
    
    Mat gray_out_3;
    Mat kernel_3 = (Mat_<int>(5,5)<<0,0,0,-1,0,0,-2,-4,0,1,0,-4,0,4,0,-1,0,4,2,0,0,1,0,0,0);
    filter2D(gray,gray_out_3,-1,kernel_3,Point(-1,-1),0,0);
    
    Mat gray_out_4;
    Mat kernel_4 = (Mat_<int>(5,5)<<0,0,-1,0,0,0,-2,-4,2,0,0,-4,0,4,0,0,-2,4,2,0,0,0,1,0,0);
    filter2D(gray,gray_out_4,-1,kernel_4,Point(-1,-1),0,0);
    
    Mat gray_out_5;
    Mat kernel_5 = (Mat_<int>(5,5)<<0,-1,0,1,0,0,-2,0,2,0,0,-4,0,4,0,0,-2,0,2,0,0,-1,0,1,0);
    filter2D(gray,gray_out_5,-1,kernel_5,Point(-1,-1),0,0);
    
    Mat gray_out_6;
    Mat kernel_6 = (Mat_<int>(5,5)<<0,0,1,0,0,0,-2,4,2,0,0,-4,0,4,0,0,-2,-4,2,0,0,0,-1,0,0);
    filter2D(gray,gray_out_6,-1,kernel_6,Point(-1,-1),0,0);
    
    Mat gray_out_7;
    Mat kernel_7 = (Mat_<int>(5,5)<<0,1,0,0,0,-1,0,4,2,0,0,-4,0,4,0,0,-2,-4,0,1,0,0,0,-1,0);
    filter2D(gray,gray_out_7,-1,kernel_7,Point(-1,-1),0,0);
    
    Mat gray_out_8;
    Mat kernel_8 = (Mat_<int>(5,5)<<0,0,0,0,0,0,2,4,2,0,-1,-4,0,4,1,0,-2,-4,-2,0,0,0,0,0,0);
    filter2D(gray,gray_out_8,-1,kernel_8,Point(-1,-1),0,0);


    Mat res_1_2;
    cv::addWeighted(gray_out_1,0.5,gray_out_2,0.5,0,res_1_2);
    Mat res_3_4;
    cv::addWeighted(gray_out_3,0.5,gray_out_4,0.5,0,res_3_4);
    Mat res_5_6;
    cv::addWeighted(gray_out_5,0.5,gray_out_6,0.5,0,res_5_6);
    Mat res_7_8;
    cv::addWeighted(gray_out_7,0.5,gray_out_8,0.5,0,res_7_8);
    
    Mat res_1_2_3_4;
    cv::addWeighted(res_1_2,0.5,res_3_4,0.5,0,res_1_2_3_4);
    Mat res_5_6_7_8;
    cv::addWeighted(res_5_6,0.5,res_7_8,0.5,0,res_5_6_7_8);
    

    cv::addWeighted(res_1_2_3_4,0.5,res_5_6_7_8,0.5,0,sobel_out);
    cv::threshold(sobel_out, sobel_out, eb_config_ptr->canny_thd_1, 255,THRESH_BINARY);
    
    EB_LOG("[EB_DEBUG::] eb_config_ptr->canny_thd_1 is : %lf\n",eb_config_ptr->canny_thd_1);
    mat_show("sobel_out",sobel_out,500,eb_config_ptr);
    
    return 1;

    
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
    #ifdef TEST_8_SOBEL
	mat_show("8_sobel_hough_img",hough_img,MAT_SIZE,eb_config_ptr);
    #endif

    #ifdef TEST_CANNY
	mat_show("canny_hough_img",hough_img,MAT_SIZE,eb_config_ptr);
    #endif

    #ifdef TEST_LoG
	mat_show("LoG_hough_img",hough_img,MAT_SIZE,eb_config_ptr);
    #endif

    return 0;
}


void init_mats(eb_mats_t *mats,const char *image_path,const char *origin_path)
{

    mats->origin_image = cv::imread(origin_path); 
    mats->input_image = cv::imread(image_path);
    mats->image_width = mats->input_image.cols;
    mats->image_height = mats->input_image.rows;
    mats->hough_image = Mat(mats->image_height,mats->image_width,CV_8UC1,Scalar(0));
    mats->contours_image = Mat(mats->image_height,mats->image_width,CV_8UC1,Scalar(0));
    
    #ifdef TEST_0
    mats->boundary_image = Mat(mats->image_height,mats->image_width,CV_8UC3,Scalar(255,255,255));
    mats->buffer_image = Mat(mats->image_height,mats->image_width,CV_8UC1,Scalar(255));
    #else
    mats->boundary_image = Mat(mats->image_height,mats->image_width,CV_8UC1,Scalar(0));
    mats->buffer_image = Mat(mats->image_height,mats->image_width,CV_8UC1,Scalar(0));
    #endif
    mats->buf_filter_image = Mat(mats->image_height,mats->image_width,CV_8UC1,Scalar(0));
    mats->adsorb_filter_image = Mat(mats->image_height,mats->image_width,CV_8UC3,Scalar(255,255,255));
    mats->adsorb_update_image = Mat(mats->image_height,mats->image_width,CV_8UC3,Scalar(255,255,255));
    mats->simplify_lines_image = Mat(mats->image_height,mats->image_width,CV_8UC3,Scalar(255,255,255));
    
    #ifdef TEST_2
    mats->refine_liens_image = Mat(mats->image_height,mats->image_width,CV_8UC3,Scalar(0,0,0));
    mats->reset_lines_image = Mat(mats->image_height,mats->image_width,CV_8UC3,Scalar(0,0,0));
    #else
    mats->refine_liens_image = Mat(mats->image_height,mats->image_width,CV_8UC3,Scalar(255,255,255));
    mats->reset_lines_image = Mat(mats->image_height,mats->image_width,CV_8UC3,Scalar(255,255,255));
    #endif
    #if 0
    mats->close_lines_image = Mat(mats->image_height,mats->image_width,CV_8UC3,Scalar(255,255,255));
    #endif
    mats->close_lines_image = mats->origin_image.clone();
    mats->roofs_image = Mat(mats->image_height,mats->image_width,CV_8UC3,Scalar(255,255,255));
    mats->roofs_lidar_image = Mat(mats->image_height,mats->image_width,CV_64FC1,cv::Scalar(INVALID_VAL));
    mats->org_roofs_lidar_image = Mat(mats->image_height,mats->image_width,CV_64FC1,cv::Scalar(INVALID_VAL));
    mats->ndvi_image = Mat(mats->image_height,mats->image_width,CV_8UC1,Scalar(0));
    #ifdef MID_RESULT
    mats->all_buffer_image = imread("/home/jaxzhong/Exp_data/Area3/result/Area3_all/all_buffer.bmp",0);
    if(mats->all_buffer_image.empty())
    {
        int rows = 2418;
        int cols = 1623;
        mats->all_buffer_image = Mat(rows,cols,CV_8UC1,Scalar(0));
        mats->all_buf_filter_image = Mat(rows,cols,CV_8UC1,Scalar(0));
        mats->all_simply_image = Mat(rows,cols,CV_8UC3,Scalar(255,255,255));
        mats->all_refine_image = Mat(rows,cols,CV_8UC3,Scalar(0,0,0));
    }
    else
    {
        mats->all_buf_filter_image = imread("/home/jaxzhong/Exp_data/Area3/result/Area3_all/all_buffer_filter.bmp",0);
        
        mats->all_simply_image = imread("/home/jaxzhong/Exp_data/Area3/result/Area3_all/all_simplfy.bmp",1);

        mats->all_refine_image = imread("/home/jaxzhong/Exp_data/Area3/result/Area3_all/all_refine.bmp",1);
    }

    
    #endif
    

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

void find_contours(cv::Mat &mask_img,eb_features_t *eb_feature_ptr,eb_config_t* eb_config_ptr)
{
    mat_show("mask",mask_img,MAT_SIZE,eb_config_ptr);
    std::vector< std::vector<cv::Point> > contours;
    cv::findContours(mask_img,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_NONE,cv::Point(0,0));
    cv::drawContours(eb_feature_ptr->eb_mats.contours_image, contours, -1, cv::Scalar::all(255));
    mat_show("contours",eb_feature_ptr->eb_mats.contours_image,MAT_SIZE,eb_config_ptr);
    int max_index = 0;
    int tmp_size = 0;
    for(int i = 0; i < contours.size(); i++)
    {
        max_index  = contours[i].size() > tmp_size?i:max_index;
        tmp_size  = contours[i].size() > tmp_size?contours[i].size():tmp_size;
    }
    eb_feature_ptr->delau_boundary_pois.point_size = contours[max_index].size();
    EB_LOG("[EB_INFO::] CNN_Mask Find contours sum is %d\n",contours[max_index].size());
    eb_feature_ptr->delau_boundary_pois.points = (eb_point_t*)malloc(sizeof(eb_point_t) * contours[max_index].size());
    for(int i =0; i < eb_feature_ptr->delau_boundary_pois.point_size; i++)
    {
        eb_feature_ptr->delau_boundary_pois.points[i].absorb_num = 0;
        eb_feature_ptr->delau_boundary_pois.points[i].adsorb_line_idx = -1;
        eb_feature_ptr->delau_boundary_pois.points[i].delau_pois_idx = i;
        eb_feature_ptr->delau_boundary_pois.points[i].dx = contours[max_index][i].x;
        eb_feature_ptr->delau_boundary_pois.points[i].dy = contours[max_index][i].y;
        eb_feature_ptr->delau_boundary_pois.points[i].is_adsorbed = 0;
        eb_feature_ptr->delau_boundary_pois.points[i].is_delaunay = 1;
        eb_feature_ptr->delau_boundary_pois.points[i].point_x = contours[max_index][i].x;
        eb_feature_ptr->delau_boundary_pois.points[i].point_y = contours[max_index][i].y;
        eb_feature_ptr->delau_boundary_pois.points[i].point_z = 0.;
    }
    
}

void get_NDVI_res(cv::Mat &ndvi_image,cv::Mat &input_image)
{
    printf("cols is %d, rows is %d\n",input_image.cols,input_image.rows);
    for(int i = 0; i<input_image.rows; i++)
    {
        for(int j = 0; j < input_image.cols; j++)
        {
            uchar NI_Red = input_image.at<cv::Vec3b>(i,j)[2];
            uchar Red = input_image.at<cv::Vec3b>(i,j)[1];
            uchar Green = input_image.at<cv::Vec3b>(i,j)[0];

            double NDVI = (NI_Red-Red)/(NI_Red+Red+MIN_DOUBLE);
            double val = (NDVI+1)/2 * 255.0;
            ndvi_image.at<uchar>(i,j) = (uchar)val;
        }
    }
}

void set_buffer_mat(cv::Mat &boundary_mat,cv::Mat &buffer_mat ,
                    eb_points_t *delau, 
                    eb_config_t *config_ptr
                    #ifdef MID_RESULT
                    ,
                    cv::Mat &all_buffer_mat
                    #endif
                    )
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
            #ifdef TEST_0
            cv::Scalar(0),
            #else
            cv::Scalar(255),
            #endif
            1,
            CV_AA);

        #ifdef MID_RESULT
            double trans_res1[2];
            double trans_res2[2];
            trans_dx_dy_to_all(cv::Point(delau->points[i].dx,delau->points[i].dy),
                                config_ptr,
                                trans_res1);
            trans_dx_dy_to_all(cv::Point(delau->points[next].dx,delau->points[next].dy),
                                config_ptr,
                                trans_res2);

        line(all_buffer_mat,
            cv::Point(trans_res1[0],trans_res1[1]),
            cv::Point(trans_res2[0],trans_res2[1]),
            #ifdef TEST_0
            cv::Scalar(0),
            #else
            cv::Scalar(255),
            #endif
            1,
            CV_AA);
        #endif

        circle(boundary_mat,
                cv::Point(delau->points[i].dx,delau->points[i].dy),
                2,
                #ifdef TEST_0
                cv::Scalar(0,255,0),
                #else
                cv::Scalar(255),
                #endif
                2,
                CV_AA);
    }

    #ifdef TEST_0
    erode(buffer_mat, buffer_mat, dilateElement);
    #else
    dilate(buffer_mat, buffer_mat, dilateElement);
    #endif

    mat_show("boundary_image",boundary_mat,MAT_SIZE,config_ptr);
    mat_show("boundary_buffer",buffer_mat,MAT_SIZE,config_ptr);
    
}



