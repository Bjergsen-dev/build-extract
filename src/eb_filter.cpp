#include "eb_filter.hpp"
#include "eb_config.hpp"
#include "eb_common_defines.hpp"
#include "eb_images_detect.hpp"
#include <math.h>



//decide which line is within the pcd boundary area(extended by blur)
/*
 prama: Mat &pcd_boudary_mat -- input mat describe the boundary points area with OpenCV method(instead of pcd method)
 prama: Vec4f &vec -- input line
  prama: float prameter -- prameter decide in or out
 return: bool decide_line_within_pcdBoudaryArea_or_not -- true:line within boundary area false: without
 */
static bool decide_line_within_pcdBoudaryArea_or_not(cv::Mat &pcd_boudary_mat,eb_line_t line,float prameter){
    uchar gray_0 = pcd_boudary_mat.at<uchar>(line.point_beg.dy,line.point_beg.dx);
    uchar gray_4 = pcd_boudary_mat.at<uchar>(line.point_end.dy,line.point_end.dx);
    uchar gray_2 = pcd_boudary_mat.at<uchar>((line.point_beg.dy+line.point_end.dy)/2,(line.point_beg.dx+ line.point_end.dx)/2);
    uchar gray_1 = pcd_boudary_mat.at<uchar>((3*line.point_beg.dy+line.point_end.dy)/4,(3*line.point_beg.dx+ line.point_end.dx)/4);
    uchar gray_3 = pcd_boudary_mat.at<uchar>((line.point_beg.dy+3*line.point_end.dy)/4,(line.point_beg.dx+3*line.point_end.dx)/4);
    
    uchar gray_0_m_1 = pcd_boudary_mat.at<uchar>((7*line.point_beg.dy+line.point_end.dy)/8,(7*line.point_beg.dx+ line.point_end.dx)/8);
    uchar gray_1_m_2 = pcd_boudary_mat.at<uchar>((5*line.point_beg.dy+3*line.point_end.dy)/8,(5*line.point_beg.dx+ 3*line.point_end.dx)/8);
    uchar gray_2_m_3 = pcd_boudary_mat.at<uchar>((3*line.point_beg.dy+5*line.point_end.dy)/8,(3*line.point_beg.dx+ 5*line.point_end.dx)/8);
    uchar gray_3_m_4 = pcd_boudary_mat.at<uchar>((1*line.point_beg.dy+7*line.point_end.dy)/8,(1*line.point_beg.dx+ 7*line.point_end.dx)/8);

    
    bool res = (gray_0+gray_1+gray_2+gray_3+gray_4+gray_0_m_1+gray_1_m_2+gray_2_m_3+gray_3_m_4) > (255-1)*(prameter*9);
    return res;
    
}


void buffer_filter(eb_features_t * eb_futures_ptr, eb_config_t *eb_config_ptr)
{
    eb_line_t *tmp_lines = (eb_line_t *)malloc(sizeof(eb_line_t) * eb_futures_ptr->hough_lines.line_size);
    for(int i = 0; i < eb_futures_ptr->hough_lines.line_size; i++)
    {
        if(decide_line_within_pcdBoudaryArea_or_not(
            eb_futures_ptr->eb_mats.buffer_image,
            eb_futures_ptr->hough_lines.lines[i],
            eb_config_ptr->buffer_filter_f
        ))
        {
            eb_futures_ptr->buffer_filter_lines.line_size++;
            tmp_lines[i] = eb_futures_ptr->hough_lines.lines[i];

            cv::line(eb_futures_ptr->eb_mats.buf_filter_image,
                    cv::Point(tmp_lines[i].point_beg.dx,tmp_lines[i].point_beg.dy),
                    cv::Point(tmp_lines[i].point_end.dx,tmp_lines[i].point_end.dy),
                    cv::Scalar(255),
                    1,
                    CV_AA);
        }
    }
    eb_futures_ptr->buffer_filter_lines.lines = (eb_line_t *) malloc(sizeof(eb_line_t) *
                                                                     eb_futures_ptr->buffer_filter_lines.line_size);
    memcpy(eb_futures_ptr->buffer_filter_lines.lines,
            tmp_lines,
            eb_futures_ptr->buffer_filter_lines.line_size*sizeof(eb_line_t));

    free(tmp_lines);
    tmp_lines = NULL;

    mat_show("buffer_filter_image",
            eb_futures_ptr->eb_mats.buf_filter_image,
            MAT_SIZE);

    EB_LOG("[EB::INFO] buffer filter completed!\n");
}

static void adsorbent_dis(eb_line_t *line, eb_point_t *point,int point_index, double adsorb_thresd)
{
    double a = sqrt(pow((line->point_beg.dx - line->point_end.dx),2) 
                    + pow((line->point_beg.dy - line->point_end.dy),2));

    double b = sqrt(pow((line->point_beg.dx - point->dx),2) 
                    + pow((line->point_beg.dy - point->dy),2));

    double c = sqrt(pow((point->dx - line->point_end.dx),2) 
                    + pow((point->dy - line->point_end.dy),2));

    if(a+b > c+MIN_DOUBLE && a+c > b +MIN_DOUBLE && b+c > a + MIN_DOUBLE)
    {
        double cos_b = (c*c + a*a - b*b)/(2*c*a+MIN_DOUBLE);
        double cos_c = (b*b + a*a - c*c)/(2*b*a+MIN_DOUBLE);

        if(cos_b < 0 || cos_c < 0)
        {
            return ;
        }

        line->adsorbent.adsorbent_dis[line->adsorbent.adsorbent_num] = sqrt(1-cos_c*cos_c)*b;
        line->adsorbent.adsorbent_foot[line->adsorbent.adsorbent_num] = line->point_beg;

        double scale = cos_c*b / a;
        line->adsorbent.adsorbent_foot[line->adsorbent.adsorbent_num].dx += 
                            (line->point_end.dx - line->point_beg.dx) * scale;

        line->adsorbent.adsorbent_foot[line->adsorbent.adsorbent_num].dy += 
                            (line->point_end.dy - line->point_beg.dy) * scale;
        line->adsorbent.adsorbent_index[line->adsorbent.adsorbent_num] = point_index;
        line->adsorbent.adsorbent_num++;
        
        return;
    }
    else
    {
        if(a > b && a > c)
        {
            return ;
        }

        return ;
    }


    
}

void adsorbent_filter(eb_features_t * eb_futures_ptr, eb_config_t *eb_config_ptr)
{

    for(int i = 0; i<eb_futures_ptr->buffer_filter_lines.line_size; i++ )
    {
        eb_futures_ptr->buffer_filter_lines.lines[i].adsorbent.adsorbent_index = 
        (int *)malloc(sizeof(int) * eb_futures_ptr->delau_boundary_pois.point_size);

        eb_futures_ptr->buffer_filter_lines.lines[i].adsorbent.adsorbent_foot = 
        (eb_point_t *)malloc(sizeof(eb_point_t) * eb_futures_ptr->delau_boundary_pois.point_size);

        eb_futures_ptr->buffer_filter_lines.lines[i].adsorbent.adsorbent_dis = 
        (double *)malloc(sizeof(double) * eb_futures_ptr->delau_boundary_pois.point_size);


        for(int j = 0; j < eb_futures_ptr->delau_boundary_pois.point_size; j++)
        {
            adsorbent_dis(&eb_futures_ptr->buffer_filter_lines.lines[i],
                            &eb_futures_ptr->delau_boundary_pois.points[j],
                            j,
                            eb_config_ptr->min_adsorb_dis);
        }

        if(eb_futures_ptr->buffer_filter_lines.lines[i].adsorbent.adsorbent_num > 
            eb_config_ptr->min_adsorb_num)
            {
                for(int k = 0; k < eb_futures_ptr->buffer_filter_lines.lines[i].adsorbent.adsorbent_num; k++)
                {
                    cv::circle(eb_futures_ptr->eb_mats.adsorb_filter_image,
                            cv::Point(eb_futures_ptr->buffer_filter_lines.lines[i].adsorbent.adsorbent_foot[k].dx,
                                        eb_futures_ptr->buffer_filter_lines.lines[i].adsorbent.adsorbent_foot[k].dy),
                                        2,
                                        cv::Scalar(255),
                                        1,
                                        CV_AA);
                }
                
            }
    }

    mat_show("adsorbent_image",eb_futures_ptr->eb_mats.adsorb_filter_image,MAT_SIZE);
}

