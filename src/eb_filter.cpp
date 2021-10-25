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
            tmp_lines[eb_futures_ptr->buffer_filter_lines.line_size] = 
                                eb_futures_ptr->hough_lines.lines[i];
            int tmp_index = eb_futures_ptr->buffer_filter_lines.line_size;
            eb_futures_ptr->buffer_filter_lines.line_size++;

            
            cv::line(eb_futures_ptr->eb_mats.buf_filter_image,
                    cv::Point(tmp_lines[tmp_index].point_beg.dx,tmp_lines[tmp_index].point_beg.dy),
                    cv::Point(tmp_lines[tmp_index].point_end.dx,tmp_lines[tmp_index].point_end.dy),
                    cv::Scalar(255),
                    1,
                    CV_AA);
            #ifdef EB_DEBUG
            EB_LOG("[EB_DEBUG::] %d-->(%f,%f)--(%f,%f)\n",
                                tmp_index,
                                tmp_lines[tmp_index].point_beg.dx,
                                tmp_lines[tmp_index].point_beg.dy,
                                tmp_lines[tmp_index].point_end.dx,
                                tmp_lines[tmp_index].point_end.dy);
            #endif
        }
    }
    #if 1
    eb_futures_ptr->buffer_filter_lines.lines = (eb_line_t *) malloc(sizeof(eb_line_t) *
                                                                       eb_futures_ptr->buffer_filter_lines.line_size);
    memcpy(eb_futures_ptr->buffer_filter_lines.lines,
            tmp_lines,
            eb_futures_ptr->buffer_filter_lines.line_size*sizeof(eb_line_t));
    free(tmp_lines);
    tmp_lines = NULL;
    #endif


    #ifdef EB_DEBUG
    EB_LOG("\n\n");
    for(int i = 0; i < eb_futures_ptr->buffer_filter_lines.line_size; i++)
    {
        EB_LOG("[EB_DEBUG::] %d-->(%f,%f)--(%f,%f)\n",
                                i,
                                eb_futures_ptr->buffer_filter_lines.lines[i].point_beg.dx,
                                eb_futures_ptr->buffer_filter_lines.lines[i].point_beg.dy,
                                eb_futures_ptr->buffer_filter_lines.lines[i].point_end.dx,
                                eb_futures_ptr->buffer_filter_lines.lines[i].point_end.dy);
    }
    #endif
    mat_show("buffer_filter_image",
            eb_futures_ptr->eb_mats.buf_filter_image,
            MAT_SIZE);

    EB_LOG("[EB::INFO] buffer filter completed，line sum is %d!\n",eb_futures_ptr->buffer_filter_lines.line_size);
}

typedef struct eb_adsorb_info
{
    /* data */
    int index;
    double scale;
    double adsorb_dis;
}eb_adsorb_info_t;


static void adsorbsent_check(eb_line_t *line,int line_index, eb_points_t *points, std::vector<eb_adsorb_info_t> &poi_index,double adsorb_thresd)
{
    for(int i  = 0; i<points->point_size; i++)
    {
        double a = sqrt(pow((line->point_beg.dx - line->point_end.dx),2) 
                    + pow((line->point_beg.dy - line->point_end.dy),2));

        double b = sqrt(pow((line->point_beg.dx - points->points[i].dx),2) 
                        + pow((line->point_beg.dy - points->points[i].dy),2));

        double c = sqrt(pow((points->points[i].dx - line->point_end.dx),2) 
                        + pow((points->points[i].dy - line->point_end.dy),2));

        if(a+b > c+MIN_DOUBLE && a+c > b +MIN_DOUBLE && b+c > a + MIN_DOUBLE)
        {
            double cos_b = (c*c + a*a - b*b)/(2*c*a+MIN_DOUBLE);
            double cos_c = (b*b + a*a - c*c)/(2*b*a+MIN_DOUBLE);

            if(cos_b < 0 || cos_c < 0)
            {
                continue ;
            }

            if(sqrt(1-cos_c*cos_c)*b > adsorb_thresd)
            {
                continue;
            }
            eb_adsorb_info_t tmp_info;
            tmp_info.index = i;
            tmp_info.scale = cos_c*b / a;
            tmp_info.adsorb_dis = sqrt(1-cos_c*cos_c)*b;
            poi_index.push_back(tmp_info);
            
        }
        else
        {
            if(a > b && a > c)
            {
                continue ;
            }
            eb_adsorb_info_t tmp_info;
            tmp_info.index = i;
            double d = sqrt(pow((line->point_beg.dx - points->points[i].dx),2) 
                    + pow((line->point_beg.dy - points->points[i].dy),2));
            tmp_info.scale = d/a;
            tmp_info.adsorb_dis = 0.;
            poi_index.push_back(tmp_info);
            EB_LOG("[EB::WARNING] point %d is in the line %d!!!\n",i,line_index);
            continue ;
        }
    }

    EB_LOG("[EB_INFO::] line %d is adsorbed %d points.\n",line_index,poi_index.size());
    
} 

#if 0
static void adsorbent_dis(eb_line_t *line, eb_point_t *point,int point_index, double adsorb_thresd
                            #ifdef EB_DEBUG
                            ,
                            bool ass
                            #endif
                            )
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

        if(sqrt(1-cos_c*cos_c)*b > adsorb_thresd)
        {
            return;
        }



        line->adsorbent.adsorbent_dis[line->adsorbent.adsorbent_num] = sqrt(1-cos_c*cos_c)*b;

        line->adsorbent.adsorbent_foot[line->adsorbent.adsorbent_num].dx = line->point_beg.dx;
        line->adsorbent.adsorbent_foot[line->adsorbent.adsorbent_num].dy = line->point_beg.dy;
        line->adsorbent.adsorbent_foot[line->adsorbent.adsorbent_num].is_delaunay = line->point_beg.is_delaunay;
        line->adsorbent.adsorbent_foot[line->adsorbent.adsorbent_num].point_x = line->point_beg.point_x;
        line->adsorbent.adsorbent_foot[line->adsorbent.adsorbent_num].point_y = line->point_beg.point_y;
        line->adsorbent.adsorbent_foot[line->adsorbent.adsorbent_num].point_z = line->point_beg.point_z;

        double scale = cos_c*b / a;
        line->adsorbent.adsorbent_foot[line->adsorbent.adsorbent_num].dx += 
                            (line->point_end.dx - line->point_beg.dx) * scale;

        line->adsorbent.adsorbent_foot[line->adsorbent.adsorbent_num].dy += 
                            (line->point_end.dy - line->point_beg.dy) * scale;
        line->adsorbent.adsorbent_index[line->adsorbent.adsorbent_num] = point_index;
        line->adsorbent.adsorbent_num++;

        #ifdef EB_DEBUG
        if(ass)
        {
            EB_LOG("[EB_DEBUG::] %d--> (%f,%f)--(%f,%f)  (%f,%f)\n",
                line->adsorbent.adsorbent_num,
                line->point_beg.dx,
                line->point_beg.dy,
                line->point_end.dx,
                line->point_end.dy,
                point->dx,
                point->dy);
        }
        #endif

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
#endif


static void adsorbent_dis(eb_line_t *line,std::vector<eb_adsorb_info_t> &poi_index)
{
    for(int i = 0; i < poi_index.size(); i++)
    {
        line->adsorbent.adsorbent_dis[line->adsorbent.adsorbent_num] = poi_index[i].adsorb_dis;

        line->adsorbent.adsorbent_foot[line->adsorbent.adsorbent_num].dx = line->point_beg.dx;
        line->adsorbent.adsorbent_foot[line->adsorbent.adsorbent_num].dy = line->point_beg.dy;
        line->adsorbent.adsorbent_foot[line->adsorbent.adsorbent_num].is_delaunay = line->point_beg.is_delaunay;
        line->adsorbent.adsorbent_foot[line->adsorbent.adsorbent_num].point_x = line->point_beg.point_x;
        line->adsorbent.adsorbent_foot[line->adsorbent.adsorbent_num].point_y = line->point_beg.point_y;
        line->adsorbent.adsorbent_foot[line->adsorbent.adsorbent_num].point_z = line->point_beg.point_z;

        line->adsorbent.adsorbent_foot[line->adsorbent.adsorbent_num].dx += 
                            (line->point_end.dx - line->point_beg.dx) * poi_index[i].scale;

        line->adsorbent.adsorbent_foot[line->adsorbent.adsorbent_num].dy += 
                            (line->point_end.dy - line->point_beg.dy) * poi_index[i].scale;
        line->adsorbent.adsorbent_index[line->adsorbent.adsorbent_num] = poi_index[i].index;
        line->adsorbent.adsorbent_num++;
    }
}


#if 0
void adsorbent_filter(eb_features_t * eb_futures_ptr, eb_config_t *eb_config_ptr)
{
    int absord_line_num = 0;
    
    for(int i = 0; i<eb_futures_ptr->buffer_filter_lines.line_size; i++ )
    {
        eb_futures_ptr->buffer_filter_lines.lines[i].adsorbent.adsorbent_index = 
        (int *)malloc(sizeof(int) * eb_futures_ptr->delau_boundary_pois.point_size);

        eb_futures_ptr->buffer_filter_lines.lines[i].adsorbent.adsorbent_foot = 
        (eb_point_t *)malloc(sizeof(eb_point_t) * eb_futures_ptr->delau_boundary_pois.point_size);

        eb_futures_ptr->buffer_filter_lines.lines[i].adsorbent.adsorbent_dis = 
        (double *)malloc(sizeof(double) * eb_futures_ptr->delau_boundary_pois.point_size);


        #ifdef EB_DEBUG
        EB_LOG("[EB_DEBUG::] absord filter is processing line %d\n",i);
        #endif
        for(int j = 0; j < eb_futures_ptr->delau_boundary_pois.point_size; j++)
        {
            #ifdef EB_DEBUG
            EB_LOG("    [EB_DEBUG::] absord filter is processing point %d\n",j);
            bool PRINT_ABSORD = 0;
            if(i == 15 && j == 57)
            {
                PRINT_ABSORD = 1;
            }
            else
            {
                PRINT_ABSORD = 0;
            }
            #endif





            adsorbent_dis(&eb_futures_ptr->buffer_filter_lines.lines[i],
                            &eb_futures_ptr->delau_boundary_pois.points[j],
                            j,
                            eb_config_ptr->min_adsorb_dis
                            #ifdef EB_DEBUG
                            ,
                            PRINT_ABSORD
                            #endif
                            );
        }

        if(eb_futures_ptr->buffer_filter_lines.lines[i].adsorbent.adsorbent_num > 
            eb_config_ptr->min_adsorb_num)
            {
                int absord_num = eb_futures_ptr->buffer_filter_lines.lines[i].adsorbent.adsorbent_num;
                int rand_r = rand()%255;
                int rand_g = rand()%255;
                int rand_b = rand()%255;
                absord_line_num++;
                // #ifdef EB_DEBUG
                // if(i != 0)continue;
                // #endif
                for(int k = 0; k < absord_num; k++)
                {
                    cv::circle(eb_futures_ptr->eb_mats.adsorb_filter_image,
                            cv::Point(eb_futures_ptr->buffer_filter_lines.lines[i].adsorbent.adsorbent_foot[k].dx,
                                        eb_futures_ptr->buffer_filter_lines.lines[i].adsorbent.adsorbent_foot[k].dy),
                                        1,
                                        cv::Scalar(rand_r,rand_g,rand_b),
                                        1,
                                        CV_AA);
                }              
            }
    }
    EB_LOG("[EB INFO::] absord_line_num is %d\n",absord_line_num);
    mat_show("adsorbent_image",eb_futures_ptr->eb_mats.adsorb_filter_image,MAT_SIZE);
}

#endif 



void adsorbent_filter(eb_features_t * eb_futures_ptr, eb_config_t *eb_config_ptr)
{
    int absord_line_num = 0;
    
    for(int i = 0; i<eb_futures_ptr->buffer_filter_lines.line_size; i++ )
    {
        std::vector<eb_adsorb_info_t> poi_index;
        adsorbsent_check(&eb_futures_ptr->buffer_filter_lines.lines[i],
                        i,
                        &eb_futures_ptr->delau_boundary_pois,
                        poi_index,
                        eb_config_ptr->min_adsorb_dis);

        eb_futures_ptr->buffer_filter_lines.lines[i].adsorbent.adsorbent_index = 
        (int *)malloc(sizeof(int) * poi_index.size());

        eb_futures_ptr->buffer_filter_lines.lines[i].adsorbent.adsorbent_foot = 
        (eb_point_t *)malloc(sizeof(eb_point_t) * poi_index.size());

        eb_futures_ptr->buffer_filter_lines.lines[i].adsorbent.adsorbent_dis = 
        (double *)malloc(sizeof(double) * poi_index.size());


        adsorbent_dis(&eb_futures_ptr->buffer_filter_lines.lines[i],poi_index);

        if(eb_futures_ptr->buffer_filter_lines.lines[i].adsorbent.adsorbent_num > 
            eb_config_ptr->min_adsorb_num)
            {
                int absord_num = eb_futures_ptr->buffer_filter_lines.lines[i].adsorbent.adsorbent_num;
                int rand_r = rand()%255;
                int rand_g = rand()%255;
                int rand_b = rand()%255;
                absord_line_num++;
                // #ifdef EB_DEBUG
                // if(i != 0)continue;
                // #endif
                for(int k = 0; k < absord_num; k++)
                {
                    cv::circle(eb_futures_ptr->eb_mats.adsorb_filter_image,
                            cv::Point(eb_futures_ptr->buffer_filter_lines.lines[i].adsorbent.adsorbent_foot[k].dx,
                                        eb_futures_ptr->buffer_filter_lines.lines[i].adsorbent.adsorbent_foot[k].dy),
                                        1,
                                        cv::Scalar(rand_r,rand_g,rand_b),
                                        1,
                                        CV_AA);
                }              
            }
    }
    mat_show("adsorbent_image",eb_futures_ptr->eb_mats.adsorb_filter_image,MAT_SIZE);
}


