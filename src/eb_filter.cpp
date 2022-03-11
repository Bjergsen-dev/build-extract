#include "eb_filter.hpp"
#include "eb_config.hpp"
#include "eb_common_defines.hpp"
#include "eb_images_detect.hpp"
#include "eb_transform.hpp"
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
    #ifdef TEST_1
    cv::Mat ass_hole_mat = cv::Mat(eb_futures_ptr->eb_mats.image_height,
                                    eb_futures_ptr->eb_mats.image_width,
                                    CV_8UC3,cv::Scalar(255,255,255));
    
    #endif
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

            #ifdef MID_RESULT
            double trans_res1[2];
            double trans_res2[2];
            trans_dx_dy_to_all(cv::Point(tmp_lines[tmp_index].point_beg.dx,tmp_lines[tmp_index].point_beg.dy),
                                eb_config_ptr,
                                trans_res1);
            trans_dx_dy_to_all(cv::Point(tmp_lines[tmp_index].point_end.dx,tmp_lines[tmp_index].point_end.dy),
                                eb_config_ptr,
                                trans_res2);
            cv::line(eb_futures_ptr->eb_mats.all_buf_filter_image,
                    cv::Point(trans_res1[0],trans_res1[1]),
                    cv::Point(trans_res2[0],trans_res2[1]),
                    cv::Scalar(255),
                    5,
                    CV_AA);
            #endif

            #ifdef TEST_1
            cv::line(ass_hole_mat,
                    cv::Point(tmp_lines[tmp_index].point_beg.dx,tmp_lines[tmp_index].point_beg.dy),
                    cv::Point(tmp_lines[tmp_index].point_end.dx,tmp_lines[tmp_index].point_end.dy),
                    cv::Scalar(0,0,255),
                    1,
                    CV_AA);

            cv::line(eb_futures_ptr->eb_mats.adsorb_filter_image,
                    cv::Point(tmp_lines[tmp_index].point_beg.dx,tmp_lines[tmp_index].point_beg.dy),
                    cv::Point(tmp_lines[tmp_index].point_end.dx,tmp_lines[tmp_index].point_end.dy),
                    cv::Scalar(0,0,255),
                    1,
                    CV_AA);
            #endif
        }
    }

    #ifdef TEST_1
    for(int i = 0; i < eb_futures_ptr->delau_boundary_pois.point_size;i++)
    {
        cv::circle(ass_hole_mat,
                    cv::Point(eb_futures_ptr->delau_boundary_pois.points[i].dx,
                    eb_futures_ptr->delau_boundary_pois.points[i].dy),
                    5,
                    cv::Scalar(0,0,0),
                    1,
                    CV_AA);
    }
    mat_show("ass_hole",ass_hole_mat,MAT_SIZE,eb_config_ptr);
    #endif
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

    #ifdef TEST_8_SOBEL
    mat_show("8_sobel_buffer_filter_image",
            eb_futures_ptr->eb_mats.buf_filter_image,
            MAT_SIZE,eb_config_ptr);
    #endif

    #ifdef TEST_CANNY
    mat_show("canny_buffer_filter_image",
            eb_futures_ptr->eb_mats.buf_filter_image,
            MAT_SIZE,eb_config_ptr);
    #endif

    #ifdef TEST_LoG
    mat_show("Log_buffer_filter_image",
            eb_futures_ptr->eb_mats.buf_filter_image,
            MAT_SIZE,eb_config_ptr);
    #endif

    EB_LOG("[EB::INFO] buffer filter completed，line sum is %d!\n\n",eb_futures_ptr->buffer_filter_lines.line_size);
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

            #ifdef TEST_EVALUATION
            points->points[i].absorb_num++;
            #endif
            
        }
        else
        {
            if(a < b || a < c)
            {
                continue ;
            }
            #ifdef TEST_EVALUATION
            points->points[i].absorb_num++;
            #endif
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

    EB_LOG("[EB::INFO] line %d is adsorbed %ld points.\n",line_index,poi_index.size());
    
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


static void adsorbent_dis(eb_line_t *line,std::vector<eb_adsorb_info_t> &poi_index,eb_points_t *boudary_pois)
{
    for(int i = 0; i < poi_index.size(); i++)
    {
        line->adsorbent.adsorbent_dis[line->adsorbent.adsorbent_num] = poi_index[i].adsorb_dis;

        line->adsorbent.adsorbent_foot[line->adsorbent.adsorbent_num].dx = line->point_beg.dx;
        line->adsorbent.adsorbent_foot[line->adsorbent.adsorbent_num].dy = line->point_beg.dy;
        line->adsorbent.adsorbent_foot[line->adsorbent.adsorbent_num].is_delaunay = 1;
        line->adsorbent.adsorbent_foot[line->adsorbent.adsorbent_num].point_x = boudary_pois->points[poi_index[i].index].point_x;
        line->adsorbent.adsorbent_foot[line->adsorbent.adsorbent_num].point_y = boudary_pois->points[poi_index[i].index].point_y;
        line->adsorbent.adsorbent_foot[line->adsorbent.adsorbent_num].point_z = boudary_pois->points[poi_index[i].index].point_z;

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
            EB_LOG("[EB_DEBUG::] absord filter is processing point %d\n",j);
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

        eb_futures_ptr->buffer_filter_lines.lines[i].adsorbent.adsorbent_num = 0;
        eb_futures_ptr->buffer_filter_lines.lines[i].adsorbent.adsorbent_index = 
        (int *)malloc(sizeof(int) * poi_index.size());

        eb_futures_ptr->buffer_filter_lines.lines[i].adsorbent.adsorbent_foot = 
        (eb_point_t *)malloc(sizeof(eb_point_t) * poi_index.size());

        eb_futures_ptr->buffer_filter_lines.lines[i].adsorbent.adsorbent_dis = 
        (double *)malloc(sizeof(double) * poi_index.size());


        adsorbent_dis(&eb_futures_ptr->buffer_filter_lines.lines[i],poi_index,&eb_futures_ptr->delau_boundary_pois);

        if(eb_futures_ptr->buffer_filter_lines.lines[i].adsorbent.adsorbent_num > 
            eb_config_ptr->min_adsorb_num)
            {
                int absord_num = eb_futures_ptr->buffer_filter_lines.lines[i].adsorbent.adsorbent_num;
                int rand_r = rand()%255;
                int rand_g = rand()%255;
                int rand_b = rand()%255;
                absord_line_num++;
                for(int k = 0; k < absord_num; k++)
                {
                    #ifdef TEST_1
                    cv::circle(eb_futures_ptr->eb_mats.adsorb_filter_image,
                            cv::Point(eb_futures_ptr->buffer_filter_lines.lines[i].adsorbent.adsorbent_foot[k].dx,
                                        eb_futures_ptr->buffer_filter_lines.lines[i].adsorbent.adsorbent_foot[k].dy),
                                        3,
                                        cv::Scalar(rand_r,rand_g,rand_b),
                                        1,
                                        CV_AA);
                    #else
                    cv::circle(eb_futures_ptr->eb_mats.adsorb_filter_image,
                            cv::Point(eb_futures_ptr->buffer_filter_lines.lines[i].adsorbent.adsorbent_foot[k].dx,
                                        eb_futures_ptr->buffer_filter_lines.lines[i].adsorbent.adsorbent_foot[k].dy),
                                        1,
                                        cv::Scalar(rand_r,rand_g,rand_b),
                                        1,
                                        CV_AA);
                    #endif
                }              
            }
    }
    EB_LOG("[EB::INFO] adsorb filter completed!\n\n");
    mat_show("adsorbent_image",eb_futures_ptr->eb_mats.adsorb_filter_image,MAT_SIZE,eb_config_ptr);
}

typedef struct eb_update_poinfo
{
    int adsorb_num;
    eb_point_t point;
    int is_head;

}eb_update_poinfo_t;

typedef struct eb_insert_info
{
    eb_point_t point;
    bool is_head;
}eb_insert_info_t;



void eb_update_boundary_pois(eb_features_t * eb_futures_ptr,eb_config_t *eb_config)
{
    eb_lines_t *adsorb_lines = &eb_futures_ptr->buffer_filter_lines;
    eb_points_t *boudry_pois = &eb_futures_ptr->delau_boundary_pois;
    eb_update_poinfo *tmp_update_ptr = (eb_update_poinfo *)malloc(sizeof(eb_update_poinfo) * boudry_pois->point_size);
    memset(tmp_update_ptr,0,sizeof(eb_update_poinfo) * boudry_pois->point_size);

    std::vector<eb_insert_info_t> tmp_insert_pois;
        for(int i = 0; i < adsorb_lines->line_size; i++)
        {
            if(adsorb_lines->lines[i].adsorbent.adsorbent_num <= eb_config->min_adsorb_num)
            {
                continue;
            }
            else
            {
                #ifdef EB_DEBUG
                EB_LOG("[EB_DEBUG::] line %d --> ",i);
                #endif
                for(int j =0; j < adsorb_lines->lines[i].adsorbent.adsorbent_num; j++ )
                {
                    int poi_index = adsorb_lines->lines[i].adsorbent.adsorbent_index[j];
                    #ifdef EB_DEBUG
                    EB_LOG("%d ",poi_index);
                    #endif
                    if(adsorb_lines->lines[i].adsorbent.adsorbent_num > 
                        tmp_update_ptr[poi_index].adsorb_num)
                        {
                            if(j == 0)
                            {
                                if(tmp_update_ptr[poi_index].is_head == -1)
                                {
                                    eb_insert_info_t tmp;
                                    tmp.point = tmp_update_ptr[poi_index].point;
                                    tmp.is_head = 0;
                                    tmp_insert_pois.push_back(tmp);
                                }
                                tmp_update_ptr[poi_index].is_head = 1;
                            }

                            if(j == adsorb_lines->lines[i].adsorbent.adsorbent_num-1)
                            {
                                if(tmp_update_ptr[poi_index].is_head == 1)
                                {
                                    eb_insert_info_t tmp;
                                    tmp.point = tmp_update_ptr[poi_index].point;
                                    tmp.is_head = 1;
                                    tmp_insert_pois.push_back(tmp);
                                }
                                
                                tmp_update_ptr[poi_index].is_head = -1;
                            }


                            tmp_update_ptr[poi_index].adsorb_num = adsorb_lines->lines[i].adsorbent.adsorbent_num;
                            adsorb_lines->lines[i].adsorbent.adsorbent_foot[j].is_adsorbed = 1;
                            tmp_update_ptr[poi_index].point = adsorb_lines->lines[i].adsorbent.adsorbent_foot[j];
                            tmp_update_ptr[poi_index].point.adsorb_line_idx = i;
                        }
                        else
                        {
                            if(j == 0)
                            {
                                if(tmp_update_ptr[poi_index].is_head == -1)
                                {
                                    eb_insert_info_t tmp;
                                    tmp.point = adsorb_lines->lines[i].adsorbent.adsorbent_foot[j];
                                    tmp.point.is_adsorbed = 1;
                                    tmp.point.adsorb_line_idx = i;
                                    tmp.is_head = 1;
                                    tmp_insert_pois.push_back(tmp);
                                }
                            }

                            if(j == adsorb_lines->lines[i].adsorbent.adsorbent_num-1)
                            {
                                if(tmp_update_ptr[poi_index].is_head == 1)
                                {
                                    eb_insert_info_t tmp;
                                    tmp.point = adsorb_lines->lines[i].adsorbent.adsorbent_foot[j];
                                    tmp.point.is_adsorbed = 1;
                                    tmp.point.adsorb_line_idx = i;
                                    tmp.is_head = 0;
                                    tmp_insert_pois.push_back(tmp);
                                }

                            }
                        }
                }
                #ifdef EB_DEBUG
                EB_LOG("\n");
                #endif
            }
        }

        eb_futures_ptr->insert_pois.points = (eb_point_t *)malloc(sizeof(eb_point_t)*tmp_insert_pois.size());
        eb_futures_ptr->insert_pois.insert_index = (bool *)malloc(sizeof(bool) * tmp_insert_pois.size());
        eb_futures_ptr->insert_pois.point_size = tmp_insert_pois.size();

        for(int i =0; i < eb_futures_ptr->insert_pois.point_size; i++)
        {
            eb_futures_ptr->insert_pois.points[i] = tmp_insert_pois[i].point;
            eb_futures_ptr->insert_pois.insert_index[i] = tmp_insert_pois[i].is_head;
            cv::circle(eb_futures_ptr->eb_mats.adsorb_update_image,
                            cv::Point(eb_futures_ptr->insert_pois.points[i].dx,eb_futures_ptr->insert_pois.points[i].dy),
                            2,
                            cv::Scalar(0,0,255),
                            1,
                            CV_AA);
            #ifdef TEST_2
            cv::circle(eb_futures_ptr->eb_mats.simplify_lines_image,
                            cv::Point(eb_futures_ptr->insert_pois.points[i].dx,eb_futures_ptr->insert_pois.points[i].dy),
                            5,
                            cv::Scalar(0,0,255),
                            1,
                            CV_AA);
            #endif

        }
        #ifdef TEST_EVALUATION
        int un_adsorb_num = 0;
        float insensity = 0.;
        #endif

        #ifdef TEST_EVALUATION
        EB_LOG("[EB_DEBUG::] boundary_pois sum is %d\n",boudry_pois->point_size);
        #endif
        for(int i = 0; i < boudry_pois->point_size; i++)
        {
            #ifdef TEST_EVALUATION
            EB_LOG("[EB_DEBUG::] boundary_pois %d adsorbnum is %d\n",i,boudry_pois->points[i].absorb_num);
            insensity += boudry_pois->points[i].absorb_num;
            #endif
            if(tmp_update_ptr[i].adsorb_num == 0)
            {
                cv::circle(eb_futures_ptr->eb_mats.adsorb_update_image,
                            cv::Point(boudry_pois->points[i].dx,boudry_pois->points[i].dy),
                            2,
                            cv::Scalar(0,0,0),
                            1,
                            CV_AA);
                #ifdef TEST_2
                #ifdef TEST_EVALUATION
                un_adsorb_num++;
                #endif
                cv::circle(eb_futures_ptr->eb_mats.simplify_lines_image,
                            cv::Point(boudry_pois->points[i].dx,boudry_pois->points[i].dy),
                            3,
                            cv::Scalar(0,0,0),
                            1,
                            CV_AA);
                #endif
                continue;
            }
            else
            {
                boudry_pois->points[i] = tmp_update_ptr[i].point;
                cv::circle(eb_futures_ptr->eb_mats.adsorb_update_image,
                            cv::Point(boudry_pois->points[i].dx,boudry_pois->points[i].dy),
                            1,
                            cv::Scalar(0,255,0),
                            1,
                            CV_AA);
                
                #ifdef TEST_2
                cv::circle(eb_futures_ptr->eb_mats.simplify_lines_image,
                            cv::Point(boudry_pois->points[i].dx,boudry_pois->points[i].dy),
                            3,
                            cv::Scalar(0,255,0),
                            1,
                            CV_AA);
                #endif
            }
        }

        #ifdef TEST_EVALUATION
        insensity = insensity/(float)boudry_pois->point_size;
        char eva_path[256];
        snprintf(eva_path,strlen(eb_config->file_config.out_path)+1,eb_config->file_config.out_path);
        #ifdef TEST_LoG
        snprintf(eva_path+strlen(eva_path),20,"/LoG_evaluation.txt");
        #endif
        #ifdef TEST_CANNY
        snprintf(eva_path+strlen(eva_path),22,"/Canny_evaluation.txt");
        #endif
        #ifdef TEST_8_SOBEL
        snprintf(eva_path+strlen(eva_path),24,"/8_Sobel_evaluation.txt");
        #endif
        FILE * evaulation_fp = fopen(eva_path,"w");
        fprintf(evaulation_fp,"all_lines: %d\n in_lines: %d \n p_r: %f\n sum_pois: %d \n un_adsorb_num: %d\n r_c: %f\n insensity: %f\n",
                    eb_futures_ptr->hough_lines.line_size,
                    eb_futures_ptr->buffer_filter_lines.line_size,
                    (float)eb_futures_ptr->buffer_filter_lines.line_size/(float)eb_futures_ptr->hough_lines.line_size,
                    boudry_pois->point_size,
                    un_adsorb_num,
                    (float)(boudry_pois->point_size-un_adsorb_num)/(float)boudry_pois->point_size,
                    insensity);
        fclose(evaulation_fp);
        #endif

        mat_show("adsorb_update_image",
                    eb_futures_ptr->eb_mats.adsorb_update_image,
                    MAT_SIZE,eb_config);

        EB_LOG("[EB::INFO] adsorb update completed!\n\n");
        free(tmp_update_ptr);
        tmp_update_ptr = NULL;
}

static void update_corner_pois(std::vector<eb_final_line> &lines_vec, eb_ins_points_t *ins_pois)
{
    for(int i = 0; i< lines_vec.size(); i++)
    {
        int adsorb_line_idx = lines_vec[i].point_beg.adsorb_line_idx;
        for(int j =0; j< ins_pois->point_size; j++)
        {
            if(ins_pois->points[j].adsorb_line_idx == adsorb_line_idx)
            {
                if(ins_pois->insert_index[j] == 1)
                {
                    ins_pois->points[j].delau_pois_idx = lines_vec[i].point_beg.delau_pois_idx;
                    lines_vec[i].point_beg = ins_pois->points[j];
                }
                else
                {
                    ins_pois->points[j].delau_pois_idx = lines_vec[i].point_end.delau_pois_idx;
                    lines_vec[i].point_end = ins_pois->points[j];
                }
            }
        }
    }
}

static void pre_generate_roof(eb_features_t *eb_features_ptr,std::vector<eb_final_line_t> &tmp_lines_vec,eb_config_t *eb_config_ptr)
{
    eb_points_t *boudary_pois = &eb_features_ptr->delau_boundary_pois;
    eb_ins_points_t *ins_pois = &eb_features_ptr->insert_pois;
    int  sum_poi_num = boudary_pois->point_size ;//+ ins_pois->point_size;
    eb_point_t *tmp_pois = (eb_point_t *)malloc(sizeof(eb_point_t) *sum_poi_num);
    memset(tmp_pois,0,sizeof(eb_point_t) *sum_poi_num);

#ifdef EB_DEBUG
    for(int i = 0 ; i < ins_pois->point_size; i++)
    {
        
        EB_LOG("[EB_DEBUG::] insert point %d -->line %d is_head : %d\n",
                i,ins_pois->points[i].adsorb_line_idx,ins_pois->insert_index[i]);
        
    }
#endif

    #ifdef EB_DEBUG
    EB_LOG("\n\n");
    #endif

    for(int i =0; i < boudary_pois->point_size; i++)
    {

        #ifdef EB_DEBUG
        EB_LOG("[EB_DEBUG::] origin point %d -->line %d --%d\n",
                i,boudary_pois->points[i].adsorb_line_idx,boudary_pois->points[i].is_delaunay);
        #endif
        int cur_index = 0;
        while(tmp_pois[cur_index].is_delaunay != 0)
        {
            cur_index++;
        }
        
        tmp_pois[cur_index] = boudary_pois->points[i];

        #ifdef EB_DEBUG
        if(boudary_pois->points[i].adsorb_line_idx == -1)
        {
            EB_LOG("[EB_DEBUG::] --------------------cur_index:%d tmp_pois[%d].adsorb_line_idx = %d \n",
                    cur_index,cur_index,tmp_pois[cur_index].adsorb_line_idx);
        }
        #endif
    }

    #ifdef EB_DEBUG
    EB_LOG("\n\n");
    #endif

    int last_adsorb_idx = -2;
    eb_final_line_t tmp_line;
    int un_adsorb_num = 0;
    for(int i = 0 ;  i< sum_poi_num; i++)
    {
        #ifdef EB_DEBUG
        EB_LOG("[EB_DEBUG::] combine point %d --> line %d --%d\n",
                i,tmp_pois[i].adsorb_line_idx,tmp_pois[i].is_adsorbed);
        #endif
        if(tmp_pois[i].is_adsorbed ==  0)
        {
            un_adsorb_num += 1;
            continue;
        }

        if(last_adsorb_idx == -2)
        {
            tmp_line.point_beg = tmp_pois[i];
            tmp_line.point_beg.delau_pois_idx = i;
            last_adsorb_idx = tmp_pois[i].adsorb_line_idx;
            un_adsorb_num = 0;
            continue;
        }

        if(tmp_pois[i].adsorb_line_idx != last_adsorb_idx)
        {
            tmp_line.point_end = tmp_pois[i-1 - un_adsorb_num];
            tmp_line.point_end.delau_pois_idx = i-1 - un_adsorb_num;
            if(tmp_line.point_beg.dx != tmp_line.point_end.dx || 
                tmp_line.point_beg.dy != tmp_line.point_end.dy)
                {
                    tmp_lines_vec.push_back(tmp_line);
                    
                }     
            tmp_line.point_beg = tmp_pois[i];
            tmp_line.point_beg.delau_pois_idx = i;
            last_adsorb_idx = tmp_pois[i].adsorb_line_idx;
            un_adsorb_num = 0;
            continue;
        }

        tmp_line.point_end = tmp_pois[i];
        tmp_line.point_end.delau_pois_idx = i;
        un_adsorb_num = 0;

    }

    if(tmp_line.point_beg.adsorb_line_idx == tmp_lines_vec[0].point_beg.adsorb_line_idx)
    {
        tmp_lines_vec[0].point_beg = tmp_line.point_beg;
    }
    else
    {
        if(tmp_line.point_beg.delau_pois_idx < tmp_line.point_end.delau_pois_idx)
        {
            tmp_lines_vec.push_back(tmp_line);
        }
        
    }

    update_corner_pois(tmp_lines_vec,&eb_features_ptr->insert_pois);

    for(int i = 0; i < tmp_lines_vec.size();i++)
    {
        int next = i == tmp_lines_vec.size()-1? 0:i+1;

        if(tmp_lines_vec[i].point_beg.adsorb_line_idx == tmp_lines_vec[next].point_beg.adsorb_line_idx)
        {
            tmp_lines_vec[i].point_end = tmp_lines_vec[next].point_end;
            tmp_lines_vec.erase(tmp_lines_vec.begin()+next);
        }
        #ifdef TEST_2
        int rgb_r = 0;
        int rgb_g = 0;
        int rgb_b = 255;
        #else
        int rgb_r = rand()%255;
        int rgb_g = rand()%255;
        int rgb_b = rand()%255;
        #endif

        #if 1
        cv::line(eb_features_ptr->eb_mats.simplify_lines_image,
                cv::Point(tmp_lines_vec[i].point_beg.dx,tmp_lines_vec[i].point_beg.dy),
                cv::Point(tmp_lines_vec[i].point_end.dx,tmp_lines_vec[i].point_end.dy),
                cv::Scalar(rgb_r,rgb_g,rgb_b),
                #ifdef TEST_2
                2,
                #else
                2,
                #endif
                CV_AA);

        #ifdef MID_RESULT
            double trans_res1[2];
            double trans_res2[2];
            trans_dx_dy_to_all(cv::Point(tmp_lines_vec[i].point_beg.dx,tmp_lines_vec[i].point_beg.dy),
                                eb_config_ptr,
                                trans_res1);
            trans_dx_dy_to_all(cv::Point(tmp_lines_vec[i].point_end.dx,tmp_lines_vec[i].point_end.dy),
                                eb_config_ptr,
                                trans_res2);
        cv::line(eb_features_ptr->eb_mats.all_simply_image,
                cv::Point(trans_res1[0],trans_res1[1]),
                cv::Point(trans_res2[0],trans_res2[1]),
                cv::Scalar(rgb_r,rgb_g,rgb_b),
                #ifdef TEST_2
                5,
                #else
                2,
                #endif
                CV_AA);
        #endif
        #endif
    }

    free(tmp_pois);
    tmp_pois = NULL;

    #if 1
    mat_show("simplify_image",eb_features_ptr->eb_mats.simplify_lines_image,MAT_SIZE,eb_config_ptr);
    #endif
    EB_LOG("[EB::INFO] simplify roof completed!\n");

    
}

static double length_of_line(eb_final_line_t *line)
{
    return sqrt(pow(line->point_beg.dx - line->point_end.dx, 2) + 
                pow(line->point_beg.dy - line->point_end.dy, 2));
}

static void direction_of_line(eb_final_line_t *line, double *dirct)
{
    double delta_x = line->point_end.dx - line->point_beg.dx;
    double delta_y = line->point_end.dy - line->point_beg.dy;
    double length = length_of_line(line);
    dirct[0] = delta_x/length;
    dirct[1] = delta_y/length;

    line->direct[0] = dirct[0];
    line->direct[1] = dirct[1];
}

static void reset_line_with_direc(eb_final_line_t *line,double *dirct,double thrd)
{
    double tmp_dire[2];
    direction_of_line(line,tmp_dire);

    double cc = pow(dirct[0] - tmp_dire[0], 2) + pow(dirct[1] - tmp_dire[1], 2);

    double cos = (2-cc)/2;
    double sin = sqrt(1 - cos*cos);

    double x,y,a,b,c,d,e;
    bool is_reset = 0;

    
    a = (line->point_beg.dx + line->point_end.dx)/2;
    b = (line->point_beg.dy + line->point_end.dy)/2;

    if(cos>thrd)
    {
        c = dirct[0];
        d = dirct[1];
        e = length_of_line(line) * fabs(cos);
        is_reset = 1;
    }

    if(cos < -thrd)
    {
        c = -dirct[0];
        d = -dirct[1];
        e = length_of_line(line) * fabs(cos);
        is_reset = 1;
    }

    if(sin > thrd)
    {
        #if 0
        if(tmp_dire[1] > 0)
        {
            c = -dirct[1];
            d = dirct[0];
        }

        if(tmp_dire[1] <= 0)
        {
            c = dirct[1];
            d = -dirct[0];
        }
        #endif 
        #if 0
        if(tmp_dire[0] >= 0)c=fabs(dirct[1]);
        if(tmp_dire[0] < 0)c=-fabs(dirct[1]);
        if(tmp_dire[1] >= 0)d=fabs(dirct[0]);
        if(tmp_dire[1] < 0)d=-fabs(dirct[0]);
        #endif

        if(fabs(tmp_dire[0]) >= fabs(tmp_dire[1]))
        {
            if(tmp_dire[0] >= 0)c=fabs(dirct[1]);
            if(tmp_dire[0] < 0)c=-fabs(dirct[1]);
            d = -c*dirct[0]/dirct[1];
        }
        else
        {
            if(tmp_dire[1] >= 0)d=fabs(dirct[0]);
            if(tmp_dire[1] < 0)d=-fabs(dirct[0]);
            c = -d*dirct[1]/dirct[0];
        }


        e = length_of_line(line) * fabs(sin);
        is_reset = 1;
    }

    if(is_reset)
    {
        line->point_beg.dx = a - (c/2)*e;
        line->point_beg.dy = b - (d/2)*e;

        line->point_end.dx = 2*a-line->point_beg.dx;
        line->point_end.dy = 2*b-line->point_beg.dy;

        direction_of_line(line,tmp_dire);
    }


}

#if 1
static int parallel_check(eb_final_line_t* line_1, eb_final_line_t *line_2, eb_points_t *delau_budry_pois,int signal)
{
    double first_sum = 0.;
    double second_sum = 0.;
    int end = signal == 1? delau_budry_pois->point_size : line_2->point_beg.delau_pois_idx;
    for(int i = line_1->point_end.delau_pois_idx+1; i < end; i++)
    {
        //if(!(i < line_2->point_beg.delau_pois_idx))break;
        eb_point_t *poi = &delau_budry_pois->points[i];
#if 1
        double dis_1_cos = fabs(line_1->direct[0]*(poi->dx-line_1->point_end.dx)+
                            line_1->direct[1]*(poi->dy-line_1->point_end.dy));
        double dis_1_sin = fabs(line_1->direct[0]*(poi->dy-line_1->point_end.dy)-
                            line_1->direct[1]*(poi->dx-line_1->point_beg.dx));
        

        double dis_2_cos = fabs(line_2->direct[0]*(poi->dx-line_2->point_end.dx)+
                            line_2->direct[1]*(poi->dy-line_2->point_end.dy));
        double dis_2_sin = fabs(line_2->direct[0]*(poi->dy-line_2->point_end.dy)-
                            line_2->direct[1]*(poi->dx-line_2->point_beg.dx));
        
        #if 0
        double first_dis = dis_1_sin + dis_2_cos;
        double second_dis = dis_1_cos + dis_2_sin;
        #endif
        double first_dis = EB_MIN(dis_1_sin,dis_2_cos);
        double second_dis = EB_MIN(dis_1_cos,dis_2_sin);
        EB_LOG("[EB::DEBUG] poi(%f %f),dis_1_sin: %f, dis_1_cos: %f, dis_2_sin: %f, dis_2_cos: %f\n",
                poi->dx,
                poi->dy,
                dis_1_sin,
                dis_1_cos,
                dis_2_sin,
                dis_2_cos);

#endif
#if 0
        double first_dis = pow((poi->dx - line_1->point_end.dx),2) + 
                            pow((poi->dy - line_1->point_end.dy),2);

        double second_dis = pow((poi->dx - line_2->point_beg.dx),2) + 
                            pow((poi->dy - line_2->point_beg.dy),2);
#endif

        first_sum += first_dis;
        second_sum += second_dis;
    }

    if(first_sum == second_sum && first_sum == 0.)
    {
        EB_LOG("[EB_DEBUG::] PARALLEL CHECK is 0!\n");
        return 0;
    }

    if(first_sum > second_sum)
    {
        EB_LOG("[EB_DEBUG::] PARALLEL CHECK is -1! First sum is %lf Second sum is %lf\n",first_sum,second_sum);
        return -1;
    }

    if(first_sum < second_sum)
    {
        EB_LOG("[EB_DEBUG::] PARALLEL CHECK is 1!\n");
        return 1;
    }
}


static void close_roof_lines(std::vector<eb_final_line_t> &lines_vec,double dire_trd,eb_features_t *eb_feature_ptr,eb_roof_t *roof,eb_config_t* eb_config_ptr)
{
    eb_points_t * delau_budry_pois = &eb_feature_ptr->delau_boundary_pois;
    std::vector<eb_final_line_t> final_roof_vec;
    for(int i =0; i<lines_vec.size(); i++)
    {
        int next = i==lines_vec.size()-1?0:i+1;
        
        
        double cc = pow(lines_vec[i].direct[0] - lines_vec[next].direct[0], 2) + 
                    pow(lines_vec[i].direct[1] - lines_vec[next].direct[1], 2);
        double cos = (2-cc)/2;
        double sin = sqrt(1-cos*cos);

        if(fabs(cos) > dire_trd)
        {
            int signal = 0;
            if(next == 0)signal = 1;
            switch (parallel_check(&lines_vec[i],&lines_vec[next],delau_budry_pois,signal))
            {
            case -1:
                {
                    EB_LOG("[EB::DEBUG] fisrt line is (%f,%f)--(%f %f) direct is (%f,%f)\n",lines_vec[i].point_beg.dx,lines_vec[i].point_beg.dy,
                                                lines_vec[i].point_end.dx,lines_vec[i].point_end.dy,
                                                lines_vec[i].direct[0],
                                                lines_vec[i].direct[1]);
                    EB_LOG("[EB::DEBUG] second line is (%f,%f)--(%f %f) direct is (%f,%f)\n",lines_vec[next].point_beg.dx,lines_vec[next].point_beg.dy,
                                                lines_vec[next].point_end.dx,lines_vec[next].point_end.dy,
                                                lines_vec[next].direct[0],
                                                lines_vec[next].direct[1]);
                    double c = lines_vec[next].point_beg.dx;
                    double d = lines_vec[next].point_beg.dy;
                    double a = lines_vec[i].point_end.dx;
                    double b = lines_vec[i].point_end.dy;
                    double m = lines_vec[next].direct[0];
                    double n = lines_vec[next].direct[1];
                    
                    double y = b*n*n + m*m*d + m*n*(a-c);
                    double x = m/(n+MIN_DOUBLE) * (y-d) + c;
                    #ifdef EB_DEBUG
                    cv::circle(eb_feature_ptr->eb_mats.refine_liens_image,
                                cv::Point(x,y),
                                2,
                                cv::Scalar(255,0,0),
                                2,
                                CV_AA);
                    #endif
                    final_roof_vec.push_back(lines_vec[i]);
                    lines_vec[next].point_beg.dx = x;
                    lines_vec[next].point_beg.dy = y;
                    eb_final_line_t new_line;
                    new_line.point_beg = lines_vec[i].point_end;
                    new_line.point_end = lines_vec[next].point_beg;
                    double dir[2];
                    direction_of_line(&new_line,dir);
                    final_roof_vec.push_back(new_line);


                    break;
                }
                
            case 0:
                {
                    double a = (lines_vec[next].point_beg.dx + lines_vec[i].point_end.dx)/2;
                    double b = (lines_vec[next].point_beg.dy + lines_vec[i].point_end.dy)/2;
                    double c1 = lines_vec[i].point_end.dx;
                    double d1 = lines_vec[i].point_end.dy;
                    double c2 = lines_vec[next].point_beg.dx;
                    double d2 = lines_vec[next].point_beg.dy;
                    double m1 = lines_vec[i].direct[0];
                    double n1 = lines_vec[i].direct[1];
                    double m2 = lines_vec[next].direct[0];
                    double n2 = lines_vec[next].direct[1];
                    
                    double y1 = b*n1*n1 + m1*m1*d1+ m1*n1*(a-c1);
                    double x1 = m1/(n1+MIN_DOUBLE) * (y1-d1) + c1;

                    double y2 = b*n2*n2 + m2*m2*d2+ m2*n2*(a-c2);
                    double x2 = m2/(n2+MIN_DOUBLE) * (y2-d2) + c2;
                    #ifdef EB_DEBUG
                    cv::circle(eb_feature_ptr->eb_mats.refine_liens_image,
                                cv::Point(x1,y1),
                                2,
                                cv::Scalar(0,255,0),
                                2,
                                CV_AA);
                    cv::circle(eb_feature_ptr->eb_mats.refine_liens_image,
                                cv::Point(x2,y2),
                                2,
                                cv::Scalar(0,255,0),
                                2,
                                CV_AA);
                    #endif
                    lines_vec[i].point_end.dx = x1;
                    lines_vec[i].point_end.dy = y1;
                    lines_vec[next].point_beg.dx = x2;
                    lines_vec[next].point_beg.dy = y2;
                    final_roof_vec.push_back(lines_vec[i]);
                    eb_final_line_t new_line;
                    new_line.point_beg = lines_vec[i].point_end;
                    new_line.point_end = lines_vec[next].point_beg;
                    double dir[2];
                    direction_of_line(&new_line,dir);
                    final_roof_vec.push_back(new_line);
                    
                    break;
                }
                

            case 1:
                {
                    double a = lines_vec[next].point_beg.dx;
                    double b = lines_vec[next].point_beg.dy;
                    double c = lines_vec[i].point_end.dx;
                    double d = lines_vec[i].point_end.dy;
                    double m = lines_vec[i].direct[0];
                    double n = lines_vec[i].direct[1];
                    
                    double y = b*n*n + m*m*d + m*n*(a-c);

                    double x = m/(n+MIN_DOUBLE) * (y-d) + c;
                    #ifdef EB_DEBUG
                    cv::circle(eb_feature_ptr->eb_mats.refine_liens_image,
                                cv::Point(x,y),
                                2,
                                cv::Scalar(0,0,255),
                                2,
                                CV_AA);
                    #endif
                    
                    lines_vec[i].point_end.dx = x;
                    lines_vec[i].point_end.dy = y;
                    final_roof_vec.push_back(lines_vec[i]);
                    eb_final_line_t new_line;
                    new_line.point_beg = lines_vec[i].point_end;
                    new_line.point_end = lines_vec[next].point_beg;
                    double dir[2];
                    direction_of_line(&new_line,dir);
                    final_roof_vec.push_back(new_line);
                    break;
                }
                
            
            default:
                break;
            }
        }
        else if(sin > dire_trd)
        {
            double c = lines_vec[next].point_beg.dx;
            double d = lines_vec[next].point_beg.dy;
            double a = lines_vec[i].point_end.dx;
            double b = lines_vec[i].point_end.dy;
            double m1 = lines_vec[i].direct[0];
            double n1 = lines_vec[i].direct[1];
            double m2 = lines_vec[next].direct[0];
            double n2 = lines_vec[next].direct[1];

            double y = ((m1/(n1+MIN_DOUBLE))*b - 
                            (m2/(n2+MIN_DOUBLE))*d + c - a)/(m1/(n1+MIN_DOUBLE) - 
                                    m2/(n2+MIN_DOUBLE));
            double x = (y-b)*(m1/(n1+MIN_DOUBLE))+a;

            #ifdef EB_DEBUG
            cv::circle(eb_feature_ptr->eb_mats.refine_liens_image,
                                cv::Point(x,y),
                                2,
                                cv::Scalar(0,0,0),
                                2,
                                CV_AA);
            #endif
            lines_vec[i].point_end.dx = x;
            lines_vec[i].point_end.dy = y;
            lines_vec[next].point_beg.dx = x;
            lines_vec[next].point_beg.dy = y;
            final_roof_vec.push_back(lines_vec[i]);
        }
        else
        {
            final_roof_vec.push_back(lines_vec[i]);
            eb_final_line_t new_line;
            new_line.point_beg = lines_vec[i].point_end;
            new_line.point_end = lines_vec[next].point_beg;
            double dir[2];
            direction_of_line(&new_line,dir);
            final_roof_vec.push_back(new_line);
        }


    }
    final_roof_vec[0].point_beg.dx = 
        final_roof_vec[final_roof_vec.size()-1].point_end.dx;
    final_roof_vec[0].point_beg.dy = 
        final_roof_vec[final_roof_vec.size()-1].point_end.dy;

    #if 0
    for(std::vector<eb_final_line_t>::iterator iter = final_roof_vec.begin(); iter!= final_roof_vec.end(); iter++)
    {
        eb_final_line_t tmp_final_line = *iter;
        double tmp = length_of_line(&tmp_final_line);

        if(tmp < eb_config_ptr->min_length_of_line)
        {
            final_roof_vec.erase(iter);
            iter--;
        }
    }
    #endif

    roof->basic_poly.line_size = final_roof_vec.size();
    roof->basic_poly.lines = (eb_final_line_t *)malloc(sizeof(eb_final_line_t) * final_roof_vec.size());
    for(int i = 0; i < final_roof_vec.size(); i++)
    {
        roof->basic_poly.lines[i] = final_roof_vec[i];
        roof->basic_poly.lines[i].poly_index = i;
        #ifdef EB_DEBUG
        EB_LOG("[EB_DEBUG::] final line %d-->delau_idx:%d %d direct:%lf %lf g_x %lf g_y %lf g_z %lf\n",
                i,
                roof->basic_poly.lines[i].point_beg.delau_pois_idx,
                roof->basic_poly.lines[i].point_end.delau_pois_idx,
                roof->basic_poly.lines[i].direct[0],
                roof->basic_poly.lines[i].direct[1],
                roof->basic_poly.lines[i].point_beg.point_x,
                roof->basic_poly.lines[i].point_beg.point_y,
                roof->basic_poly.lines[i].point_beg.point_z);
        #endif
        cv::line(eb_feature_ptr->eb_mats.close_lines_image,
                    cv::Point(roof->basic_poly.lines[i].point_beg.dx,
                        roof->basic_poly.lines[i].point_beg.dy),
                    cv::Point(roof->basic_poly.lines[i].point_end.dx,
                        roof->basic_poly.lines[i].point_end.dy),
                    cv::Scalar(0,255,0),
                    3,
                    CV_AA);
        
        cv::circle(eb_feature_ptr->eb_mats.close_lines_image,
                    cv::Point(roof->basic_poly.lines[i].point_beg.dx,
                        roof->basic_poly.lines[i].point_beg.dy),
                    3,
                    cv::Scalar(0,0,0),
                    1,
                    CV_AA);
    }
}

#endif

static void save_CNN_roof_pois(eb_roof_t *roof,eb_config_t *eb_config_ptr)
{
    char final_path[256];
    snprintf(final_path,strlen(eb_config_ptr->file_config.out_path)+1,eb_config_ptr->file_config.out_path);
    snprintf(final_path+strlen(final_path),16,"/final_pois.txt");
    FILE *final_poi_fp = fopen(final_path,"wb");
    for(int i = 0; i < roof->basic_poly.line_size; i++)
    {
        roof->basic_poly.lines[i].point_beg.point_x = roof->basic_poly.lines[i].point_beg.dx;
        roof->basic_poly.lines[i].point_beg.point_y = roof->basic_poly.lines[i].point_beg.dy;
        double geo_x = roof->basic_poly.lines[i].point_beg.point_x;
        double geo_y = roof->basic_poly.lines[i].point_beg.point_y;
        double geo_z = roof->basic_poly.lines[i].point_beg.point_z;
        fprintf(final_poi_fp,"%lf %lf %lf\n",geo_x,geo_y,geo_z);
    }
    fclose(final_poi_fp);
}

static void refine_line_with_direc(eb_features_t *eb_feature_ptr,
                                    std::vector<eb_final_line_t> &tmp_lines_vec,
                                std::vector<eb_final_line_t> &refine_lines_vec,
                                eb_config_t *eb_config_ptr,
                                eb_roof_t *roof)
{
    eb_final_line_t refine_line;
    bool new_line = 1;
    EB_LOG("\n");
    for(int i = 0; i < tmp_lines_vec.size(); i++)
    {
        if(new_line)refine_line.point_beg = tmp_lines_vec[i].point_beg;
        int next = i == tmp_lines_vec.size()-1?0:i+1;
        if(fabs(tmp_lines_vec[i].direct[0] - tmp_lines_vec[next].direct[0]) < MIN_DISPARITY &&
            fabs(tmp_lines_vec[i].direct[1] - tmp_lines_vec[next].direct[1]) < MIN_DISPARITY)
            {
                float a = tmp_lines_vec[i].point_beg.dx;
                float b = tmp_lines_vec[i].point_beg.dy;
                float c = tmp_lines_vec[i].point_end.dx;
                float d = tmp_lines_vec[i].point_end.dy;
                float a1 = tmp_lines_vec[next].point_beg.dx;
                float b1 = tmp_lines_vec[next].point_beg.dy;
                float c1 = tmp_lines_vec[next].point_end.dx;
                float d1 = tmp_lines_vec[next].point_end.dy;

                float dis = 0.;
                if(fabs(a-c) < MIN_DISPARITY)dis = fabs(a-a1);
                else
                {
                    float C1 = b-(b-d)/(a-c) * a;
                    float C2 = b1-(b1-d1)/(a1-c1) * a1;
                    float A = (b-d)/(a-c);
                    float B = -1.;
                    dis = fabs(C1-C2)/sqrt(A*A+1);
                }
                if(dis < eb_config_ptr->min_adsorb_dis*eb_config_ptr->refine_trd)
                {   
                    EB_LOG("[EB_DEBUG::] lines refine -->same dire and close %d\n",i);
                    if(next == 0)
                    {
                        refine_lines_vec[0].point_beg = refine_line.point_beg;
                        continue;
                    }    
                    refine_line.point_end = tmp_lines_vec[next].point_end;
                    new_line = 0;
                    continue;
                }
                else
                {
                    EB_LOG("[EB_DEBUG::] lines refine -->same dire and not close %d\n",i);
                    refine_line.point_end = tmp_lines_vec[i].point_end;
                    refine_lines_vec.push_back(refine_line);
                    new_line = 1;
                    continue;
                }
            }
            else
            {
                EB_LOG("[EB_DEBUG::] lines refine -->diff dirct %d\n",i);
                refine_line.point_end = tmp_lines_vec[i].point_end;
                refine_lines_vec.push_back(refine_line);
                new_line = 1;
                continue;
            }
    }

    EB_LOG("\n");
    for(int i =0; i<refine_lines_vec.size(); i++)
    {
        double dir[2];
        direction_of_line(&refine_lines_vec[i],dir);
        refine_lines_vec[i].poly_index = i;
        reset_line_with_direc(&refine_lines_vec[i],
                                roof->roof_direct,eb_config_ptr->min_direct_trd);
        EB_LOG("[EB_DEBUG::] refine line %d-->delau_idx : %d %d direct: %lf %lf\n",
                    i,
                    refine_lines_vec[i].point_beg.delau_pois_idx,
                    refine_lines_vec[i].point_end.delau_pois_idx,
                    refine_lines_vec[i].direct[0],
                    refine_lines_vec[i].direct[1]);
#ifdef TEST_2
        int rgb_r = 255;
        int rgb_g = 255;
        int rgb_b = 255;
#else
        int rgb_r = rand()%255;
        int rgb_g = rand()%255;
        int rgb_b = rand()%255;
#endif
        
        cv::line(eb_feature_ptr->eb_mats.refine_liens_image,
                cv::Point(refine_lines_vec[i].point_beg.dx,refine_lines_vec[i].point_beg.dy),
                cv::Point(refine_lines_vec[i].point_end.dx,refine_lines_vec[i].point_end.dy),
                cv::Scalar(rgb_r,rgb_g,rgb_b),
                3,
                CV_AA);

        #ifdef MID_RESULT
            double trans_res1[2];
            double trans_res2[2];
            trans_dx_dy_to_all(cv::Point(refine_lines_vec[i].point_beg.dx,refine_lines_vec[i].point_beg.dy),
                                eb_config_ptr,
                                trans_res1);
            trans_dx_dy_to_all(cv::Point(refine_lines_vec[i].point_end.dx,refine_lines_vec[i].point_end.dy),
                                eb_config_ptr,
                                trans_res2);
        cv::line(eb_feature_ptr->eb_mats.all_refine_image,
                cv::Point(trans_res1[0],trans_res1[1]),
                cv::Point(trans_res2[0],trans_res2[1]),
                cv::Scalar(rgb_r,rgb_g,rgb_b),
                5,
                CV_AA);
        #endif

        #if 0
        cv::line(eb_feature_ptr->eb_mats.simplify_lines_image,
                cv::Point(refine_lines_vec[i].point_beg.dx,refine_lines_vec[i].point_beg.dy),
                cv::Point(refine_lines_vec[i].point_end.dx,refine_lines_vec[i].point_end.dy),
                cv::Scalar(0,0,255),
                2,
                CV_AA);
        #endif
    }
}

void generate_basic_roof(eb_features_t *eb_features_ptr,eb_config_t *eb_config_ptr,eb_roof_t *roof_ptr)
{
    std::vector<eb_final_line_t> tmp_lines_vec;
    pre_generate_roof(eb_features_ptr,tmp_lines_vec,eb_config_ptr);
    
    #if 1
    for(std::vector<eb_final_line_t>::iterator iter = tmp_lines_vec.begin(); iter!= tmp_lines_vec.end(); iter++)
    {
        eb_final_line_t tmp_final_line = *iter;
        double tmp = length_of_line(&tmp_final_line);

        if(tmp < eb_config_ptr->min_length_of_line)
        {
            tmp_lines_vec.erase(iter);
            iter--;
        }
    }
    #endif


    #if 1
    eb_final_line_t *tmp_line_ptr;
    double tmp_length = 0.0;
    for(int i = 0; i < tmp_lines_vec.size(); i++)
    {
        double tmp = length_of_line(&tmp_lines_vec[i]);
        if(tmp > tmp_length)
        {
            tmp_length = tmp;
            tmp_line_ptr = &tmp_lines_vec[i];
        }
        

    }
    #endif

    

    direction_of_line(tmp_line_ptr,roof_ptr->roof_direct);
    EB_LOG("\n[EB::INFO] the main direction is : (%lf,%lf) \n",
            roof_ptr->roof_direct[0],roof_ptr->roof_direct[1]);

    for(int i = 0; i < tmp_lines_vec.size(); i++)
    {
        reset_line_with_direc(&tmp_lines_vec[i],roof_ptr->roof_direct,eb_config_ptr->min_direct_trd);
        #ifdef EB_DEBUG
        EB_LOG("[EB::DEBUG] reset line %d-->delau_idx : %d %d direct: %lf %lf\n",
                i,
                tmp_lines_vec[i].point_beg.delau_pois_idx,
                tmp_lines_vec[i].point_end.delau_pois_idx,
                tmp_lines_vec[i].direct[0],
                tmp_lines_vec[i].direct[1]);
        #endif

        #ifdef TEST_2
        int rgb_r = 255;
        int rgb_g = 255;
        int rgb_b = 255;
        #else
        int rgb_r = rand()%255;
        int rgb_g = rand()%255;
        int rgb_b = rand()%255;
        #endif
        
        cv::line(eb_features_ptr->eb_mats.reset_lines_image,
                cv::Point(tmp_lines_vec[i].point_beg.dx,tmp_lines_vec[i].point_beg.dy),
                cv::Point(tmp_lines_vec[i].point_end.dx,tmp_lines_vec[i].point_end.dy),
                cv::Scalar(rgb_r,rgb_g,rgb_b),
                3,
                CV_AA);
    }
    std::vector<eb_final_line_t> refine_lines_vec;
    refine_line_with_direc(eb_features_ptr,tmp_lines_vec,refine_lines_vec,eb_config_ptr,roof_ptr);

    EB_LOG("\n[EB::INFO] reset image completed!\n");

    close_roof_lines(refine_lines_vec,eb_config_ptr->min_direct_trd,eb_features_ptr,roof_ptr,eb_config_ptr);
    mat_show("reset_roof_image",eb_features_ptr->eb_mats.reset_lines_image,MAT_SIZE,eb_config_ptr);
    mat_show("refine_roof_image",eb_features_ptr->eb_mats.refine_liens_image,MAT_SIZE,eb_config_ptr);

    if(eb_config_ptr->TYPE == EB_CNN)
        save_CNN_roof_pois(roof_ptr,eb_config_ptr);
    #if 0
    mat_show("simplify_lines_image",eb_features_ptr->eb_mats.simplify_lines_image,MAT_SIZE,eb_config_ptr);
    #endif

    #ifdef TEST_NICE_GROUND
    #else
    mat_show("close_line_images",eb_features_ptr->eb_mats.close_lines_image,MAT_SIZE,eb_config_ptr);
    #endif

    if(eb_config_ptr->TYPE == EB_CNN)
    {
        mat_show("close_line_images",eb_features_ptr->eb_mats.close_lines_image,MAT_SIZE,eb_config_ptr);
    }
    

}

static bool poi_inside_poly(eb_polygon_t *poly, eb_point_t *poi)
{
    bool res = 0;
    for(int i = 0 , j = poly->line_size-1; i <poly->line_size;j=i++)
    {
        if((int)poi->dx == (int)poly->lines[j].point_beg.dx && (int)poi->dy == (int)poly->lines[j].point_beg.dy)
        {
            return true;
        }
        if ( ((poly->lines[i].point_beg.dy >poi->dy) != (poly->lines[j].point_beg.dy>poi->dy)) &&
     (poi->dx < (poly->lines[j].point_beg.dx-poly->lines[i].point_beg.dx) * 
     (poi->dy-poly->lines[i].point_beg.dy) / (poly->lines[j].point_beg.dy-poly->lines[i].point_beg.dy) + 
        poly->lines[i].point_beg.dx) )
        res = !res;
    }
    return res;
}

static bool line_inside_poly(eb_polygon_t *poly, eb_line_t *line)
{
    return poi_inside_poly(poly,&line->point_beg)&&poi_inside_poly(poly,&line->point_end);
}

static void gerate_inside_roof_lns(eb_roof_t *roof_ptr,
                                    eb_features_t *eb_feature_ptr,
                                    cv::Mat &buffer_image,
                                    float trd)
{
    eb_lines_t *hough_lines = &eb_feature_ptr->hough_lines;
    for(int i =0; i < hough_lines->line_size; i++)
    {
        if(line_inside_poly(&roof_ptr->basic_poly,&hough_lines->lines[i]) && 
            !decide_line_within_pcdBoudaryArea_or_not(buffer_image,
                    hough_lines->lines[i],
                    trd))
        {
            cv::line(eb_feature_ptr->eb_mats.roofs_image,
                        cv::Point(hough_lines->lines[i].point_beg.dx,
                            hough_lines->lines[i].point_beg.dy),
                        cv::Point(hough_lines->lines[i].point_end.dx,
                            hough_lines->lines[i].point_end.dy),
                        cv::Scalar(0,0,255),
                        1,
                        CV_AA);
        }
    }
}

static void buble_sort(std::vector<double> &dis_vec,std::vector<double> &val_vec)
{
    for(int i = 0; i < dis_vec.size()-1; i++)
    {
        for(int j =0; j < dis_vec.size()-1 - i; j++)
        {
            if(dis_vec[j] > dis_vec[j+1])
            {
                double tmp;
                double tmp_u;
                tmp = dis_vec[j];
                tmp_u = val_vec[j];
                dis_vec[j] = dis_vec[j+1];
                val_vec[j] = val_vec[j+1];
                dis_vec[j+1] = tmp;  
                val_vec[j+1] = tmp_u;
            }
        }
    }
}

static double lidar_interpola(cv::Mat &image_copy,int col , int row, eb_roof_t *roof_ptr,bool signal)
{
    int count = 0;
    int step = 1;
    
    while(1)
    {
        std::vector<double> dis_vec;
        std::vector<double> val_vec;

        if((row - step < 0) && (row + step >= image_copy.rows) && 
            (col - step<0) && (col + step>=image_copy.cols))
            {
                break;
            }
        int beg_i = row - step < 0? 0:row - step;
        int end_i = row + step >= image_copy.rows? image_copy.rows-1:row + step;

        int beg_j = col - step<0? 0:col - step;
        int end_j = col + step>=image_copy.cols?image_copy.cols-1:col + step;
        for(int i = beg_i; i <= end_i; i++)
        {
            for(int j = beg_j; j <= end_j; j++)
            {
                eb_point_t tmp_poi;
                tmp_poi.dx = j;
                tmp_poi.dy = i;
                if(image_copy.at<double>(i,j) != INVALID_VAL && signal == poi_inside_poly(&roof_ptr->basic_poly,&tmp_poi))
                {
                    
                    val_vec.push_back(image_copy.at<double>(i,j));
                    dis_vec.push_back(sqrt(pow((i-row),2)+pow(j-col,2)));
                    count++;
                }
            }
        }


        if(count >= 4)
        {
            buble_sort(dis_vec,val_vec);
            double k = 0;
            for(int j = 0; j < 4; j++)
            {

                double dis = dis_vec[j];
                k += 1/dis;
            }

            k = 1/k;
            double res = 0.;
            for(int i = 0; i < 4; i++)
            {
                res += 1/dis_vec[i] * k * val_vec[i];
            }

            #if 0
            EB_LOG("[EB_DEBUG::] val is %lf %lf %lf %lf  interpola res is %lf\n",
                        val_vec[0],
                        val_vec[1],
                        val_vec[2],
                        val_vec[3],
                        res);
            #endif
            return res;
            
        }
        else
        {
            count = 0;
            step++;
        }
    }

    return INVALID_VAL;
}

static void generate_lidar_roof(cv::Mat &img,eb_points_t *palnar_pois,eb_roof_t *roof_ptr,eb_config_t *eb_config_ptr,bool signal)
{
    EB_LOG("[EB_DEBUG::] poi_size is %ld\n",palnar_pois->point_size);
    for(int i = 0; i < palnar_pois->point_size; i++)
    {
        int dx = palnar_pois->points[i].dx + 0.5;
        int dy = palnar_pois->points[i].dy + 0.5;
        if(dx<0 || dx >= img.cols || dy<0 || dy >= img.rows)
        {
            continue;
        }
        img.at<double>(dy,dx) = 
                        palnar_pois->points[i].point_z;
        #if 0
        EB_LOG("[EB_DEBUG::] planar poi %d z is %lf\n",i,palnar_pois->points[i].point_z);
        #endif
            
    }
    
    cv::Mat image_copy;
    image_copy = img.clone();
    #ifdef TEST_NICE_GROUND
    if(signal)
    {
        mat_show("lidar_roof_origin",image_copy,MAT_SIZE,eb_config_ptr);
    }
    else
    {
        mat_show("org_lidar_roof_origin",image_copy,MAT_SIZE,eb_config_ptr);
    }
    #else
    mat_show("lidar_roof_origin",image_copy,MAT_SIZE,eb_config_ptr);
    #endif
    
    #if 0
    cv::imwrite("../images/lidar_roof_0.tif",image_copy);
    #endif
    #if 1
    for(int i = 0; i < img.rows;i++)
    {
        for(int j = 0; j < img.cols;j++)
        {
            eb_point_t tmp_poi;
            tmp_poi.dx = j;
            tmp_poi.dy = i;
            if(signal == poi_inside_poly(&roof_ptr->basic_poly,&tmp_poi))
            {
                if(img.at<double>(i,j) == INVALID_VAL)
                {
                    img.at<double>(i,j)=
                    lidar_interpola(image_copy,
                                        j,
                                        i,
                                        roof_ptr,
                                        signal);
                }
            }
            else
            {
                img.at<double>(i,j) = INVALID_VAL;
            }
        }
    }
    #endif
}

void  generate_roofs(eb_features_t *eb_featur_ptr, eb_roof_t *roof_ptr, eb_config_t *eb_config_ptr)
{
    #if 1
    gerate_inside_roof_lns(roof_ptr,
                            eb_featur_ptr,
                            eb_featur_ptr->eb_mats.buffer_image,
                            eb_config_ptr->buffer_filter_f);
    mat_show("roofs_image",eb_featur_ptr->eb_mats.roofs_image,MAT_SIZE,eb_config_ptr);
    #endif

    #if 1
    generate_lidar_roof(eb_featur_ptr->eb_mats.roofs_lidar_image,
                        &eb_featur_ptr->palnar_pois,roof_ptr,eb_config_ptr,true);
    mat_show("lidar_roof",eb_featur_ptr->eb_mats.roofs_lidar_image,MAT_SIZE,eb_config_ptr);
    #ifdef EB_DEBUG

    #ifdef TEST_NICE_GROUND
    generate_lidar_roof(eb_featur_ptr->eb_mats.org_roofs_lidar_image,
                        &eb_featur_ptr->org_palnar_pois,roof_ptr,eb_config_ptr,false);
    mat_show("org_lidar_roof",eb_featur_ptr->eb_mats.org_roofs_lidar_image,MAT_SIZE,eb_config_ptr);
    #endif
    
    cv::Mat dst;
    double angle = acos(fabs(roof_ptr->roof_direct[1])) * 180 / M_PI;
    if(angle < -45)
    {
        angle = -90 - angle; 
    }
    else if(angle > 45)
    {
        angle = 90 -angle;
    }
    angle = roof_ptr->roof_direct[1] > 0? angle : -angle;
    EB_LOG("[EB_DEBUG]::angle rotate is %lf\n",angle);
    rotate(eb_featur_ptr->eb_mats.roofs_lidar_image,dst,
            angle,    
            cv::Point2f(eb_featur_ptr->eb_mats.roofs_lidar_image.cols/2,
                        eb_featur_ptr->eb_mats.roofs_lidar_image.rows/2));
    mat_show("rotate_lidar",dst,MAT_SIZE,eb_config_ptr);
    #if 0
    cv::imwrite("../images/lidar_roof_rotate.tif",dst);
    #endif
    #endif
    #endif

    #ifdef EB_DEBUG
    for(int i = 0; i< roof_ptr->basic_poly.line_size; i++)
    {
        bool test_bool = poi_inside_poly(&roof_ptr->basic_poly,&roof_ptr->basic_poly.lines[0].point_beg);
        EB_LOG("[EB_DEBUG:: ] point %d -->TEST BOOL %d \n",i,test_bool);
    }
    
    #endif
}



