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
    EB_LOG("[EB::INFO] adsorb filter completed!\n\n");
    mat_show("adsorbent_image",eb_futures_ptr->eb_mats.adsorb_filter_image,MAT_SIZE);
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

        }

        for(int i = 0; i < boudry_pois->point_size; i++)
        {
            if(tmp_update_ptr[i].adsorb_num == 0)
            {
                cv::circle(eb_futures_ptr->eb_mats.adsorb_update_image,
                            cv::Point(boudry_pois->points[i].dx,boudry_pois->points[i].dy),
                            2,
                            cv::Scalar(0,0,0),
                            1,
                            CV_AA);
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
            }
        }

        mat_show("adsorb_update_image",
                    eb_futures_ptr->eb_mats.adsorb_update_image,
                    MAT_SIZE);

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

static void pre_generate_roof(eb_features_t *eb_features_ptr,std::vector<eb_final_line_t> &tmp_lines_vec)
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
        if(i==37)
        {
            printf("ass\n");
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
        int rgb_r = rand()%255;
        int rgb_g = rand()%255;
        int rgb_b = rand()%255;
        cv::line(eb_features_ptr->eb_mats.simplify_lines_image,
                cv::Point(tmp_lines_vec[i].point_beg.dx,tmp_lines_vec[i].point_beg.dy),
                cv::Point(tmp_lines_vec[i].point_end.dx,tmp_lines_vec[i].point_end.dy),
                cv::Scalar(rgb_r,rgb_g,rgb_b),
                2,
                CV_AA);
    }

    free(tmp_pois);
    tmp_pois = NULL;

    mat_show("simplify_image",eb_features_ptr->eb_mats.simplify_lines_image,MAT_SIZE);
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
        if(tmp_dire[0] >= 0)c=fabs(dirct[1]);
        if(tmp_dire[0] < 0)c=-fabs(dirct[1]);
        if(tmp_dire[1] >= 0)d=fabs(dirct[0]);
        if(tmp_dire[1] < 0)d=-fabs(dirct[0]);


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
static int parallel_check(eb_final_line_t* line_1, eb_final_line_t *line_2, eb_points_t *delau_budry_pois)
{
    double first_sum = 0.;
    double second_sum = 0.;
    for(int i = line_1->point_end.delau_pois_idx+1; i < line_2->point_beg.delau_pois_idx; i++)
    {
        eb_point_t *poi = &delau_budry_pois->points[i];
        double first_dis = pow((poi->dx - line_1->point_end.dx),2) + 
                            pow((poi->dy - line_1->point_end.dy),2);

        double second_dis = pow((poi->dx - line_2->point_beg.dx),2) + 
                            pow((poi->dy - line_2->point_beg.dy),2);

        first_sum += first_dis;
        second_sum += second_dis;
    }

    if(first_sum == second_sum && first_sum == 0.)
    {
        return 0;
    }

    if(first_sum > second_sum)
    {
        return -1;
    }

    if(first_sum < second_sum)
    {
        return 1;
    }
}


static void close_roof_lines(std::vector<eb_final_line_t> &lines_vec,double dire_trd,eb_features_t *eb_feature_ptr,eb_roof_t *roof)
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
            switch (parallel_check(&lines_vec[i],&lines_vec[next],delau_budry_pois))
            {
            case -1:
                {
                    double c = lines_vec[next].point_beg.dx;
                    double d = lines_vec[next].point_beg.dy;
                    double a = lines_vec[i].point_end.dx;
                    double b = lines_vec[i].point_end.dy;
                    double m = lines_vec[next].direct[0];
                    double n = lines_vec[next].direct[1];
                    
                    double y = b*n*n + m*m*d + m*n*(a-c);
                    double x = m/(n+MIN_DOUBLE) * (y-d) + c;
                    #ifdef EB_DEBUG
                    cv::circle(eb_feature_ptr->eb_mats.reset_lines_image,
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
                    cv::circle(eb_feature_ptr->eb_mats.reset_lines_image,
                                cv::Point(x1,y1),
                                2,
                                cv::Scalar(0,255,0),
                                2,
                                CV_AA);
                    cv::circle(eb_feature_ptr->eb_mats.reset_lines_image,
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
                    cv::circle(eb_feature_ptr->eb_mats.reset_lines_image,
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
            cv::circle(eb_feature_ptr->eb_mats.reset_lines_image,
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

    roof->basic_poly.line_size = final_roof_vec.size();
    roof->basic_poly.lines = (eb_final_line_t *)malloc(sizeof(eb_final_line_t) * final_roof_vec.size());
    for(int i = 0; i < final_roof_vec.size(); i++)
    {
        roof->basic_poly.lines[i] = final_roof_vec[i];
        roof->basic_poly.lines[i].poly_index = i;
        #ifdef EB_DEBUG
        EB_LOG("[EB_DEBUG::] final line %d-->delau_idx:%d %d direct:%lf %lf\n",
                i,
                roof->basic_poly.lines[i].point_beg.delau_pois_idx,
                roof->basic_poly.lines[i].point_end.delau_pois_idx,
                roof->basic_poly.lines[i].direct[0],
                roof->basic_poly.lines[i].direct[1]);
        #endif
        cv::line(eb_feature_ptr->eb_mats.close_lines_image,
                    cv::Point(roof->basic_poly.lines[i].point_beg.dx,
                        roof->basic_poly.lines[i].point_beg.dy),
                    cv::Point(roof->basic_poly.lines[i].point_end.dx,
                        roof->basic_poly.lines[i].point_end.dy),
                    cv::Scalar(0,255,0),
                    2,
                    CV_AA);
    }
}

#endif

void generate_basic_roof(eb_features_t *eb_features_ptr,eb_config_t *eb_config_ptr,eb_roof_t *roof_ptr)
{
    std::vector<eb_final_line_t> tmp_lines_vec;
    pre_generate_roof(eb_features_ptr,tmp_lines_vec);

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

    direction_of_line(tmp_line_ptr,roof_ptr->roof_direct);
    EB_LOG("[EB::INFO] the main direction is : (%lf,%lf) \n",
            roof_ptr->roof_direct[0],roof_ptr->roof_direct[1]);

    for(int i = 0; i < tmp_lines_vec.size(); i++)
    {
        reset_line_with_direc(&tmp_lines_vec[i],roof_ptr->roof_direct,eb_config_ptr->min_direct_trd);
        #ifdef EB_DEBUG
        EB_LOG("[EB_DEBUG::] line %d-->delau_idx : %d %d direct: %lf %lf\n",
                i,
                tmp_lines_vec[i].point_beg.delau_pois_idx,
                tmp_lines_vec[i].point_end.delau_pois_idx,
                tmp_lines_vec[i].direct[0],
                tmp_lines_vec[i].direct[1]);
        #endif
        int rgb_r = rand()%255;
        int rgb_g = rand()%255;
        int rgb_b = rand()%255;
        cv::line(eb_features_ptr->eb_mats.reset_lines_image,
                cv::Point(tmp_lines_vec[i].point_beg.dx,tmp_lines_vec[i].point_beg.dy),
                cv::Point(tmp_lines_vec[i].point_end.dx,tmp_lines_vec[i].point_end.dy),
                cv::Scalar(rgb_r,rgb_g,rgb_b),
                2,
                CV_AA);
    }

    EB_LOG("\n[EB::INFO] reset image completed!\n");

    close_roof_lines(tmp_lines_vec,eb_config_ptr->min_direct_trd,eb_features_ptr,roof_ptr);
    mat_show("reset_roof_image",eb_features_ptr->eb_mats.reset_lines_image,MAT_SIZE);
    mat_show("close_line_images",eb_features_ptr->eb_mats.close_lines_image,MAT_SIZE);

    

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

static double lidar_interpola(cv::Mat &image_copy,int col , int row, eb_roof_t *roof_ptr)
{
    int count = 0;
    int step = 1;
    
    while(1)
    {
        std::vector<double> dis_vec;
        std::vector<double> val_vec;
        for(int i = row - step; i <= row+step; i++)
        {
            for(int j = col - step; j <= col+step; j++)
            {
                eb_point_t tmp_poi;
                tmp_poi.dx = j;
                tmp_poi.dy = i;
                if(image_copy.at<double>(i,j) != 0. && poi_inside_poly(&roof_ptr->basic_poly,&tmp_poi))
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

    return 0.;
}

static void generate_lidar_roof(eb_features_t *eb_featur_ptr,eb_roof_t *roof_ptr,eb_config_t *eb_config_ptr)
{

    for(int i = 0; i < eb_featur_ptr->palnar_pois.point_size; i++)
    {
        int dx = eb_featur_ptr->palnar_pois.points[i].dx + 0.5;
        int dy = eb_featur_ptr->palnar_pois.points[i].dy + 0.5;
        eb_featur_ptr->eb_mats.roofs_lidar_image.at<double>(dy,dx) = 
                        eb_featur_ptr->palnar_pois.points[i].point_z;
        #if 0
        EB_LOG("[EB_DEBUG::] planar poi %d z is %lf\n",i,eb_featur_ptr->palnar_pois.points[i].point_z);
        #endif
            
    }
    cv::Mat image_copy;
    image_copy = eb_featur_ptr->eb_mats.roofs_lidar_image.clone();
    mat_show("lidar_roof_origin",image_copy,MAT_SIZE);
    #if 0
    cv::imwrite("../images/lidar_roof_0.tif",image_copy);
    #endif
    #if 1
    for(int i = 0; i < eb_featur_ptr->eb_mats.roofs_lidar_image.rows;i++)
    {
        for(int j = 0; j < eb_featur_ptr->eb_mats.roofs_lidar_image.cols;j++)
        {
            eb_point_t tmp_poi;
            tmp_poi.dx = j;
            tmp_poi.dy = i;
            if(poi_inside_poly(&roof_ptr->basic_poly,&tmp_poi))
            {
                if(eb_featur_ptr->eb_mats.roofs_lidar_image.at<double>(i,j) == 0.)
                {
                    eb_featur_ptr->eb_mats.roofs_lidar_image.at<double>(i,j)=
                    lidar_interpola(image_copy,
                                        j,
                                        i,
                                        roof_ptr);
                }
            }
            else
            {
                eb_featur_ptr->eb_mats.roofs_lidar_image.at<double>(i,j) = 0.;
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
    mat_show("roofs_image",eb_featur_ptr->eb_mats.roofs_image,MAT_SIZE);
    #endif

    #if 1
    generate_lidar_roof(eb_featur_ptr,roof_ptr,eb_config_ptr);
    mat_show("lidar_roof",eb_featur_ptr->eb_mats.roofs_lidar_image,MAT_SIZE);
    #ifdef EB_DEBUG
    #if 0
    cv::imwrite("../images/lidar_roof.tif",eb_featur_ptr->eb_mats.roofs_lidar_image);
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
    mat_show("rotate_lidar",dst,MAT_SIZE);
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



