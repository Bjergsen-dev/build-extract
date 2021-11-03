#include "eb_rebuild.hpp"
#include "eb_transform.hpp"
#include "opencv2/opencv.hpp"
#include "polygon.h"
#include "eb_common_defines.hpp"

static void get_geo_coorZ(float dx, float dy, double *geo_z, cv::Mat &roof_lidar_image,double *cloud_z)
{

    uchar color = roof_lidar_image.at<uchar>(dy,dx);
    #ifdef EB_DEBUG
    EB_LOG("[EB_DEBUG::] color :%d ",color);
    #endif

    *geo_z = ((double)color-MIN_PIXEL_VAL)/(MAX_PIXEL_VAL - MIN_PIXEL_VAL)  * (cloud_z[1] - cloud_z[0]) + cloud_z[0];

}

static void reset_geo_coorXY(eb_roof_t *roof,eb_config_t *eb_config_ptr, cv::Mat &roof_lidar_image,double *cloud_z)
{
    int size = roof->basic_poly.line_size;
    eb_final_line_t *lines  = roof->basic_poly.lines;

    for(int i =0 ; i < size; i++)
    {
        get_geoX_geoY_frm_row_column(eb_config_ptr->trans,
                                        lines[i].point_beg.dx,
                                        lines[i].point_beg.dy,
                                        &lines[i].point_beg.point_x,
                                        &lines[i].point_beg.point_y);

        get_geo_coorZ(lines[i].point_beg.dx,
                        lines[i].point_beg.dy,
                        &lines[i].point_beg.point_z,
                        roof_lidar_image,
                        cloud_z);

        #ifdef EB_DEBUG
        EB_LOG("point %d ---> dx: %f dy: %f geo_x: %lf geo_y: %lf geo_z: %lf\n",
                        i,
                        lines[i].point_beg.dx,
                        lines[i].point_beg.dy,
                        lines[i].point_beg.point_x,
                        lines[i].point_beg.point_y,
                        lines[i].point_beg.point_z);
        #endif 
    }
}


void roof_rebuild(eb_roof_t *roof,eb_config_t *eb_config_ptr, cv::Mat &roof_lidar_image,double *cloud_z)
{
    reset_geo_coorXY(roof,eb_config_ptr,roof_lidar_image,cloud_z);

    

#if 0    
    double min_z = INT16_MAX;
    fim::Polygon ori;

    for(int i = 0; i < roof->basic_poly.line_size; i++)
    {
        min_z = min_z < roof->basic_poly.lines[i].point_beg.point_z ? 
                        min_z : roof->basic_poly.lines[i].point_beg.point_z;

        ori.push_back(roof->basic_poly.lines[i].point_beg.point_x,roof->basic_poly.lines[i].point_beg.point_y);

    }

    auto ans = ori.convexDecomposition();
    EB_LOG("[COVEX_SEG::INFO ] segment completed!\n");

#endif

#if 0
    std::vector<std::vector<rebuild_poi_t>> re_pois_v_vec;

    for (auto it = ans.begin(); it != ans.end(); ++it) 
    {
        std::vector<rebuild_poi_t> tmp_vec;
        for(auto itt = (*it).begin(); itt != (*it).end(); ++itt)
        {
            rebuild_poi_t tmp_poi;
            tmp_poi.poi_x = itt->x;
            tmp_poi.poi_y = itt->y;
            tmp_poi.poi_z = min_z;
        }
        re_pois_v_vec.push_back(tmp_vec);
    }
#endif 



    


}
