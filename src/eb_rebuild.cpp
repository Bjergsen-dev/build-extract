#include "eb_rebuild.hpp"
#include "eb_transform.hpp"
#include "opencv2/opencv.hpp"
#include "polygon.h"
#include "eb_common_defines.hpp"

static void get_geo_coorZ(float dx, float dy, double *geo_z, cv::Mat &roof_lidar_image)
{

    double color = roof_lidar_image.at<double>(dy,dx);
    #ifdef EB_DEBUG
    EB_LOG("[EB_DEBUG::] color:%lf ",color);
    #endif

    *geo_z = color;

}

static void reset_geo_coorXY(eb_roof_t *roof,eb_config_t *eb_config_ptr, cv::Mat &roof_lidar_image)
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
                        roof_lidar_image);

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


void vector_roof_rebuild(eb_roof_t *roof,eb_config_t *eb_config_ptr, cv::Mat &roof_lidar_image)
{
    reset_geo_coorXY(roof,eb_config_ptr,roof_lidar_image);

    

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

 static void generate_mtl_file(const char *mtl_path,const char* image_name)
 {
     FILE *mtl_fp = fopen(mtl_path,"wb");
     if(mtl_fp == NULL)
     {
         EB_LOG("[EB::ERROR] open %s failed\n",mtl_path);
         return;
     }

     fprintf(mtl_fp,"newmtl %s_mtl\n",image_name);
     fprintf(mtl_fp,"Ka 0 0 0\n");
     fprintf(mtl_fp,"Kd 1 1 1\n");
     fprintf(mtl_fp,"Ks 0 0 0\n");
     fprintf(mtl_fp,"Ni 1\n");
     fprintf(mtl_fp,"Ns 400\n");
     fprintf(mtl_fp,"Tf 1 1 1\n");
     fprintf(mtl_fp,"d 1\n");
     fprintf(mtl_fp,"map_Kd %s.bmp\n",image_name);

     EB_LOG("[EB::INFO] write to mtl file %s ok!!!!\n",mtl_path);

 } 

 static void generate_obj_file(const char* image_name,
                                const char *pois_path, 
                                const char *context_path,
                                const char *pois_index_path, 
                                const char *obj_path)
 {
     FILE *pois_fp = fopen(pois_path,"rb");
     FILE *context_fp = fopen(context_path,"rb");
     FILE *pois_index_fp = fopen(pois_index_path,"rb");
     FILE *obj_fp = fopen(obj_path,"wb");

     if(pois_fp == NULL)
     {
         EB_LOG("[EB::ERROR] open %s failed\n",pois_path);
         return;
     }

     if(context_fp == NULL)
     {
         EB_LOG("[EB::ERROR] open %s failed\n",context_path);
         return;
     }

     if(pois_index_fp == NULL)
     {
         EB_LOG("[EB::ERROR] open %s failed\n",pois_index_path);
         return;
     }
     if(obj_fp == NULL)
     {
         EB_LOG("[EB::ERROR] open %s failed\n",obj_path);
         return;
     }

     fprintf(obj_fp,"mtllib building.mtl\n");
     double x,y,z;
     double v_x,v_y;
     int a,b,c;
     while(fscanf(pois_fp,"v %lf %lf %lf\n",&x,&y,&z) != EOF)
     {
         fprintf(obj_fp,"v %lf %lf %lf\n",x,y,z);
     }

     while(fscanf(context_fp,"vt %lf %lf\n",&v_x,&v_y) != EOF)
     {
         fprintf(obj_fp,"vt %lf %lf\n",v_x,v_y);
     }

     fprintf(obj_fp,"usemtl %s_mtl\n",image_name);

     while(fscanf(pois_index_fp,"f %d %d %d\n",&a,&b,&c) != EOF)
     {
         fprintf(obj_fp,"f %d/%d %d/%d %d/%d\n",a,a,b,b,c,c);
     }

     fclose(pois_fp);
     fclose(context_fp);
     fclose(pois_index_fp);
     fclose(obj_fp);

     EB_LOG("[EB::INFO] write to obj file %s ok!!!!\n",obj_path);
     

 } 

 void grid_roof_rebuild(eb_roof_t *roof,eb_config_t *eb_config_ptr, cv::Mat &roof_lidar_image)
 {
     char pois_path[256];
     snprintf(pois_path,strlen(eb_config_ptr->file_config.out_path)+1,eb_config_ptr->file_config.out_path);
     snprintf(pois_path+strlen(pois_path),10,"/pois.txt");

     char pois_index_path[256];
     snprintf(pois_index_path,strlen(eb_config_ptr->file_config.out_path)+1,eb_config_ptr->file_config.out_path);
     snprintf(pois_index_path+strlen(pois_index_path),16,"/pois_index.txt");

     char context_path[256];
     snprintf(context_path,strlen(eb_config_ptr->file_config.out_path)+1,eb_config_ptr->file_config.out_path);
     snprintf(context_path+strlen(context_path),13,"/context.txt");

     char mtl_path[256];
     snprintf(mtl_path,strlen(eb_config_ptr->file_config.out_path)+1,eb_config_ptr->file_config.out_path);
     snprintf(mtl_path+strlen(mtl_path),14,"/building.mtl");

     char obj_path[256];
     snprintf(obj_path,strlen(eb_config_ptr->file_config.out_path)+1,eb_config_ptr->file_config.out_path);
     snprintf(obj_path+strlen(obj_path),strlen(eb_config_ptr->file_config.name)+6,"/%s.obj",eb_config_ptr->file_config.name);

     EB_LOG("[EB::INFO] output_path is :\n[EB::INFO] pois_path is : %s\n[EB::INFO] pois_index_path is : %s\n[EB::INFO] context_path is : %s\n[EB::INFO] mtl_path is : %s\n[EB::INFO] obj_path is : %s\n",
                pois_path,
                pois_index_path,
                context_path,
                mtl_path,
                obj_path);
     FILE *pois_fp = fopen(pois_path,"wb");
     FILE *pois_index_fp = fopen(pois_index_path,"wb");
     FILE *context_path_fp = fopen(context_path,"wb");

     if(pois_fp == NULL)
     {
         EB_LOG("[EB::ERROR] open %s failed\n",pois_path);
         return;
     }

     if(pois_index_fp == NULL)
     {
         EB_LOG("[EB::ERROR] open %s failed\n",pois_index_path);
         return;
     }

     if(context_path_fp == NULL)
     {
         EB_LOG("[EB::ERROR] open %s failed\n",context_path_fp);
         return;
     }

     int former_pois_num = 0;

     for(int i = 0; i < roof_lidar_image.cols - eb_config_ptr->rebud_density_x; i += eb_config_ptr->rebud_density_x)
     {
         for(int j = 0; j < roof_lidar_image.rows - eb_config_ptr->rebud_density_y; j += eb_config_ptr->rebud_density_y)
         {
             if(roof_lidar_image.at<double>(j,i) != 0. && 
                    roof_lidar_image.at<double>(j + eb_config_ptr->rebud_density_y,i) != 0. &&
                    roof_lidar_image.at<double>(j,i + eb_config_ptr->rebud_density_x) != 0. &&
                    roof_lidar_image.at<double>(j+eb_config_ptr->rebud_density_y,i+eb_config_ptr->rebud_density_x) != 0.)
             {
                 double geo_x,geo_y; 
                 get_geoX_geoY_frm_row_column(eb_config_ptr->trans,i,j,&geo_x,&geo_y);
                 fprintf(pois_fp,"v %lf %lf %lf\n",geo_x,geo_y,roof_lidar_image.at<double>(j,i));
                 fprintf(context_path_fp,"vt %lf %lf\n",(double)i/(double)roof_lidar_image.cols,
                                                        1. - (double)j/(double)roof_lidar_image.rows);

                 get_geoX_geoY_frm_row_column(eb_config_ptr->trans,
                                                i +eb_config_ptr->rebud_density_x,j,&geo_x,&geo_y);
                 fprintf(pois_fp,"v %lf %lf %lf\n",geo_x,geo_y,
                                    roof_lidar_image.at<double>(j,i +eb_config_ptr->rebud_density_x));
                 fprintf(context_path_fp,"vt %lf %lf\n",(double)(i +eb_config_ptr->rebud_density_x)/(double)roof_lidar_image.cols,
                                                        1. - (double)j/(double)roof_lidar_image.rows);

                 get_geoX_geoY_frm_row_column(eb_config_ptr->trans,
                                                i +eb_config_ptr->rebud_density_x,
                                                j+eb_config_ptr->rebud_density_y,&geo_x,&geo_y);
                 fprintf(pois_fp,"v %lf %lf %lf\n",geo_x,geo_y,
                                    roof_lidar_image.at<double>(j+eb_config_ptr->rebud_density_y,
                                                                i +eb_config_ptr->rebud_density_x));
                 fprintf(context_path_fp,"vt %lf %lf\n",(double)(i +eb_config_ptr->rebud_density_x)/(double)roof_lidar_image.cols,
                                                    1.- (double)(j+eb_config_ptr->rebud_density_y)/(double)roof_lidar_image.rows);

                 get_geoX_geoY_frm_row_column(eb_config_ptr->trans,
                                                i,j+eb_config_ptr->rebud_density_y,&geo_x,&geo_y);
                 fprintf(pois_fp,"v %lf %lf %lf\n",geo_x,geo_y,
                                    roof_lidar_image.at<double>(j+eb_config_ptr->rebud_density_y,i));
                 fprintf(context_path_fp,"vt %lf %lf\n",(double)i/(double)roof_lidar_image.cols,
                                                        1.-(double)(j+eb_config_ptr->rebud_density_y)/(double)roof_lidar_image.rows);

                 fprintf(pois_index_fp,"f %d %d %d\n",former_pois_num+1,former_pois_num+4,former_pois_num+2);
                 fprintf(pois_index_fp,"f %d %d %d\n",former_pois_num+2,former_pois_num+4,former_pois_num+3);

                 former_pois_num += 4;

                 

             }
         }
     }

     fclose(pois_fp);
     fclose(pois_index_fp);
     fclose(context_path_fp);
     EB_LOG("[EB::INFO] grid roofs info has been staged!\n");

     generate_mtl_file(mtl_path,eb_config_ptr->file_config.name);
     generate_obj_file(eb_config_ptr->file_config.name,pois_path,context_path,pois_index_path,obj_path);

 }
