#include "eb_rebuild.hpp"
#include "eb_transform.hpp"
#include "opencv2/opencv.hpp"
#include "polygon.h"
#ifdef TEST_DELAUNAY
#include "delaunayT.hpp"
#endif
#include "eb_common_defines.hpp"
#include "eb_images_detect.hpp"

static int check_clock_wise(eb_polygon_t *basic_poly)
{
    int beg = 0;
    int mid = 1;
    int end = 2;
    double max_y = -INT16_MAX;
    for(int i = 0 ; i < basic_poly->line_size; i++)
    {
        if(max_y < basic_poly->lines[i].point_beg.point_y)
        {
            max_y = basic_poly->lines[i].point_beg.point_y;
            mid = i;
        }
        
    }

    if(mid == 0)
    {
        beg = basic_poly->line_size-1;
        end = 1;
    }
    else if(mid == basic_poly->line_size-1)
    {
        beg = basic_poly->line_size-2;
        end = 0;
    }
    else
    {
        end = mid+1;
        beg = mid-1;
    }

    EB_LOG("[EB_DEBUG::] CLOCK_WISE beg mid end is %d %d %d\n",beg,mid,end);

    double s = (basic_poly->lines[beg].point_beg.point_x - basic_poly->lines[end].point_beg.point_x)*
                    (basic_poly->lines[mid].point_beg.point_y - basic_poly->lines[end].point_beg.point_y)-
                (basic_poly->lines[beg].point_beg.point_y - basic_poly->lines[end].point_beg.point_y)*
                    (basic_poly->lines[mid].point_beg.point_x - basic_poly->lines[end].point_beg.point_x);

    if(s > 0) return -1;
    return 1;
}

static void save_roof_pois(eb_roof_t *roof,eb_config_t *eb_config_ptr)
{
    char final_path[256];
    snprintf(final_path,strlen(eb_config_ptr->file_config.out_path)+1,eb_config_ptr->file_config.out_path);
    snprintf(final_path+strlen(final_path),16,"/final_pois.txt");
    FILE *final_poi_fp = fopen(final_path,"wb");
    for(int i = 0; i < roof->basic_poly.line_size; i++)
    {
        double geo_x = roof->basic_poly.lines[i].point_beg.point_x;
        double geo_y = roof->basic_poly.lines[i].point_beg.point_y;
        double geo_z = roof->basic_poly.lines[i].point_beg.point_z;
        fprintf(final_poi_fp,"%lf %lf %lf\n",geo_x,geo_y,geo_z);
    }
    fclose(final_poi_fp);
}

static void get_geo_coorZ(float dx, float dy, double *geo_z, cv::Mat &roof_lidar_image)
{

    double color = roof_lidar_image.at<double>(dy,dx);
    #ifdef EB_DEBUG
    EB_LOG("[EB_DEBUG::] color:%lf ",color);
    #endif

    *geo_z = color;

}

static double reset_geo_coorXY(eb_roof_t *roof,eb_config_t *eb_config_ptr, cv::Mat &roof_lidar_image)
{
    int size = roof->basic_poly.line_size;
    eb_final_line_t *lines  = roof->basic_poly.lines;
    double min_z = INT16_MAX;
    for(int i =0 ; i < size; i++)
    {
        get_geoX_geoY_frm_row_column(eb_config_ptr->trans,
                                        lines[i].point_beg.dx,
                                        lines[i].point_beg.dy,
                                        &lines[i].point_beg.point_x,
                                        &lines[i].point_beg.point_y);

        float dx = lines[i].point_beg.dx > 0? lines[i].point_beg.dx:0;
        float dy = lines[i].point_beg.dy > 0? lines[i].point_beg.dy:0;
        get_geo_coorZ(  dx,
                        dy,
                        &lines[i].point_beg.point_z,
                        roof_lidar_image);

        if(lines[i].point_beg.point_z != INVALID_VAL && lines[i].point_beg.point_z != 0.)
        {
            min_z = min_z > lines[i].point_beg.point_z?lines[i].point_beg.point_z:min_z;
        }
        
        

        #ifdef EB_DEBUG
        EB_LOG("[EB::DEBUG] reset roof corner %d ---> dx: %f dy: %f geo_x: %lf geo_y: %lf geo_z: %lf\n",
                        i,
                        lines[i].point_beg.dx,
                        lines[i].point_beg.dy,
                        lines[i].point_beg.point_x,
                        lines[i].point_beg.point_y,
                        lines[i].point_beg.point_z);
        #endif 
    }

    return min_z;
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

#if 1
 static void save_ground_elevation(std::vector<eb_point_t> &tmp_vec,cv::Mat &org_roof_lidar_image,
                                    cv::Mat &roof_image,eb_config_t *config_ptr)
 {
     char pois_path[256];
     snprintf(pois_path,strlen(config_ptr->file_config.out_path)+1,config_ptr->file_config.out_path);
     snprintf(pois_path+strlen(pois_path),14,"/grd_pois.txt");
     FILE *out_pois_fp = fopen(pois_path,"wb");

     for(int i = 0; i < tmp_vec.size(); i++)
     {
         int next = i == tmp_vec.size()-1? 0 : i+1;
         float length = EB_LENGTH(tmp_vec[i].point_x,tmp_vec[next].point_x,
                            tmp_vec[i].point_y,tmp_vec[next].point_y);

         for(float j = 0.; j < length; j = j+0.5)
         {
             float poi_x = tmp_vec[i].point_x + (tmp_vec[next].point_x - tmp_vec[i].point_x)*(j/length);
             float poi_y = tmp_vec[i].point_y + (tmp_vec[next].point_y - tmp_vec[i].point_y)*(j/length);

             int dx = get_row_column_frm_geoX_geoY(config_ptr->trans,poi_x,poi_y,1);
             int dy = get_row_column_frm_geoX_geoY(config_ptr->trans,poi_x,poi_y,2);
             
             if(dx <0 || dx >= org_roof_lidar_image.cols || dy < 0 || dy >= org_roof_lidar_image.rows)
             continue;
            
             #if 0
             cv::circle(roof_image,cv::Point2d(dx,dy),2,cv::Scalar(0,0,255),2,CV_AA); 
             #endif
             float poi_z = org_roof_lidar_image.at<double>(dy,dx);
             

             if(-500<poi_z && poi_z < 500)
             fprintf(out_pois_fp,"%f %f %f\n",poi_x,poi_y,poi_z);

         }
     }
     mat_show("close_line_images",roof_image,MAT_SIZE,config_ptr);

     fclose(out_pois_fp);
     
 }
#endif


 static void resize_basic_roof(eb_roof_t *roof,std::vector<eb_point_t> &tmp_vec,double roof_trd,int type)
 {
     for(int i = 0; i < roof->basic_poly.line_size; i++)
     {
         int left = i==0?roof->basic_poly.line_size-1:i-1;
         double a = roof->basic_poly.lines[i].direct[0];
         double b = -roof->basic_poly.lines[i].direct[1];

         double c = -roof->basic_poly.lines[left].direct[0];
         double d = roof->basic_poly.lines[left].direct[1];
        
         #if 0
         double cos_t = a*c + b*d;
         double sin_t = sqrt(1- cos_t*cos_t);
         #endif

         double sin_t = (-a*d+c*b)*type;

         double new_x = roof->basic_poly.lines[i].point_beg.point_x + (roof_trd/sin_t)*(a+c);
         double new_y = roof->basic_poly.lines[i].point_beg.point_y + (roof_trd/sin_t)*(b+d);

         eb_point_t poi = roof->basic_poly.lines[i].point_beg;
         poi.point_x = new_x;
         poi.point_y = new_y;
         tmp_vec.push_back(poi);

     }
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


     fprintf(mtl_fp,"newmtl no_context_mtl\n");

     EB_LOG("[EB::INFO] write to mtl file %s ok!!!!\n",mtl_path);

 } 

 static void generate_obj_file(const char* image_name,
                                const char *pois_path, 
                                const char *context_path,
                                const char *pois_index_path, 
                                const char *obj_path,
                                int mtl_change_idx)
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

     int tmp = 0;
     while(fscanf(pois_index_fp,"f %d %d %d\n",&a,&b,&c) != EOF)
     {
         tmp++;
         fprintf(obj_fp,"f %d/%d %d/%d %d/%d\n",a,a,b,b,c,c);

         if(tmp == mtl_change_idx)
         {
             fprintf(obj_fp,"usemtl no_context_mtl\n");
         }
     }

     fclose(pois_fp);
     fclose(context_fp);
     fclose(pois_index_fp);
     fclose(obj_fp);

     EB_LOG("[EB::INFO] write to obj file %s ok!!!!\n",obj_path);
     

 } 

 static void generate_final_info(eb_roof_t *roof, eb_config_t * config)
 {
     roof->basic_poly_info.poi_size = roof->basic_poly.line_size;
     std::vector<cv::Point2f> approx;
     for(int i = 0; i < roof->basic_poly.line_size; i++)
     {
         approx.push_back(cv::Point2f(roof->basic_poly.lines[i].point_beg.dx,
                                        roof->basic_poly.lines[i].point_beg.dy));
         
     }
     roof->basic_poly_info.area_size = RESOLUTION*RESOLUTION*fabs(cv::contourArea(approx,true));
     EB_LOG("[EB::INFO] %s final info: Extract_time is %lf Rebuild_time is %lf Area_size is %lf Poi_size is %d\n",
                config->file_config.name, roof->basic_poly_info.eb_time, roof->basic_poly_info.rebuild_time,
                roof->basic_poly_info.area_size, roof->basic_poly_info.poi_size);
 }

 void grid_roof_rebuild(eb_roof_t *roof,eb_config_t *eb_config_ptr, 
                        cv::Mat &roof_lidar_image,cv::Mat &grd_roof_lidar_image,cv::Mat &close_line_image)
 {
     #ifdef TEST_NICE_GROUND
     std::vector<eb_point_t> grd_tmp_pois;
     resize_basic_roof(roof,grd_tmp_pois,eb_config_ptr->roof_resize,-1);
     save_ground_elevation(grd_tmp_pois,grd_roof_lidar_image,close_line_image,eb_config_ptr);
     #endif

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
     #ifdef RB_NO_ROOF
     snprintf(obj_path+strlen(obj_path),strlen(eb_config_ptr->file_config.name)+11,"/%s_base.obj",eb_config_ptr->file_config.name);
     #endif

     #ifdef RB_NO_BASE
     snprintf(obj_path+strlen(obj_path),strlen(eb_config_ptr->file_config.name)+11,"/%s_roof.obj",eb_config_ptr->file_config.name);
     #endif

     #ifndef RB_NO_ROOF
     #ifndef RB_NO_BASE
     snprintf(obj_path+strlen(obj_path),strlen(eb_config_ptr->file_config.name)+6,"/%s.obj",eb_config_ptr->file_config.name);
     #endif
     #endif

     

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
     int mtl_change_idx = 0;

     #ifndef RB_NO_ROOF
     for(int i = 0; i < roof_lidar_image.cols - eb_config_ptr->rebud_density_x; i += eb_config_ptr->rebud_density_x)
     {
         for(int j = 0; j < roof_lidar_image.rows - eb_config_ptr->rebud_density_y; j += eb_config_ptr->rebud_density_y)
         {
            
                
             if(roof_lidar_image.at<double>(j,i) != INVALID_VAL && 
                    roof_lidar_image.at<double>(j + eb_config_ptr->rebud_density_y,i) != INVALID_VAL &&
                    roof_lidar_image.at<double>(j,i + eb_config_ptr->rebud_density_x) != INVALID_VAL &&
                    roof_lidar_image.at<double>(j+eb_config_ptr->rebud_density_y,i+eb_config_ptr->rebud_density_x) != INVALID_VAL)
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

                 mtl_change_idx += 2;
                 former_pois_num += 4;

                 

             }
         }
     }
     #endif

    //To record the time in EB process.
    clock_t eb_time = clock();
    EB_LOG("[EB::INFO] Rebuild the roof finished! time is %ld\n",eb_time);

     #ifndef RB_NO_BASE
     double min_z = reset_geo_coorXY(roof,eb_config_ptr,roof_lidar_image);
     EB_LOG("[EB_DEBUG::] MIN_Z IS %lf \n",min_z);
     save_roof_pois(roof,eb_config_ptr);

     //wall polys
     std::vector<eb_point_t> tmp_pois;
     int clock_wise = check_clock_wise(&roof->basic_poly);
     EB_LOG("[EB::DEBUG] ---------------->CLOCK_WAISE IS %d\n",clock_wise);
     resize_basic_roof(roof,tmp_pois,eb_config_ptr->roof_resize,clock_wise);

     for(int i = 0; i < tmp_pois.size(); i++)
     {
         int next = i == roof->basic_poly.line_size-1?0:i+1;
         fprintf(pois_fp,"v %lf %lf %lf\n",
                    tmp_pois[i].point_x,
                    tmp_pois[i].point_y,
                    #ifdef RB_NO_ROOF
                    min_z);
                    #else
                    tmp_pois[i].point_z);
                    #endif

         fprintf(context_path_fp,"vt 0 0\n");

         fprintf(pois_fp,"v %lf %lf %lf\n",
                    tmp_pois[next].point_x,
                    tmp_pois[next].point_y,
                    #ifdef RB_NO_ROOF
                    min_z);
                    #else
                    tmp_pois[next].point_z);
                    #endif

         fprintf(context_path_fp,"vt 0 0\n");

         fprintf(pois_fp,"v %lf %lf %lf\n",
                    tmp_pois[next].point_x,
                    tmp_pois[next].point_y,
                    #ifdef NO_FIT_GROUND
                    eb_config_ptr->ground_plane[0]
                    #else
                    eb_config_ptr->ground_plane[0]*tmp_pois[next].point_x+
                    eb_config_ptr->ground_plane[1]*tmp_pois[next].point_y+
                    eb_config_ptr->ground_plane[2]
                    #endif
                    );

         fprintf(context_path_fp,"vt 0 0\n");

         fprintf(pois_fp,"v %lf %lf %lf\n",
                    tmp_pois[i].point_x,
                    tmp_pois[i].point_y,
                    #ifdef NO_FIT_GROUND
                    eb_config_ptr->ground_plane[0]
                    #else
                    eb_config_ptr->ground_plane[0]*tmp_pois[i].point_x+
                    eb_config_ptr->ground_plane[1]*tmp_pois[i].point_y+
                    eb_config_ptr->ground_plane[2]
                    #endif
                    );

         fprintf(context_path_fp,"vt 0 0\n");

         

         fprintf(pois_index_fp,"f %d %d %d\n",former_pois_num+1,former_pois_num+2,former_pois_num+3);
         fprintf(pois_index_fp,"f %d %d %d\n",former_pois_num+3,former_pois_num+4,former_pois_num+1);

         former_pois_num += 4;
         


     }
     #endif

     #ifndef RB_NO_BASE
     #ifdef TEST_DELAUNAY
     DelaunayT delaunayT(tmp_pois);
     delaunayT.generateAlphaShape();
     std::vector<Triangle> tris = delaunayT.getTri();
     EB_LOG("[EB_DEBUG::] Delaunay tris num is %ld\n",tris.size());
     //ground
     for(int i = 0; i < tris.size(); i++)
     {
          
         fprintf(pois_fp,"v %lf %lf %lf\n",
                    tris[i].p1_[0],
                    tris[i].p1_[1],
                    eb_config_ptr->ground_plane[0]*tris[i].p1_[0]+
                    eb_config_ptr->ground_plane[1]*tris[i].p1_[1]+
                    eb_config_ptr->ground_plane[2]);
        fprintf(context_path_fp,"vt 0 0\n");
        fprintf(pois_fp,"v %lf %lf %lf\n",
                    tris[i].p2_[0],
                    tris[i].p2_[1],
                    eb_config_ptr->ground_plane[0]*tris[i].p1_[0]+
                    eb_config_ptr->ground_plane[1]*tris[i].p1_[1]+
                    eb_config_ptr->ground_plane[2]);
        fprintf(context_path_fp,"vt 0 0\n");
        fprintf(pois_fp,"v %lf %lf %lf\n",
                    tris[i].p3_[0],
                    tris[i].p3_[1],
                    eb_config_ptr->ground_plane[0]*tris[i].p1_[0]+
                    eb_config_ptr->ground_plane[1]*tris[i].p1_[1]+
                    eb_config_ptr->ground_plane[2]);
        fprintf(context_path_fp,"vt 0 0\n");

        fprintf(pois_index_fp,"f %d %d %d\n",former_pois_num+1,former_pois_num+2,former_pois_num+3);
        former_pois_num += 3;
     }
     //basic roof
     for(int i = 0; i < tris.size(); i++)
     {
          
         fprintf(pois_fp,"v %lf %lf %lf\n",
                    tris[i].p1_[0],
                    tris[i].p1_[1],
                    min_z);
        fprintf(context_path_fp,"vt 0 0\n");
        fprintf(pois_fp,"v %lf %lf %lf\n",
                    tris[i].p2_[0],
                    tris[i].p2_[1],
                    min_z);
        fprintf(context_path_fp,"vt 0 0\n");
        fprintf(pois_fp,"v %lf %lf %lf\n",
                    tris[i].p3_[0],
                    tris[i].p3_[1],
                    min_z);
        fprintf(context_path_fp,"vt 0 0\n");

        fprintf(pois_index_fp,"f %d %d %d\n",former_pois_num+1,former_pois_num+2,former_pois_num+3);
        former_pois_num += 3;
     }
     #else
     fim::Polygon ori;
     for(int i = 0; i < tmp_pois.size(); i++)
     {
         ori.push_back(fim::vec2(tmp_pois[i].point_x,
                        clock_wise* tmp_pois[i].point_y));
     }

    auto ans = ori.convexDecomposition();
    EB_LOG("[EB_DEBUG::] ans's num is %ld pois num is %ld\n",ans.size(),ori.size());

    for (auto it = ans.begin(); it != ans.end(); ++it) 
    {
        int num = 0;
        for(auto itt = (*it).begin(); itt != (*it).end(); ++itt)
        {
            fprintf(pois_fp,"v %lf %lf %lf\n",
                    itt->x,
                    clock_wise * itt->y,
                    #ifdef NO_FIT_GROUND
                    eb_config_ptr->ground_plane[0]
                    #else
                    eb_config_ptr->ground_plane[0]*itt->x+
                    eb_config_ptr->ground_plane[1]*(itt->y)+
                    eb_config_ptr->ground_plane[2]
                    #endif
                    );
            fprintf(context_path_fp,"vt 0 0\n");
            num++;
        }
        for(int j = 0; j < num-2;j++)
        {
            fprintf(pois_index_fp,"f %d %d %d\n",former_pois_num+1,former_pois_num+2+j,former_pois_num+3+j);
        }
        former_pois_num += num;
    }

    eb_time = clock();
    EB_LOG("[EB::INFO] Rebuild the base finished! time is %ld\n",eb_time);

    roof->basic_poly_info.rebuild_time = (double) eb_time/CLOCKS_PER_SEC - roof->basic_poly_info.eb_time;

    #ifdef RB_NO_ROOF
    for (auto it = ans.begin(); it != ans.end(); ++it) 
    {
        int num = 0;
        for(auto itt = (*it).begin(); itt != (*it).end(); ++itt)
        {
            fprintf(pois_fp,"v %lf %lf %lf\n",
                    itt->x,
                    clock_wise* itt->y,
                    min_z);
            fprintf(context_path_fp,"vt 0 0\n");
            num++;
        }
        for(int j = 0; j < num-2;j++)
        {
            fprintf(pois_index_fp,"f %d %d %d\n",former_pois_num+1,former_pois_num+2+j,former_pois_num+3+j);
        }
        former_pois_num += num;
    }
    #endif
    #endif
    #endif


     fclose(pois_fp);
     fclose(pois_index_fp);
     fclose(context_path_fp);
     EB_LOG("[EB::INFO] grid roofs info has been staged!\n");

     generate_mtl_file(mtl_path,eb_config_ptr->file_config.name);
     generate_obj_file(eb_config_ptr->file_config.name,pois_path,context_path,pois_index_path,obj_path,mtl_change_idx);

     eb_time = clock();
     EB_LOG("[EB::INFO] Generate the Building model files finished! time is : %ld\n",eb_time);

     generate_final_info(roof,eb_config_ptr);
     //The cidx is the index for the Area model generation by the seperate models. 
     char change_idx_path[256];
     snprintf(change_idx_path,strlen(eb_config_ptr->file_config.out_path)+1,eb_config_ptr->file_config.out_path);
     snprintf(change_idx_path+strlen(change_idx_path),10,"/cidx.txt");

     FILE *cidx_fp = fopen(change_idx_path,"w");
     fprintf(cidx_fp,"%d",mtl_change_idx);

 }
