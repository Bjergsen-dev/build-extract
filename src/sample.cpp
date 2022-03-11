#include "delaunayT.hpp"
#include "eb_config.hpp"
#include "eb_common_defines.hpp"
#include "eb_images_detect.hpp"
#include "eb_lidar_detect.hpp"
#include "eb_transform.hpp"
#include "eb_filter.hpp"
#include "eb_rebuild.hpp"

#ifdef TEST_IOU
static bool poi_inside_poly(std::vector<cv::Point> &poly, float x, float y)
{
    bool res = 0;
    for(int i = 0 , j = poly.size()-1; i <poly.size();j=i++)
    {
        if((int)x == (int)poly[j].x && (int)y == (int)poly[j].y)
        {
            return true;
        }
        if ( ((poly[i].y >y) != (poly[j].y>y)) &&
     (x < (poly[j].x-poly[i].x) * 
     (y-poly[i].y) / (poly[j].y-poly[i].y) + 
        poly[i].x) )
        res = !res;
    }
    return res;
}
#endif

int main(
    int argc, char** argv
    ) {

   
    #if 0

    int build_num = 47;
    char *dem_path = "/home/jaxzhong/Exp_data/DEM/ortho__DEM_120_1_4.tiff";
    char *loc_path = "/home/jaxzhong/Exp_data/Area3/result/";

    GDALDataset *poDataset;
    GDALAllRegister();  //注册所有的驱动
    poDataset = (GDALDataset *) GDALOpen(dem_path, GA_ReadOnly );
    if( poDataset == NULL )
    {
        EB_LOG("GDAL::ERROR :%s open failed!\n",dem_path);

    }

    double dem_trans[6];
    getTrans_of_TiffFile(dem_path,dem_trans);

    printf("%lf %lf %lf %lf %lf %lf\n",dem_trans[0],
                                        dem_trans[1],
                                        dem_trans[2],
                                        dem_trans[3],
                                        dem_trans[4],
                                        dem_trans[5]);

    FILE *rd_fp;
    FILE *rd_config_fp;
    FILE *out_fp;


    for(int i = 1 ; i <= build_num; i++)
    {

        char p_name[256];
        snprintf(p_name,strlen(loc_path)+1,loc_path);
        snprintf(p_name+strlen(p_name),24,"build_%d/final_pois.txt",i);
        EB_LOG("[EB_DEBUG:: ] %s\n",p_name);

        char p_out_name[256];
        snprintf(p_out_name,strlen(loc_path)+1,loc_path);
        snprintf(p_out_name+strlen(p_out_name),24,"build_%d/final_eles.txt",i);

        EB_LOG("[EB_DEBUG:: ] %s\n",p_out_name);

        char config_name[256];
        snprintf(config_name,strlen(loc_path)+1,loc_path);
        snprintf(config_name+strlen(config_name),20,"build_%d/config.txt",i);

        EB_LOG("[EB_DEBUG:: ] %s\n",config_name);

        rd_fp = fopen(p_name,"r");
        if(rd_fp == NULL)
        {
            continue;
        }
        out_fp = fopen(p_out_name,"w");

        rd_config_fp = fopen(config_name,"r");
        double gx,gy,ele,dem_ele(0.),pre_ele;
        int num = 0;
        char buf[256];
        for(int j = 28; j > 0; j--)
        {
            fscanf(rd_config_fp, "%[^\n]\n", buf);
          //  printf("%s\n",buf);
        }
        fscanf(rd_config_fp,"ground_plane: %lf\n",&pre_ele);
        

        while(fscanf(rd_fp,"%lf %lf %lf\n",&gx,&gy,&ele) != EOF)
        {
            
            dem_ele += get_elevation_frm_geo_XY(poDataset,gx,gy,dem_trans);
            num++;

            //fprintf(out_fp,"%lf %lf %lf %lf\n",gx,gy,ele,dem_ele);
        }
        dem_ele /= num;

        fprintf(out_fp,"%lf %lf %lf\n",pre_ele,dem_ele,(dem_ele-pre_ele));


        fclose(rd_fp);
        fclose(out_fp);
        fclose(rd_config_fp);



    }
    GDALClose(poDataset);

    return 0;
    #endif


    
    #ifdef TEST_IOU
    int sum = 47;
    char *path = "/home/jaxzhong/Exp_data/Area3/result/";
    char *type = "";
    int dil_size = 13;

    char all_path[256];
    snprintf(all_path,strlen(path)+1,path);
    snprintf(all_path+strlen(all_path),20,"Area3_all/Area3.tif");
    
    char shp_file[256];
    snprintf(shp_file,strlen(path)+1,path);
    snprintf(shp_file+strlen(shp_file),38,"Area3_all/building_outline_area_3.shp");

    char eb_path[256];
    snprintf(eb_path,strlen(path)+1,path);
    #ifdef IOU_BRY
    snprintf(eb_path+strlen(eb_path),27+strlen(type),"Area3_all/%seb_Area3_bry.tif",type);
    #else
    snprintf(eb_path+strlen(eb_path),30,"Area3_all/origin_eb_Area3.tif");
    #endif

    char isprs_path[256];
    snprintf(isprs_path,strlen(path)+1,path);
    #ifdef IOU_BRY
    snprintf(isprs_path+strlen(isprs_path),30,"Area3_all/isprs_Area3_bry.tif");
    #else
    snprintf(isprs_path+strlen(isprs_path),26,"Area3_all/isprs_Area3.tif");
    #endif

    cv::Mat area_image = cv::imread(all_path);
    #ifdef IOU_BRY
    cv::Mat eb_image = cv::Mat(area_image.rows,area_image.cols,CV_8UC1,cv::Scalar(0));
    cv::Mat isprs_image = cv::Mat(area_image.rows,area_image.cols,CV_8UC1,cv::Scalar(0));
    #else
    cv::Mat eb_image = cv::Mat(area_image.rows,area_image.cols,CV_8UC3,cv::Scalar(255,255,255));
    cv::Mat isprs_image = cv::Mat(area_image.rows,area_image.cols,CV_8UC3,cv::Scalar(255,255,255));
    #endif
    double trans[6];
    getTrans_of_TiffFile(all_path,trans);
    std::vector<std::vector<cv::Point>> contours;
    for(int i = 1; i<= sum; i++)
    {
        EB_LOG("[EB_DEBUG::] Build %d in process!\n",i);
        char build_path[256];
        snprintf(build_path,strlen(path)+1,path);
        snprintf(build_path+strlen(build_path),24+strlen(type),"build_%d/%sfinal_pois.txt",i,type);
        FILE *fp = fopen(build_path,"r");
        if(fp == NULL)
        {
            EB_LOG("[EB_ERROR::] open %s failed!\n",build_path);
            //fclose(fp);
            continue;
        }
        double geo_x;
        double geo_y;
        double geo_z;
        std::vector<cv::Point> contour;
        while(fscanf(fp,"%lf %lf %lf",&geo_x,&geo_y,&geo_z) != EOF)
        {
            float dx = get_row_column_frm_geoX_geoY(trans,geo_x,geo_y,1);
            float dy = get_row_column_frm_geoX_geoY(trans,geo_x,geo_y,2);
            EB_LOG("[EB_DEBUG::] geox: %lf geo_y: %lf dx: %f dy: %f\n",
                                geo_x,geo_y,dx,dy);
            contour.push_back(cv::Point(dx,dy));
        }
        contours.push_back(contour);
        fclose(fp);

    }

    std::vector<std::vector<cv::Vec2f>> polygons_vec;
    std::vector<std::vector<cv::Point>> isprs_contours;
    shp_reader(shp_file,polygons_vec);
    for(int i = 0; i < polygons_vec.size(); i++)
    {
        EB_LOG("[EB_DEBUG::] shapefile poly %d:\n",i);
        std::vector<cv::Point> isprs_contour;
        for(int j = 0; j < polygons_vec[i].size(); j++)
        {
            float dx = get_row_column_frm_geoX_geoY(trans,polygons_vec[i][j][0],polygons_vec[i][j][1],1);
            float dy = get_row_column_frm_geoX_geoY(trans,polygons_vec[i][j][0],polygons_vec[i][j][1],2);
            isprs_contour.push_back(cv::Point(dx,dy));
            EB_LOG("[EB_DEBUG::] shp_geox: %lf shp_geo_y: %lf dx: %f dy: %f\n",
                                polygons_vec[i][j][0],polygons_vec[i][j][1],dx,dy);
        }
        isprs_contours.push_back(isprs_contour);
    }

    #ifdef IOU_BRY
    cv::polylines(eb_image, contours, true, cv::Scalar(255), 1, cv::LINE_AA);
    cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_RECT, 
                                            cv::Size(dil_size,dil_size));
    cv::dilate(eb_image,eb_image,dilateElement);
    #else
    cv::polylines(eb_image, contours, true, cv::Scalar(237, 231, 21), 1, cv::LINE_AA);//第2个参数可以采用contour或者contours，均可
	cv::fillPoly(eb_image, contours, cv::Scalar(237, 231, 21));//fillPoly函数的第二个参数是二维数组！！
    #endif
    cv::imwrite(eb_path,eb_image);

    #ifdef IOU_BRY
    cv::polylines(isprs_image, isprs_contours, true, cv::Scalar(255), 1, cv::LINE_AA);
    cv::dilate(isprs_image,isprs_image,dilateElement);
    
    #else
    cv::polylines(isprs_image, isprs_contours, true, cv::Scalar(254, 0, 0), 1, cv::LINE_AA);//第2个参数可以采用contour或者contours，均可
	cv::fillPoly(isprs_image, isprs_contours, cv::Scalar(254, 0, 0));//fillPoly函数的第二个参数是二维数组！！
    #endif
    cv::imwrite(isprs_path,isprs_image);
    

    #if 1
    typedef struct iou_pixel
    {
        int x;
        int y;
        bool T_F;
        bool P_N;
        int contour_index; 
    }iou_pixel_t;

    typedef struct iou
    {
        int TP;
        int FP;
        int TN;
        int FN;
    }iou_t;
    

    #if 0
    iou_t * iou_p = (iou_t *)malloc(sizeof(iou_t)*isprs_contours.size());
    memset(iou_p,0,sizeof(iou_t)*isprs_contours.size());
    #endif
    iou_t * all_iou_p = (iou_t *)malloc(sizeof(iou_t));
    memset(all_iou_p,0,sizeof(iou_t));
    #endif
    
    cv::Mat iou_res = cv::Mat(isprs_image.rows,isprs_image.cols,CV_8UC3,cv::Scalar(255,255,255));
    char iou_path[256];
    snprintf(iou_path,strlen(path)+1,path);
    snprintf(iou_path+strlen(iou_path),28+strlen(type),"Area3_all/Area3_%siou_bry.tif",type);

    for(int col = 0; col < eb_image.cols; col++)
    {
        for(int row = 0; row < eb_image.rows; row++)
        {
            if(
                #ifdef IOU_BRY
                eb_image.at<uchar>(row,col) == 0 &&
                isprs_image.at<uchar>(row,col) == 0
                #else
                eb_image.at<cv::Vec3b>(row,col)[2] ==255 &&
                isprs_image.at<cv::Vec3b>(row,col)[2] ==255
                #endif
                )
                {
                    #if 0
                    for(int k = 0; k<isprs_contours.size();k++)
                    {
                        if(poi_inside_poly(isprs_contours[k],col,row))
                        {
                            iou_p[k].TN++;
                            break;
                        }
                    }
                    #endif

                    all_iou_p->TN++;
                    continue;
                }

            if(
                #ifdef IOU_BRY
                eb_image.at<uchar>(row,col) != 0 &&
                isprs_image.at<uchar>(row,col) != 0
                #else
                eb_image.at<cv::Vec3b>(row,col)[2] != 255 && 
                isprs_image.at<cv::Vec3b>(row,col)[2] != 255
                #endif
                )
                {
                    iou_res.at<cv::Vec3b>(row,col)[0] = 0;
                    iou_res.at<cv::Vec3b>(row,col)[1] = 255;
                    iou_res.at<cv::Vec3b>(row,col)[2] = 255;

                    #if 0
                    for(int k = 0; k<isprs_contours.size();k++)
                    {
                        if(poi_inside_poly(isprs_contours[k],col,row))
                        {
                            iou_p[k].TP++;
                            break;
                        }
                    }
                    #endif
                    all_iou_p->TP++;
                    continue;

                }

            if(
                #ifdef IOU_BRY
                eb_image.at<uchar>(row,col) == 0 &&
                isprs_image.at<uchar>(row,col) != 0
                #else
                eb_image.at<cv::Vec3b>(row,col)[2] == 255 && 
                isprs_image.at<cv::Vec3b>(row,col)[2] != 255
                #endif
                )
                {
                    iou_res.at<cv::Vec3b>(row,col)[0] = 255;
                    iou_res.at<cv::Vec3b>(row,col)[1] = 0;
                    iou_res.at<cv::Vec3b>(row,col)[2] = 0;

                    #if 0
                    for(int k = 0; k<isprs_contours.size();k++)
                    {
                        if(poi_inside_poly(isprs_contours[k],col,row))
                        {
                            iou_p[k].FN++;
                            break;
                        }
                    }
                    #endif

                    all_iou_p->FN++;
                    continue;

                }

            if(
                #ifdef IOU_BRY
                eb_image.at<uchar>(row,col) != 0 &&
                isprs_image.at<uchar>(row,col) == 0
                #else
                eb_image.at<cv::Vec3b>(row,col)[2] != 255 && 
                isprs_image.at<cv::Vec3b>(row,col)[2] == 255
                #endif
                )
                {
                    iou_res.at<cv::Vec3b>(row,col)[0] = 0;
                    iou_res.at<cv::Vec3b>(row,col)[1] = 0;
                    iou_res.at<cv::Vec3b>(row,col)[2] = 255;

                    #if 0
                    for(int k = 0; k<isprs_contours.size();k++)
                    {
                        if(poi_inside_poly(isprs_contours[k],col,row))
                        {
                            iou_p[k].FP++;
                            break;
                        }
                    }
                    #endif
                    all_iou_p->FP++;
                    continue;

                }
        }
    }

    char iou_txt_path[256];
    snprintf(iou_txt_path,strlen(path)+1,path);
    snprintf(iou_txt_path+strlen(iou_txt_path),30+strlen(type),"Area3_all/%d_Area3_%siou_bry.txt",dil_size,type);
    FILE *iou_txt_fp = fopen(iou_txt_path,"w");
    double all_C_R = (double)all_iou_p->TP/(double)(all_iou_p->TP + all_iou_p->FP);
    double all_C_P = (double)all_iou_p->TP/(double)(all_iou_p->TP + all_iou_p->FN);
    double all_Q = (double)all_iou_p->TP / (double)(all_iou_p->TP + all_iou_p->FP + all_iou_p->FN);
    fprintf(iou_txt_fp,"all_CP all_CR all_Q\n");
    fprintf(iou_txt_fp,"%lf %lf %lf\n\n",all_C_P,all_C_R,all_Q);
    #if 0
    fprintf(iou_txt_fp,"build_idx CP CR Q\n");
    double cr_sum = 0.;
    double cp_sum = 0.;
    double q_sum = 0.;
    for(int i =0; i < isprs_contours.size(); i++)
    {
        double cr = (double)iou_p[i].TP/(double)(iou_p[i].TP + iou_p[i].FP);
        cr_sum += cr;
        double cp = (double)iou_p[i].TP/(double)(iou_p[i].TP + iou_p[i].FN);
        cp_sum += cp;
        double q = (double)iou_p[i].TP/(double)(iou_p[i].TP + iou_p[i].FP + iou_p[i].FN);
        q_sum += q;
        fprintf(iou_txt_fp,"%d %lf %lf %lf\n",i,cp,cr,q);
    }
    fprintf(iou_txt_fp,"\naverage CP CR Q\n");
    cr_sum = cr_sum/isprs_contours.size();
    cp_sum = cp_sum/isprs_contours.size();
    q_sum = q_sum/isprs_contours.size();
    fprintf(iou_txt_fp,"%lf %lf %lf\n",cp_sum,cr_sum,q_sum);
    #endif
    fclose(iou_txt_fp);
    


    cv::imwrite(iou_path,iou_res);
    return 0;
    #endif

    /******************************************** read tiif and get the boundary points in dom with CV methods*************************************************/
    eb_features_t eb_extract_features;
    init_features(&eb_extract_features);
    static eb_config_t eb_config; 
    read_eb_config(&eb_config,argv[1]);

    clock_t eb_time = clock();
    EB_LOG("[EB::INFO] %s EB process Begin! time is %ld\n",eb_config.file_config.name,eb_time);
    
    init_mats(&eb_extract_features.eb_mats,
                eb_config.file_config.image_path,
                eb_config.file_config.origin_path);
    if(!eb_extract_features.eb_mats.input_image.empty())
    {
        EB_LOG("[CV::INFO] Image %s open success!\n",eb_config.file_config.image_path);
    }
    else
    {
        EB_LOG("[CV::ERROR] Image %s open failed!\n",eb_config.file_config.image_path);
        return 0;
    }

    #if 0
    get_NDVI_res(eb_extract_features.eb_mats.ndvi_image,eb_extract_features.eb_mats.input_image);
    mat_show("ndvi_image",eb_extract_features.eb_mats.ndvi_image,MAT_SIZE,&eb_config);
    return 0;
    #endif

    #if 1
    char jpg_path[256];
    snprintf(jpg_path,strlen(eb_config.file_config.out_path)+1,eb_config.file_config.out_path);
    snprintf(jpg_path+strlen(jpg_path),strlen(eb_config.file_config.name)+6,"/%s.bmp",eb_config.file_config.name);
    EB_LOG("[EB_DEBUG::] bmp path is %s\n",jpg_path);
    cv::imwrite(jpg_path,eb_extract_features.eb_mats.origin_image);
    #endif 
    //canny find boundary and record in img_out
    #ifdef TEST_8_SOBEL
    sobel_find_boundary(eb_extract_features.eb_mats.input_image,
                        eb_extract_features.eb_mats.canny_image,
                        &eb_config);
    #endif

    #ifdef TEST_CANNY
    canny_find_boundary(eb_extract_features.eb_mats.input_image,
                        eb_extract_features.eb_mats.canny_image,
                        &eb_config);
    #endif

    #ifdef TEST_LoG
    LoG_find_boundary(eb_extract_features.eb_mats.input_image,
                        eb_extract_features.eb_mats.canny_image,
                        &eb_config);
    #endif
    //hough find the lines in img_out and record in lines
    hough_find_lins(&eb_extract_features.hough_lines, 
                    eb_extract_features.eb_mats.canny_image, 
                    eb_extract_features.eb_mats.hough_image,
                    &eb_config);


    if(eb_config.TYPE == EB_LIDAR)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        if ( pcl::io::loadPCDFile <pcl::PointXYZ> (eb_config.file_config.lidar_path, *cloud) == -1)
        {
            EB_LOG("[PCL::ERROR] Point cloud %s reading failed.\n",eb_config.file_config.lidar_path);
            return (-1);
        }


        #if 0
        filter_pcl_with_z(cloud,&eb_config);
        std::vector<PointCloud<PointXYZ>::Ptr> tmp_planars_cloud_vec = {cloud} ;
        pcl::visualization::PCLVisualizer viewer("Planar Viewer");
        D3_view(viewer,tmp_planars_cloud_vec,cloud);
        pcl::io::savePCDFile("no_ground.pcd",*(tmp_planars_cloud_vec[0]));
        while (!viewer.wasStopped ())
        {
            viewer.spinOnce();
        }
        #endif

        getTrans_of_TiffFile(eb_config.file_config.image_path,eb_config.trans);
        #ifdef MID_RESULT
        getTrans_of_TiffFile("/home/jaxzhong/Exp_data/Area3/result/Area3_all/Area3.tif",
                                eb_config.all_trans);
        #endif
        //save_Trans_to_File(&eb_config);
        #ifdef TEST_NICE_GROUND
        std::vector<PointCloud<PointXYZ>::Ptr> org_planars_cloud_vec = {cloud} ;
        pcd_to_mat(org_planars_cloud_vec[0],
                    eb_config.trans,&eb_config,
                    &eb_extract_features.org_palnar_pois);
        EB_LOG("[EB_DEBUG::] org_planar_num is %ld\n",eb_extract_features.org_palnar_pois.point_size);

        #endif

        filter_pcl_with_z(cloud,&eb_config);

        double cloud_min_max_z[2];
        get_min_max_z(cloud_min_max_z,cloud,&eb_config);

        std::vector<PointCloud<PointXYZ>::Ptr> planars_cloud_vec = {cloud} ;
        
        //visualization the planar pcd
    #ifdef EB_PCL_VISUAL    
        pcl::visualization::PCLVisualizer viewer("Planar Viewer");
        D3_view(viewer,planars_cloud_vec,cloud);
    #endif
        std::vector<PointCloud<PointXYZ>::Ptr> boundaries_cloud_vec;
        // get the boundary points loud and record them in boundaries_cloud_vec
        estimateBorders(planars_cloud_vec,boundaries_cloud_vec,&eb_config); 
        //visualization the planar pcd
    #ifdef EB_PCL_VISUAL
        pcl::visualization::PCLVisualizer viewer1("boundary Viewer");
        D3_view(viewer1,boundaries_cloud_vec,cloud);
    #endif
        //pcl::io::savePCDFile(_output+"boundary_"+_name+".pcd",*(boundaries_cloud_vec[0]));

        std::vector<PointCloud<PointXYZ>::Ptr> roof_cloud_vec;
        region_grow(cloud,roof_cloud_vec);
    #ifdef EB_PCL_VISUAL_1
        pcl::visualization::PCLVisualizer viewer2("roofs Viewer");
        D3_view(viewer2,roof_cloud_vec,cloud);
    #endif

    #ifdef EB_PCL_VISUAL
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce();
        viewer1.spinOnce();
    // viewer2.spinOnce();
    }
    #endif   

        //double trans[6];
        // get the location transform prameters in input dom 
        pcd_to_mat(boundaries_cloud_vec[0],
                    eb_config.trans,&eb_config,
                    &eb_extract_features.boundary_points);

        pcd_to_mat(planars_cloud_vec[0],
                    eb_config.trans,&eb_config,
                    &eb_extract_features.palnar_pois);


        //build the 2d regular net
        DelaunayT delaunayT(&eb_extract_features.boundary_points);
        delaunayT.generateAlphaShape();
        EB_LOG("[DELAUNAY::INFO] generateAlphaShape completed!\n");
        delaunayT.getBoundary_pois(&eb_extract_features.delau_boundary_pois,
                                    &eb_extract_features.boundary_points);
        
        #ifdef TEST_LIDAR_ORIGIN_AND_DP
        cv::Mat origin_pois_mat = eb_extract_features.eb_mats.origin_image.clone();
        cv::Mat dp_origin_pois_mat = eb_extract_features.eb_mats.origin_image.clone();
        EB_LOG("[EB_DEBUG::]-----------------> DP 1\n");
        char build_path[256];
        snprintf(build_path,strlen(eb_config.file_config.out_path)+1,eb_config.file_config.out_path);
        snprintf(build_path+strlen(build_path),23,"/origin_final_pois.txt");
        FILE *origin_pois_fp = fopen(build_path,"w");
        EB_LOG("[EB::DEBUG::] build_path is %s\n",build_path);

        EB_LOG("[EB_DEBUG::]-----------------> DP 2\n");
        char dp_build_path[256];
        snprintf(dp_build_path,strlen(eb_config.file_config.out_path)+1,eb_config.file_config.out_path);
        snprintf(dp_build_path+strlen(dp_build_path),20,"/dp_final_pois.txt");
        FILE *dp_pois_fp = fopen(dp_build_path,"w");
        EB_LOG("[EB::DEBUG::] dp_build_path is %s\n",dp_build_path);
        EB_LOG("[EB_DEBUG::]-----------------> DP 3\n");
        std::vector<cv::Point> contour;
        for(int i = 0; i < eb_extract_features.delau_boundary_pois.point_size; i++)
        {
            EB_LOG("[EB_DEBUG::]-----------------> DP 3-->i = %d\n",i);
            int next = i == eb_extract_features.delau_boundary_pois.point_size-1?0:i+1;
            fprintf(origin_pois_fp,"%lf %lf %lf\n",
                    eb_extract_features.delau_boundary_pois.points[i].point_x,
                    eb_extract_features.delau_boundary_pois.points[i].point_y,
                    eb_extract_features.delau_boundary_pois.points[i].point_z);
            cv::line(origin_pois_mat,cv::Point(eb_extract_features.delau_boundary_pois.points[i].dx,
                                                eb_extract_features.delau_boundary_pois.points[i].dy),
                                    cv::Point(eb_extract_features.delau_boundary_pois.points[next].dx,
                                                eb_extract_features.delau_boundary_pois.points[next].dy),
                                    cv::Scalar(0,255,0),
                                    3,
                                    CV_AA);

            contour.push_back(cv::Point(eb_extract_features.delau_boundary_pois.points[i].dx,
                                                eb_extract_features.delau_boundary_pois.points[i].dy));
            
        }
        EB_LOG("[EB_DEBUG::]-----------------> DP 4\n");
        //double dp_trd = cv::arcLength(contour,1)/atof(argv[2]);
        double dp_trd = atof(argv[2]);
        std::vector<cv::Point> dp_contour;
        cv::approxPolyDP(contour,dp_contour,dp_trd,1);
        EB_LOG("[EB_DEBUG::]-----------------> DP 5\n");
        for(int i = 0; i < dp_contour.size(); i++)
        {
            int next = i == dp_contour.size()-1?0:i+1;
            double geo_x = -1.;
            double geo_y = -1.;
            get_geoX_geoY_frm_row_column(eb_config.trans,dp_contour[i].x,dp_contour[i].y,&geo_x,&geo_y);
            fprintf(dp_pois_fp,"%lf %lf 0.0\n",geo_x,geo_y);
            cv::line(dp_origin_pois_mat,cv::Point(dp_contour[i].x,dp_contour[i].y),
                                        cv::Point(dp_contour[next].x,dp_contour[next].y),
                                        cv::Scalar(0,255,0),
                                        3,
                                        CV_AA);
        }

        mat_show("origin_lines",origin_pois_mat,MAT_SIZE,&eb_config);
        mat_show("dp_origin_lines",dp_origin_pois_mat,MAT_SIZE,&eb_config);
        fclose(origin_pois_fp);
        fclose(dp_pois_fp);
        return 0;
        #endif
    }
    



    if(eb_config.TYPE == EB_CNN)
    {
        cv::Mat mask_image;
        mask_image = cv::imread(eb_config.file_config.lidar_path,0);
        find_contours(mask_image,&eb_extract_features,&eb_config);
    }
    

    
    set_buffer_mat(eb_extract_features.eb_mats.boundary_image,
                    eb_extract_features.eb_mats.buffer_image,
                    &eb_extract_features.delau_boundary_pois,
                    &eb_config
                    #ifdef MID_RESULT
                    ,
                    eb_extract_features.eb_mats.all_buffer_image
                    #endif
                    );

    
    #ifdef TEST_0
    cv::Mat ass_temp = eb_extract_features.eb_mats.origin_image.clone();
    for(int i =0; i < eb_extract_features.eb_mats.buffer_image.cols; i++)
    {
        for(int j =0; j < eb_extract_features.eb_mats.buffer_image.rows; j++)
        {
            if(eb_extract_features.eb_mats.buffer_image.at<uchar>(j,i) != 255)
            {
                ass_temp.at<cv::Vec3b>(j,i)[0] = 255;
                ass_temp.at<cv::Vec3b>(j,i)[1] = 255;
                ass_temp.at<cv::Vec3b>(j,i)[2] = 255;
            }
        }
    }
    mat_show("temp_ass",ass_temp,MAT_SIZE,&eb_config);
    #endif


    buffer_filter(&eb_extract_features,&eb_config);
    
    adsorbent_filter(&eb_extract_features,&eb_config);

    eb_update_boundary_pois(&eb_extract_features,&eb_config);

    eb_roof_t roof_ptr;

    generate_basic_roof(&eb_extract_features,&eb_config,&roof_ptr);

    #ifdef MID_RESULT
    cv::imwrite("/home/jaxzhong/Exp_data/Area3/result/Area3_all/all_buffer.bmp",
                        eb_extract_features.eb_mats.all_buffer_image);
    cv::imwrite("/home/jaxzhong/Exp_data/Area3/result/Area3_all/all_buffer_filter.bmp",
                        eb_extract_features.eb_mats.all_buf_filter_image);

    cv::imwrite("/home/jaxzhong/Exp_data/Area3/result/Area3_all/all_simplfy.bmp",
                        eb_extract_features.eb_mats.all_simply_image);

    cv::imwrite("/home/jaxzhong/Exp_data/Area3/result/Area3_all/all_refine.bmp",
                        eb_extract_features.eb_mats.all_refine_image);
    #else
    if(eb_config.TYPE == EB_LIDAR)
    {
        generate_roofs(&eb_extract_features, &roof_ptr, &eb_config);
        //To record the time in EB process.
        eb_time = clock();
        EB_LOG("[EB::INFO] Extract the Building Contours finished! time is %ld\n",eb_time);
        roof_ptr.basic_poly_info.eb_time = (double)eb_time/CLOCKS_PER_SEC;
        #if 0
        vector_roof_rebuild(&roof_ptr,&eb_config,eb_extract_features.eb_mats.roofs_lidar_image);
        #endif
        grid_roof_rebuild(&roof_ptr,&eb_config, 
                eb_extract_features.eb_mats.roofs_lidar_image,
                eb_extract_features.eb_mats.org_roofs_lidar_image,
                eb_extract_features.eb_mats.close_lines_image);
    }
    #endif
    

    return 0;

}
