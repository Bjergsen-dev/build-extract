#include "checkCornerPoints.hpp"
#include "delaunayT.hpp"
#include "fetch_building_three_d_points.hpp"
#include "shapefile_crt.hpp"
#include "eb_config.hpp"
#include "eb_common_defines.hpp"
#include "eb_images_detect.hpp"
#include "eb_lidar_detect.hpp"
#include "eb_transform.hpp"
#include "eb_filter.hpp"
#include "eb_rebuild.hpp"

#if 0 
float fangcha(vector<float> foot_dis_vec){
    
    float average_foot_dis = 0.0;
    float dis_num = 0.0;
    for(float dis : foot_dis_vec){
        dis_num += dis;
    }
    
    average_foot_dis = dis_num/(foot_dis_vec.size()-1);
    
    float dis_pow_num = 0.0;
    
    for(float dis : foot_dis_vec){
        dis_pow_num += pow((dis-average_foot_dis),2);
    }
    
    return sqrt(dis_pow_num/(foot_dis_vec.size()-1));
    
}


int describe_the_similarity_of_two_line_with_point_num__vec(vector<int> vec1,vector<int> vec2){
    int same_count = 0;
    for(int ii : vec1){
        for(int jj : vec2){
            if(ii == jj){
                same_count++;
                
            }
        }
    }
    
    float bigger_one = (float)same_count/(float)vec1.size() > (float)same_count/(float)vec2.size()?(float)same_count/(float)vec1.size():(float)same_count/(float)vec2.size();
    
    float res = bigger_one > 0.5? bigger_one:0;
    
    int res_int = res==0? 0:same_count;
    
    return res_int;
}


#endif


int main() {
    /******************************************** read tiif and get the boundary points in dom with CV methods*************************************************/
    eb_features_t eb_extract_features;
    init_features(&eb_extract_features);
    static eb_config_t eb_config; 
    read_eb_config(&eb_config,"/home/jaxzhong/Projects/build_extract/config.txt");
    
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
    #if 1
    char jpg_path[256];
    snprintf(jpg_path,strlen(eb_config.file_config.out_path)+1,eb_config.file_config.out_path);
    snprintf(jpg_path+strlen(jpg_path),strlen(eb_config.file_config.name)+6,"/%s.bmp",eb_config.file_config.name);
    EB_LOG("[EB_DEBUG::] bmp path is %s\n",jpg_path);
    cv::imwrite(jpg_path,eb_extract_features.eb_mats.origin_image);
    #endif 
    //canny find boundary and record in img_out
    canny_find_boundary(eb_extract_features.eb_mats.input_image,
                        eb_extract_features.eb_mats.canny_image,
                        &eb_config);
    //hough find the lines in img_out and record in lines
    hough_find_lins(&eb_extract_features.hough_lines, 
                    eb_extract_features.eb_mats.canny_image, 
                    eb_extract_features.eb_mats.hough_image,
                    &eb_config);


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if ( pcl::io::loadPCDFile <pcl::PointXYZ> (eb_config.file_config.lidar_path, *cloud) == -1)
    {
        EB_LOG("[PCL::ERROR] Point cloud %s reading failed.\n",eb_config.file_config.lidar_path);
        return (-1);
    }

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
    getTrans_of_TiffFile(eb_config.file_config.image_path,eb_config.trans); 
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
    
    set_buffer_mat(eb_extract_features.eb_mats.boundary_image,
                    eb_extract_features.eb_mats.buffer_image,
                    &eb_extract_features.delau_boundary_pois,
                    &eb_config);


    buffer_filter(&eb_extract_features,&eb_config);
    
    adsorbent_filter(&eb_extract_features,&eb_config);

    eb_update_boundary_pois(&eb_extract_features,&eb_config);

    eb_roof_t roof_ptr;

    generate_basic_roof(&eb_extract_features,&eb_config,&roof_ptr);

    #if 0
    lidar_planar_to_image(cloud,eb_extract_features.eb_mats.roofs_lidar_image,
                            eb_config.trans,eb_config.transform_x,eb_config.transform_y);
    mat_show("lidar_roof_image",eb_extract_features.eb_mats.roofs_lidar_image,MAT_SIZE);
    #endif
    generate_roofs(&eb_extract_features, &roof_ptr, &eb_config);

    vector_roof_rebuild(&roof_ptr,&eb_config,eb_extract_features.eb_mats.roofs_lidar_image);
    
    grid_roof_rebuild(&roof_ptr,&eb_config, eb_extract_features.eb_mats.roofs_lidar_image);
   

    return 0;

}
