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
    
//     if(argc < 2){
        
//         std::cout<<"Too few pramaters!"<<std::endl;
//         return 0;
//     }
    
//     std::string json_file_path = argv[1];
//     std::vector<std::string> json_str_vec;
//     readFileJson(json_file_path,json_str_vec);
    
//     std::string image_path_str = json_str_vec[0];
//     std::string cloud_path_str = json_str_vec[1];
    
//     std::string ttstr1 = "/within_lines.tif";
//     std::string ttstr2 = "/filter_lines.tif";
//     std::string ttstr3 = "/sorted_best_line_in_similar_filter_lines.tif";
//     std::string ttstr4 = "/complete_mat.tif";
//     std::string output_path_1 = json_str_vec[4] + ttstr1;
//     std::string output_path_2 = json_str_vec[4] + ttstr2;
//     std::string output_path_3 = json_str_vec[4] + ttstr3;
//     std::string output_path_4 = json_str_vec[4] + ttstr4;
    
//     std::string oesm_path_str= json_str_vec[2];
    
//     std::string dem_path_str= json_str_vec[3];
    
//     std::string obj_str = "/buildings_model.obj";
//     std::string obj_file_path = json_str_vec[4] + obj_str;
    
    init_mats(&eb_extract_features.eb_mats,eb_config.file_config.image_path);
    if(!eb_extract_features.eb_mats.input_image.empty())
    {
        EB_LOG("[CV::INFO] Image %s open success!\n",eb_config.file_config.image_path);
    }
    else
    {
        EB_LOG("[CV::ERROR] Image %s open failed!\n",eb_config.file_config.image_path);
        return 0;
    }
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

#ifdef EB_PCL_VISUAL
   while (!viewer.wasStopped ())
  {
    viewer.spinOnce();
    viewer1.spinOnce();
  }
#endif      
    double trans[6];
    // get the location transform prameters in input dom
    getTrans_of_TiffFile(eb_config.file_config.image_path,trans); 
    pcd_to_mat(boundaries_cloud_vec[0],
                trans,&eb_config,
                &eb_extract_features.boundary_points);

    pcd_to_mat(planars_cloud_vec[0],
                trans,&eb_config,
                &eb_extract_features.palnar_pois);
    
    
    
    
//     Mat within_lines_mat = Mat(img.size().height,img.size().width,CV_8UC3,Scalar(255, 255, 255));
//     Mat test_mat_1 = Mat(img.size().height,img.size().width,CV_8UC3,Scalar(255, 255, 255));
//     Mat test_mat_2 = Mat::zeros(img.size().height,img.size().width,CV_8UC3);//Mat(img.size().height,img.size().width,CV_8UC3,Scalar(255, 255, 255));
//     Mat points_to_lines_test_mat = Mat(img.size().height,img.size().width,CV_8UC3,Scalar(255,255,255));
//     Mat similar_filter_combine_similar_lines_mat = Mat(img.size().height,img.size().width,CV_8UC3,Scalar(255, 255, 255));
//     Mat longest_similar_filter_combine_similar_lines_mat = Mat(img.size().height,img.size().width,CV_8UC3,Scalar(255, 255, 255));//Mat(img.size().height,img.size().width,CV_8UC3,Scalar(255, 255, 255));
//     Mat sorted_longest_similar_filter_combine_similar_lines_mat = Mat::zeros(img.size().height,img.size().width,CV_8UC3);//Mat(img.size().height,img.size().width,CV_8UC3,Scalar(255, 255, 255));
//     Mat lost_found_and_sorted_longest_similar_filter_combine_similar_lines_mat = Mat(img.size().height,img.size().width,CV_8UC3,Scalar(255, 255, 255));//Mat(img.size().height,img.size().width,CV_8UC3,Scalar(255, 255, 255));
//    // Mat complete_mat = Mat(img.size().height,img.size().width,CV_8UC3,Scalar(255, 255, 255));//img.clone();//imread(image_path_str,0);
//     Mat complete_mat = img.clone();
//     Mat alpha_shape_points_mat = img.clone();//Mat(img.size().height,img.size().width,CV_8UC1,Scalar(0));

    
//     std::vector<std::vector<z_Point>> z_points_v_vec;

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
                            trans,eb_config.transform_x,eb_config.transform_y);
    mat_show("lidar_roof_image",eb_extract_features.eb_mats.roofs_lidar_image,MAT_SIZE);
    #endif
    generate_roofs(&eb_extract_features, &roof_ptr, &eb_config);
    
    
    
    
    
    
    
    
    


    
    
    
    
    
//   std::cout<<"------------------choose the similar lines in filter_combine_lines-----------------------"<<std::endl<<std::endl;  
// /********************************** choose the similar lines in filter_combine_lines *******************************************************/
//     vector<vector<Vec4f>> similar_filter_combine_lines;//output record vec
//     vector<vector<int>> similar_filter_combine_lines_mid_num;
//     vector<vector<vector<int>>> similar_filter_combine_lines_points_num;
//     vector<vector<vector<Point>>> similar_filter_combine_lines_points_foot;
//     vector<vector<vector<float>>> similar_filter_combine_lines_points_foot_dis;
    
//     vector<vector<int>> v_filter_combine_lines_mid_point_indexnum;//output record simialr vec index
    
//     //first sort the lines with length 
//     vector<float> filter_lines_length;
//     for(Vec4f vv : filter_combine_lines){
//         filter_lines_length.push_back(distance_btw_two_points(Point(vv[0],vv[1]),Point(vv[2],vv[3])));
//     }
//     quick_sort(filter_lines_length,filter_combine_lines,0,filter_lines_length.size());
//     quick_sort(filter_lines_length,filter_combine_lines_mid_point_indexnum,0,filter_lines_length.size());
//     quick_sort(filter_lines_length,filter_line_cross_points,0,filter_lines_length.size());
//     quick_sort(filter_lines_length,filter_line_cross_points_foots,0,filter_lines_length.size());
//     quick_sort(filter_lines_length,filter_line_cross_points_foots_dis,0,filter_lines_length.size());
    
    
    
//     //find the similars
//     vector<int> temp_use_indexnum = filter_combine_lines_mid_point_indexnum;
//     for(int i = temp_use_indexnum.size()-1;i>=0;i--){
        
//         if(i == temp_use_indexnum.size()-1){
//              v_filter_combine_lines_mid_point_indexnum.push_back(vector<int>{i});
//              similar_filter_combine_lines.push_back(vector<Vec4f>{filter_combine_lines[i]});
//              similar_filter_combine_lines_mid_num.push_back(vector<int>{filter_combine_lines_mid_point_indexnum[i]});
//              similar_filter_combine_lines_points_num.push_back(vector<vector<int>>{filter_line_cross_points[i]});
//              similar_filter_combine_lines_points_foot.push_back(vector<vector<Point>>{filter_line_cross_points_foots[i]});
//              similar_filter_combine_lines_points_foot_dis.push_back(vector<vector<float>>{filter_line_cross_points_foots_dis[i]});
//              continue;
//         }
//         int found_not = 0;
//         int similar_value = 0;
//         int max_similar_value_index = 0;
//         for(int j =0;j<v_filter_combine_lines_mid_point_indexnum.size();j++){
//             vector<int> similar_mid_num_vec = v_filter_combine_lines_mid_point_indexnum[j];
            
//             int longest_index = get_longest_line_index_in_vec(similar_filter_combine_lines[j]);
//             //common point nums
//             /*for(int tem_i : similar_mid_num_vec){
//                 max_similar_value_index = describe_the_similarity_of_two_line_with_point_num__vec(filter_line_cross_points[tem_i],filter_line_cross_points[i])>similar_value?j:max_similar_value_index;
                
//                 similar_value = describe_the_similarity_of_two_line_with_point_num__vec(filter_line_cross_points[tem_i],filter_line_cross_points[i])>similar_value? describe_the_similarity_of_two_line_with_point_num__vec(filter_line_cross_points[tem_i],filter_line_cross_points[i]) : similar_value;
//            }*/
            
//                 max_similar_value_index = describe_the_similarity_of_two_line_with_point_num__vec(similar_filter_combine_lines_points_num[j][longest_index],filter_line_cross_points[i])>similar_value?j:max_similar_value_index;
                
//                 similar_value = describe_the_similarity_of_two_line_with_point_num__vec(similar_filter_combine_lines_points_num[j][longest_index],filter_line_cross_points[i])>similar_value? describe_the_similarity_of_two_line_with_point_num__vec(similar_filter_combine_lines_points_num[j][longest_index],filter_line_cross_points[i]) : similar_value;
            
//             //near mid num
//             if(sqrt(pow(similar_filter_combine_lines_mid_num[j][longest_index]- temp_use_indexnum[i],2)) <= min_cloud_points_num){
//                 v_filter_combine_lines_mid_point_indexnum[j].push_back(i);
//                 similar_filter_combine_lines[j].push_back(filter_combine_lines[i]);
//                 similar_filter_combine_lines_mid_num[j].push_back(filter_combine_lines_mid_point_indexnum[i]);
//                 similar_filter_combine_lines_points_num[j].push_back(filter_line_cross_points[i]);
//                 similar_filter_combine_lines_points_foot[j].push_back(filter_line_cross_points_foots[i]);
//                 similar_filter_combine_lines_points_foot_dis[j].push_back(filter_line_cross_points_foots_dis[i]);
//                 found_not = 1;
//                 break;
//             }
//         }
        
//         if(similar_value>0){
//                 v_filter_combine_lines_mid_point_indexnum[max_similar_value_index].push_back(i);
//                 similar_filter_combine_lines[max_similar_value_index].push_back(filter_combine_lines[i]);
//                 similar_filter_combine_lines_mid_num[max_similar_value_index].push_back(filter_combine_lines_mid_point_indexnum[i]);
//                 similar_filter_combine_lines_points_num[max_similar_value_index].push_back(filter_line_cross_points[i]);
//                 similar_filter_combine_lines_points_foot[max_similar_value_index].push_back(filter_line_cross_points_foots[i]);
//                 similar_filter_combine_lines_points_foot_dis[max_similar_value_index].push_back(filter_line_cross_points_foots_dis[i]);
//                 found_not = 1;
//         }
        
//         if(found_not==1){
//             continue;
//         }
        
//         v_filter_combine_lines_mid_point_indexnum.push_back(vector<int>{i});
//         similar_filter_combine_lines.push_back(vector<Vec4f>{filter_combine_lines[i]});
//         similar_filter_combine_lines_mid_num.push_back(vector<int>{filter_combine_lines_mid_point_indexnum[i]});
//         similar_filter_combine_lines_points_num.push_back(vector<vector<int>>{filter_line_cross_points[i]});
//         similar_filter_combine_lines_points_foot.push_back(vector<vector<Point>>{filter_line_cross_points_foots[i]});
//         similar_filter_combine_lines_points_foot_dis.push_back(vector<vector<float>>{filter_line_cross_points_foots_dis[i]});
//     }
    

//     //draw the lines in filter_combine_similar_lines
//     for(vector<Vec4f> lines_vec : similar_filter_combine_lines) {
//         Scalar r_color = Scalar(rand()%255, rand()%255, rand()%255);//Scalar(0,0,0);
//         for(Vec4f test_vec : lines_vec) {
//             line(similar_filter_combine_similar_lines_mat,Point(test_vec[0],test_vec[1]),Point(test_vec[2],test_vec[3]),r_color,2,LINE_AA);
//         }
//     }
    
//      for(vector<Point> tt_foot_vec : filter_line_cross_points_foots){
//             for(Point foot : tt_foot_vec){
//                 circle(similar_filter_combine_similar_lines_mat,foot,5,Scalar(0,0,0),1);
//             }
//         }
    
//     for(int i =0; i < similar_filter_combine_lines_points_num.size(); i++) {
//         for(int j = 0; j<similar_filter_combine_lines_points_num[i].size();j++) {
//             for(int num : similar_filter_combine_lines_points_num[i][j]){
//                 cout<<num<<" ";
//             }
//             cout<<"------"<<fangcha(similar_filter_combine_lines_points_foot_dis[i][j]);
//             cout<<endl;
//         }
        
//         cout<<endl<<endl;
//     }
    
    
    
    
    
//     //free the vecs
//     release_vector(filter_combine_lines);
//     release_vector(filter_combine_lines_mid_point_indexnum);
//     release_vector(filter_line_cross_points);
//     release_vector(filter_line_cross_points_foots);
//     release_vector(filter_line_cross_points_foots_dis);
    
    
//     std::cout<<"------------------Select the longgest line in each combine similar lines vector and draw them-----------------------"<<std::endl<<std::endl;
// /*************************** select the longgest line in each combine similar lines vector and draw them *******************************************************/
//     vector<Vec4f> longest_similar_filter_combine_lines;
//     vector<int> longest_similar_filter_combine_lines_mid_num;
//     vector<vector<int>> longest_similar_filter_combine_lines_points_num;
//     vector<vector<Point>> longest_similar_filter_combine_lines_points_foot;
//     vector<vector<float>> longest_similar_filter_combine_lines_points_foot_dis;
//     for(int i = 0; i< similar_filter_combine_lines.size();i++) {
//         vector<Vec4f> lines_vec = similar_filter_combine_lines[i];
//         vector<vector<float>> lines_foots_dis = similar_filter_combine_lines_points_foot_dis[i];
//         float length_of_longest_line = 0.0;
//         float min_fangcha = fangcha(lines_foots_dis[0]);
//         float score = 0.0;
//         int index = 0;
//         for(int j =0;j< lines_vec.size(); j++) {
//             Vec4f line_vec = lines_vec[j];
//             float temp_length = similar_filter_combine_lines_points_num[i][j].size()/distance_btw_two_points(Point(line_vec[0],line_vec[1]),Point(line_vec[2],line_vec[3]));
//             length_of_longest_line = temp_length > length_of_longest_line? temp_length:length_of_longest_line;
//             min_fangcha = min_fangcha < fangcha(lines_foots_dis[j])? min_fangcha:fangcha(lines_foots_dis[j]);
            
//         }
        
//         for(int k = 0; k < lines_vec.size();k++){
//             float temp_length_score =(distance_btw_two_points(Point(lines_vec[k][0],lines_vec[k][1]),Point(lines_vec[k][2],lines_vec[k][3])))/length_of_longest_line;
//             float fangcha_score = min_fangcha/fangcha(lines_foots_dis[k]);
//             float lisan_score = similar_filter_combine_lines_points_num[i][k].size()/(similar_filter_combine_lines_points_num[i][k][similar_filter_combine_lines_points_num[i][k].size()-1]-similar_filter_combine_lines_points_num[i][k][0]);
//             index = score > (4*temp_length_score + 3*fangcha_score + 3*lisan_score)?index:k;
//              score = score > (4*temp_length_score + 3*fangcha_score + 3*lisan_score)? score : (4*temp_length_score + 3*fangcha_score + 3*lisan_score);
            
//         }
//         longest_similar_filter_combine_lines.push_back(similar_filter_combine_lines[i][index]);
//         longest_similar_filter_combine_lines_mid_num.push_back(similar_filter_combine_lines_mid_num[i][index]);
//         longest_similar_filter_combine_lines_points_num.push_back(similar_filter_combine_lines_points_num[i][index]);
//         longest_similar_filter_combine_lines_points_foot.push_back(similar_filter_combine_lines_points_foot[i][index]); 
//         longest_similar_filter_combine_lines_points_foot_dis.push_back(similar_filter_combine_lines_points_foot_dis[i][index]);
//     }
    
//   /*  //delete the similar left
//     vector<int> delete_index;

//         for(int i =0 ; i< longest_similar_filter_combine_lines_points_num.size();i++){
//             for(int j = i+1; j<longest_similar_filter_combine_lines_points_num.size();j++){
                
//                 if(describe_the_similarity_of_two_line_with_point_num__vec(longest_similar_filter_combine_lines_points_num[i],longest_similar_filter_combine_lines_points_num[j]) > 0){
//                     int index = longest_similar_filter_combine_lines_points_num[i].size() > longest_similar_filter_combine_lines_points_num[j].size()? j:i;
//                     delete_index.push_back(index);
//                 }
//             }
//         }
        
//         for(int del_index : delete_index ){
//             longest_similar_filter_combine_lines.erase(longest_similar_filter_combine_lines.begin() + del_index);
//             longest_similar_filter_combine_lines_mid_num.erase(longest_similar_filter_combine_lines_mid_num.begin() + del_index);
//             longest_similar_filter_combine_lines_points_num.erase(longest_similar_filter_combine_lines_points_num.begin() + del_index);
//             longest_similar_filter_combine_lines_points_foot.erase(longest_similar_filter_combine_lines_points_foot.begin() + del_index);
//             longest_similar_filter_combine_lines_points_foot_dis.erase(longest_similar_filter_combine_lines_points_foot_dis.begin() + del_index);
            
//             for( int i =0; i< delete_index.size();i++){
//                 delete_index[i] = delete_index[i] > del_index?delete_index[i]-1:delete_index[i];
//             }
//         }*/
        
//         //draw
//         for(Vec4f line1 : longest_similar_filter_combine_lines){
//             line(longest_similar_filter_combine_similar_lines_mat,Point(line1[0],line1[1]),Point(line1[2],line1[3]),Scalar(0,0,255),2,LINE_AA);
//         }
        
    
//     //free the vecs
//     release_vector(similar_filter_combine_lines);
//     release_vector(similar_filter_combine_lines_mid_num);
//     release_vector(similar_filter_combine_lines_points_foot);
//     release_vector(similar_filter_combine_lines_points_foot_dis);
//     release_vector(similar_filter_combine_lines_points_num);
    
    
//  std::cout<<"------------------Sort the lines and correct the wrong lines-----------------------"<<std::endl<<std::endl;   
// /***************************  sort the lines in last *******************************************************/


//     quick_sort(longest_similar_filter_combine_lines_mid_num,longest_similar_filter_combine_lines,0,longest_similar_filter_combine_lines_mid_num.size()-1);
//     quick_sort(longest_similar_filter_combine_lines_mid_num,longest_similar_filter_combine_lines_points_num,0,longest_similar_filter_combine_lines_mid_num.size()-1);
//     quick_sort(longest_similar_filter_combine_lines_mid_num,longest_similar_filter_combine_lines_points_foot,0,longest_similar_filter_combine_lines_mid_num.size()-1);
//     quick_sort(longest_similar_filter_combine_lines_mid_num,longest_similar_filter_combine_lines_points_foot_dis,0,longest_similar_filter_combine_lines_mid_num.size()-1);
//     quick_sort(longest_similar_filter_combine_lines_mid_num,longest_similar_filter_combine_lines_mid_num,0,longest_similar_filter_combine_lines_mid_num.size()-1);
    
//     //how to select the wrong lines 
//     //1, too short and give a single wired direction
//     vector<int> wrong_lines_indexes_vec;
//     get_wrong_lines_indexes(longest_similar_filter_combine_lines,0.2,60,wrong_lines_indexes_vec);
    
//     //remove the wrong short lines 
//     remove_wrong_lines(wrong_lines_indexes_vec, longest_similar_filter_combine_lines);
//     remove_wrong_lines(wrong_lines_indexes_vec, longest_similar_filter_combine_lines_mid_num);
//     remove_wrong_lines(wrong_lines_indexes_vec, longest_similar_filter_combine_lines_points_foot);
//     remove_wrong_lines(wrong_lines_indexes_vec, longest_similar_filter_combine_lines_points_foot_dis);
//     remove_wrong_lines(wrong_lines_indexes_vec, longest_similar_filter_combine_lines_points_num);
    
//     //draw the correct long lines
//     int radius_of_ciclr = 2;
//     for(Vec4f ll : longest_similar_filter_combine_lines){
//         line(sorted_longest_similar_filter_combine_similar_lines_mat,Point(ll[0],ll[1]),Point(ll[2],ll[3]),Scalar(255,255,255),2,LINE_AA);
//         circle(sorted_longest_similar_filter_combine_similar_lines_mat,Point((ll[0]+ll[2])/2,(ll[1]+ll[3])/2),radius_of_ciclr,Scalar(255,255,255),2);
//         circle(sorted_longest_similar_filter_combine_similar_lines_mat,Point(ll[0],ll[1]),4,Scalar(255,255,255),2);
//         circle(sorted_longest_similar_filter_combine_similar_lines_mat,Point(ll[2],ll[3]),6,Scalar(255,255,255),2);
//         radius_of_ciclr++;
//         //std::cout<<"K of this line:"<<(ll[3] - ll[1])/(ll[2] - ll[0])<<std::endl;
//         //std::cout<<"Length of this line:"<<distance_btw_two_points(Point(ll[0],ll[1]),Point(ll[2],ll[3]))<<std::endl<<std::endl;
         
//     }
    
    
    
  
// std::cout<<"------------------Begin Check if some lines lost -----------------------"<<std::endl<<std::endl;
// /*************************** check if there some lines lost *******************************************************/
//    //situation1 : few points lost between two neighber lines
//     vector<Vec4f> lost_vec;//record the lost lines in defualt in situation 1 -------------straight both to two neighber lines
//     vector<int> lost_index_vec;//record the lostlines index in original vector 
//     float cos_line_lost_or_not_limit = 0.986; // situation 1 to decide if lost or not by two neighber lines's angel
//     //situaition 2 :too many lost between two neighber lines at least two lost 
//     vector<int> lost_index_vec_2;//record thr lost lines indexs in longest_similar_filter_combine_lines 
//     vector<vector<Vec4f>> lost_vec_2; //record the lost points indexs in contours2
    
//     for(int i = 0; i<longest_similar_filter_combine_lines.size(); i++) {
//         Vec4f vec1 = longest_similar_filter_combine_lines[i];
//         Vec4f vec2;
//         vector<int> points_num_vec1 = longest_similar_filter_combine_lines_points_num[i];
//         vector<int> points_num_vec2;
//         if(i != longest_similar_filter_combine_lines.size()-1){
//             vec2 = longest_similar_filter_combine_lines[i+1];
//             points_num_vec2 = longest_similar_filter_combine_lines_points_num[i+1];
//         }else{
//             vec2 = longest_similar_filter_combine_lines[0];
//             points_num_vec2 = longest_similar_filter_combine_lines_points_num[0];
            
//         }
//         Point point1_vec = Point((vec1[2] - vec1[0]),(vec1[3]-vec1[1]));
//         Point point2_vec = Point((vec2[2] - vec2[0]),(vec2[3]-vec2[1]));

//         float cos_point_1_2 = (point1_vec.x*point2_vec.x + point1_vec.y * point2_vec.y)/(sqrt(pow(point1_vec.x,2) + pow(point1_vec.y,2))*sqrt(pow(point2_vec.x,2) + pow(point2_vec.y,2)));
//         cos_point_1_2 = cos_point_1_2>0?cos_point_1_2:-cos_point_1_2;//if neighber lines is similar in slope ,there is likely a line lost between them
        
//        /* //situation 2
//         //first think the toomany points lost then think about the few points lost
//         int lost_points_num = vec2 == longest_similar_filter_combine_lines[0]?points_num_vec2[0] - points_num_vec1[points_num_vec1.size()-1]:points_num_vec2[0]+contours.size()-points_num_vec1[points_num_vec1.size()-1];
//         if(lost_points_num >= min_cloud_points_num){
//             vector<Point> lost_points_vec2;
            
            
//             for(int jjjj =points_num_vec1[points_num_vec1.size()-1] ; jjjj<=points_num_vec2[0];jjjj++){
//                 circle(sorted_longest_similar_filter_combine_similar_lines_mat,contours2[jjjj],3,Scalar(0,0,255),2);
//                 lost_points_vec2.push_back(contours2[jjjj]);
//             }
            
            
            
//             if(points_num_vec2[0] < points_num_vec1[points_num_vec1.size()-1]){
                
//                 for(int jjjj = points_num_vec1[points_num_vec1.size()-1]; jjjj<contours2.size();jjjj++){
//                     circle(sorted_longest_similar_filter_combine_similar_lines_mat,contours2[jjjj],3,Scalar(0,0,255),2);
//                 lost_points_vec2.push_back(contours2[jjjj]);
//                 }
                
//                  for(int jjjj =0 ; jjjj<=points_num_vec2[0];jjjj++){
//                 circle(sorted_longest_similar_filter_combine_similar_lines_mat,contours2[jjjj],3,Scalar(0,0,255),2);
//                 lost_points_vec2.push_back(contours2[jjjj]);
//             }
                
//             }
            
//             vector<Vec4f> lost_lines_vec2;
//             dp_find_boundary_keypoints(lost_points_vec2,15,lost_lines_vec2);
            
//             lost_lines_vec2[0][0] = vec1[2];
//             lost_lines_vec2[0][1] = vec1[3];
//             lost_lines_vec2[lost_lines_vec2.size()-1][2] = vec2[0];
//             lost_lines_vec2[lost_lines_vec2.size()-1][3] = vec2[1];
            
            
//             if(lost_lines_vec2.size() >= 2){
//                 lost_index_vec_2.push_back(i);
//                 lost_vec_2.push_back(lost_lines_vec2);
                
//                 for(Vec4f line1 : lost_lines_vec2){
//                 line(sorted_longest_similar_filter_combine_similar_lines_mat,Point(line1[0],line1[1]),Point(line1[2],line1[3]),Scalar(45,85,55),2,LINE_AA);
//             }
//             continue;
//             }
            
//         }*/
        
//       //situation 1
//         if(cos_point_1_2 > cos_line_lost_or_not_limit) {
//             Point p_point = Point((vec1[2] + vec2[0])/2,(vec1[3]+vec2[1])/2);
//             float k1 = (vec1[3] - vec1[1])/(vec1[2] - vec1[0]);
//             float b1 = vec1[1] - k1*vec1[0];
//             float x1 = (k1*(p_point.y - b1) + p_point.x)/(pow(k1,2) + 1);
//             float y1 = k1*x1 + b1;
            
//             float k2 = (vec2[3] - vec2[1])/(vec2[2] - vec2[0]);
//             float b2 = vec2[1] - k2*vec2[0];
//             float x2 = (k2*(p_point.y - b2) + p_point.x)/(pow(k2,2) + 1);
//             float y2 = k2*x2 + b2;
            
//             lost_vec.push_back(Vec4f(x1,y1,x2,y2));
//             lost_index_vec.push_back(i);
//             line(sorted_longest_similar_filter_combine_similar_lines_mat,Point(x1,y1),Point(x2,y2),Scalar(0,0,0),2,LINE_AA);
//             circle(sorted_longest_similar_filter_combine_similar_lines_mat,p_point,2,Scalar(0,0,0),2);
//             continue;
//         }
        
        
//     }
    
    

    
    
//  std::cout<<"------------------find the lost lines and insert in order-----------------------"<<std::endl<<std::endl;   
// /*************************** find the lost lines and insert in order *******************************************************/
//     float distance_lost_limit = 10;
//     vector<int> tem_lost_index_vec = lost_index_vec;
//     cout<<"distance_lost_limit: "<<distance_lost_limit<<endl;
//     list<Vec4f> longest_similar_filter_combine_lines_list;
//     longest_similar_filter_combine_lines_list.assign(longest_similar_filter_combine_lines.begin(),longest_similar_filter_combine_lines.end());
    
//     //situation 1 insert 
//     for(int i = 0; i<lost_vec.size(); i++) {
//         list<Vec4f>::iterator longest_similar_filter_combine_lines_list_iter = longest_similar_filter_combine_lines_list.begin();
//         Vec4f vec1 = lost_vec[i];
//         float min_distance = distance_lost_limit;
//         Vec4f tem_vec = vec1;
        
//         Vec4f vec_first = longest_similar_filter_combine_lines[tem_lost_index_vec[i]];
//         Vec4f vec_later;
//         if(tem_lost_index_vec[i] == longest_similar_filter_combine_lines.size()-1){
//             vec_later = longest_similar_filter_combine_lines[0];
//         }else{
//             vec_later = longest_similar_filter_combine_lines[tem_lost_index_vec[i] + 1];
//         }
        
//         for(Vec4f vec3 : within_lines) {
//             Point first_point = get_common_point_of_two_lines(vec3,vec_first);
//             Point later_point = get_common_point_of_two_lines(vec3,vec_later);
//             Vec4f vec2 = Vec4f(first_point.x,first_point.y,later_point.x,later_point.y);
            
//             order_line_begin_end(vec2,Point(vec1[0],vec1[1]),Point(vec1[2],vec1[3]));
//             if( (distance_btw_two_points(Point(vec1[0],vec1[1]),Point(vec2[0],vec2[1])) + distance_btw_two_points(Point(vec1[2],vec1[3]),Point(vec2[2],vec2[3])))/2<distance_lost_limit) {
//                 float dis = (distance_btw_two_points(Point(vec1[0],vec1[1]),Point(vec2[0],vec2[1]))+distance_btw_two_points(Point(vec1[2],vec1[3]),Point(vec2[2],vec2[3])))/2;
//                 tem_vec = min_distance<dis?tem_vec:vec2;
//                 min_distance = min_distance<dis?min_distance:dis;
                
//             }
//         }
        
        
//         lost_vec[i] = tem_vec;
//         int j = 0;
//         while(j != lost_index_vec[i]+1) {
//             j++;
//             longest_similar_filter_combine_lines_list_iter++;
//         }
//         //insertion
//         longest_similar_filter_combine_lines_list.insert(longest_similar_filter_combine_lines_list_iter,tem_vec);
//         for(int k = 0; k< lost_index_vec.size();k++) {
//             int index = lost_index_vec[k];
//             lost_index_vec[k] = index > lost_index_vec[i]? index+1:index;
//         }
        
//         for(int k = 0; k< lost_index_vec_2.size();k++) {
//             int index = lost_index_vec_2[k];
//             lost_index_vec_2[k] = index > lost_index_vec[i]? index+1:index;
//         }
        
//     }
    
//    /* //situation 2 insert
//     for(int i = 0; i<lost_vec_2.size(); i++) {
       
//         vector<Vec4f> dp_find_lost_lines = lost_vec_2[i];
        
//         //insert
//         list<Vec4f>::iterator longest_similar_filter_combine_lines_list_iter = longest_similar_filter_combine_lines_list.begin();
        
//         int j = 0;
//         while(j != lost_index_vec_2[i]+1) {
//             j++;
//             longest_similar_filter_combine_lines_list_iter++;
//         }
//         //insertion
//         for(Vec4f tem_vec: dp_find_lost_lines){
            
//         longest_similar_filter_combine_lines_list_iter=longest_similar_filter_combine_lines_list.insert(longest_similar_filter_combine_lines_list_iter,tem_vec);
//         longest_similar_filter_combine_lines_list_iter++;
        
//         }
//         for(int k = 0; k< lost_index_vec_2.size();k++) {
//             int index = lost_index_vec_2[k];
//             lost_index_vec_2[k] = index > lost_index_vec_2[i]? index+dp_find_lost_lines.size():index;
//         }
//     }*/

//     //draw lines after find lost and insert
//    /* auto iter_tmp = longest_similar_filter_combine_lines_list.begin();
//     int index_tmp(0);
//     while(iter_tmp != longest_similar_filter_combine_lines_list.end()){
//         if(index_tmp == 8){
//             longest_similar_filter_combine_lines_list.erase(iter_tmp++);
//             longest_similar_filter_combine_lines_list.erase(iter_tmp);
//             break;
//         }
//         iter_tmp++;
        
//         index_tmp++;
//     }*/
    

    
    
//     int radius_of_ciclr1 = 1;
//     for(Vec4f ll : longest_similar_filter_combine_lines_list){
//         line(lost_found_and_sorted_longest_similar_filter_combine_similar_lines_mat,Point(ll[0],ll[1]),Point(ll[2],ll[3]),Scalar(0,0,255),1,LINE_AA);
//         circle(lost_found_and_sorted_longest_similar_filter_combine_similar_lines_mat,Point((ll[0]+ll[2])/2,(ll[1]+ll[3])/2),radius_of_ciclr1,Scalar(0,0,0),1);
//         radius_of_ciclr1 += 1;
         
//     }
    
     
     
    
   
     
     
//  std::cout<<"------------------complete the building polygon and draw them-----------------------"<<std::endl<<std::endl;     
// /*************************** complete the building polygon and draw them *******************************************************/
    
//     //warpAffine(complete_mat,complete_mat,M,Size(complete_mat.cols,complete_mat.rows));

//     vector<Point> complete_points_vec;
//     for(list<Vec4f>::iterator iter = longest_similar_filter_combine_lines_list.begin(); iter!=longest_similar_filter_combine_lines_list.end(); iter++) {
//         Vec4f vec_front = *iter;
//         Vec4f vec_back = (++iter) == longest_similar_filter_combine_lines_list.end()? *longest_similar_filter_combine_lines_list.begin() : *iter;
//         complete_points_vec.push_back(get_common_point_of_two_lines(vec_front,vec_back));
//         iter--;
//     }

//     vector<vector<Point>> complete_points_v_vec = {complete_points_vec};
//     Mat copy_mat;
//     complete_mat.copyTo(copy_mat);
//     polylines(copy_mat,complete_points_vec,true,Scalar(255,255,255),4,LINE_AA);
//     fillPoly(copy_mat,complete_points_v_vec,Scalar(255,102,204));
//     cv::addWeighted(complete_mat,0.5,copy_mat,0.5,0,complete_mat);
//     polylines(complete_mat,complete_points_vec,true,Scalar(0,0,255),1,LINE_AA);
//     //fillPoly(complete_mat,complete_points_vec,Scalar(255,0,0));
    
     
//     std::vector<z_Point> z_points_vec;
//     fetch_td_points(complete_points_vec,image_path_str.c_str(), oesm_path_str.c_str(),dem_path_str.c_str(),z_points_vec,0);
//     z_points_v_vec.push_back(z_points_vec);
    
//     }
    
    
//   //add the dom ground plane
//     cv::Size dom_size = img_1.size();
//    /* std::vector<cv::Point> dom_ground_points_vec{cv::Point(0,0),cv::Point(dom_size.width-1,0),cv::Point(dom_size.width-1,dom_size.height-1),cv::Point(0,dom_size.height-1)};
//     std::vector<z_Point> ttemp_points_vec;
//     fetch_td_points(dom_ground_points_vec,image_path_str.c_str(), oesm_path_str.c_str(),dem_path_str.c_str(),ttemp_points_vec,1);
//     z_points_v_vec.push_back(ttemp_points_vec);*/
    
//    // mat_show("edge_test_mat",edge_test_mat,500);
//     mat_show("alpha_shape_points_mat",alpha_shape_points_mat,500);
//     imwrite(_output+"alpha_shape_"+_name+".tif", alpha_shape_points_mat);
//     mat_show("within_lines_mat",within_lines_mat,500);
//     imwrite(_output+"within_lines_"+_name+".tif", within_lines_mat);
//    // mat_show("similar_lines",test_mat_1,500);
//    // mat_show("combine_similar_lines",test_mat_2,500);
//     mat_show("filter_lines",points_to_lines_test_mat,500);
//     imwrite(_output+"points_to_lines_"+_name+".tif", points_to_lines_test_mat);
//     mat_show("similar_filter_lines",similar_filter_combine_similar_lines_mat,500);
//     imwrite(_output+"similar_"+_name+".tif", similar_filter_combine_similar_lines_mat);
//     mat_show("best_line_in_similar_filter_lines",longest_similar_filter_combine_similar_lines_mat,500);
//     imwrite(_output+"best_"+_name+".tif", longest_similar_filter_combine_similar_lines_mat);
//     mat_show("sorted_best_line_in_similar_filter_lines",sorted_longest_similar_filter_combine_similar_lines_mat,500);
//     imwrite(_output+"sorted_"+_name+".tif", sorted_longest_similar_filter_combine_similar_lines_mat);
//     mat_show("lost_found_sorted_best_line_in_similar_filter_lines",lost_found_and_sorted_longest_similar_filter_combine_similar_lines_mat,500);
//     imwrite(_output+"lost_found_"+_name+".tif", lost_found_and_sorted_longest_similar_filter_combine_similar_lines_mat);
//     mat_show("complete_mat",complete_mat,500);
//     imwrite(_output+"complete_"+_name+ ".tif", complete_mat);
    
    
//     std::cout<<"------------------create the shp file-----------------------"<<std::endl<<std::endl;
//     create_shp((_output+"complete_"+_name+ ".shp").c_str(),z_points_v_vec,32);
    
    
//  std::cout<<"------------------create the obj file-----------------------"<<std::endl<<std::endl;     
// /*************************** create the obj file *******************************************************/
// vector<z_Plane> z_planes_vec;
// vector<vector<z_Point>> normal_est_v_vec;
// int former_point_count = 0;
// int former_normal_count = 0;
// 
// 
// 
// for(int i = 0; i < z_points_v_vec.size(); i++){
//     
//     //bool is_ground = i == z_points_v_vec.size()-1? 1:0;
//     bool is_ground = 0;
//     vector<z_Point> normal_est_vec;
//     resize_ele_and_get_normal_vec(z_points_v_vec[i], normal_est_vec,z_planes_vec, former_point_count, former_normal_count,is_ground);
//     former_point_count += z_points_v_vec[i].size();
//     former_normal_count += normal_est_vec.size();
//     normal_est_v_vec.push_back(normal_est_vec);
// }
// 
// 
// write_to_objfile(obj_file_path.c_str(), z_points_v_vec, normal_est_v_vec,z_planes_vec,dom_size);



    
    //write the result to file
    //imwrite(output_path_1, within_lines_mat);
    //imwrite(output_path_2, points_to_lines_test_mat);
    //imwrite(output_path_3, sorted_longest_similar_filter_combine_similar_lines_mat);
    //imwrite(output_path_4,complete_mat);

    return 0;

}
