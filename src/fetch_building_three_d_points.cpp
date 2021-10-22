#if 0

#include "fetch_building_three_d_points.hpp"
#include "eb_transform.hpp"

//change the pixel coodinate of building corner points to geo_X geo_Y elevation
/*
 @prama: vector<cv::Point> &points_vec -- input 2d points vec;
 @prama: const char * dom_filepath -- input dom file path;
 @prama: const char * oesm_filepath -- input oesm file path;
 @prama: const char * dem_filepath -- input dem file path;
 @prama: vector<z_Point>&z_point_vec -- vec to record result 3d points;
 @prama: bool is_ground -- check if the plane is a single ground;
 */
void fetch_td_points(vector<cv::Point> &points_vec,const char * dom_filepath, const char * oesm_filepath,const char * dem_filepath,vector<z_Point>&z_point_vec,bool is_ground){
    
    double dom_trans[6];
    double oesm_trans[6];
    double dem_trans[6];
    getTrans_of_TiffFile(dom_filepath,dom_trans);
    getTrans_of_TiffFile(oesm_filepath,oesm_trans);
    getTrans_of_TiffFile(dem_filepath,dem_trans);
    
    
    GDALDataset* poDataset1;   //GDAL数据集
	GDALAllRegister();  //注册所有的驱动
	poDataset1 = (GDALDataset*)GDALOpen(oesm_filepath, GA_ReadOnly);
	float* paf1 = new float[1];
    
    GDALDataset* poDataset2;   //GDAL数据集
	GDALAllRegister();  //注册所有的驱动
	poDataset2 = (GDALDataset*)GDALOpen(dem_filepath, GA_ReadOnly);
	float* paf2 = new float[1];
    

    vector<z_Point> z_floor_point_vec;
    
    for(cv::Point pp : points_vec){
    float geoX = get_geoX_geoY_frm_row_column(dom_trans,pp.x,pp.y,2);
    float geoY = get_geoX_geoY_frm_row_column(dom_trans,pp.x,pp.y,1);
    
    
    float oesm_x = get_row_column_frm_geoX_geoY(oesm_trans,geoX,geoY,1);
    float oesm_y = get_row_column_frm_geoX_geoY(oesm_trans,geoX,geoY,2);
    
    float dem_x = get_row_column_frm_geoX_geoY(dem_trans,geoX,geoY,1);
    float dem_y = get_row_column_frm_geoX_geoY(dem_trans,geoX,geoY,2);
    
    
    
    
    float oesm_ele = is_ground? get_elevation_frm_row_column(poDataset2,paf2,dem_x,dem_y): get_elevation_frm_row_column(poDataset1,paf1,oesm_x,oesm_y);
    float dem_ele =  get_elevation_frm_row_column(poDataset2,paf2,dem_x,dem_y);
    
    
    z_Point three_d_point = z_Point(geoX,geoY,oesm_ele);
    z_point_vec.push_back(three_d_point);
    
    z_Point three_d_point1 = z_Point(geoX,geoY,dem_ele);
    z_floor_point_vec.push_back(three_d_point1);
    }
    
    if(is_ground){
    delete[] paf1;
    delete[] paf2;
    paf1 = NULL;
    paf2 = NULL;
    
    GDALClose(poDataset1);
    GDALClose(poDataset2);
    return;
    
        
    }
    z_point_vec.insert(z_point_vec.end(),z_floor_point_vec.begin(),z_floor_point_vec.end());
    
    delete[] paf1;
    delete[] paf2;
    paf1 = NULL;
    paf2 = NULL;
    
    GDALClose(poDataset1);
    GDALClose(poDataset2);
    
    
}


//get the normal vector with three points in plane
/*
 @prama: z_Point &v1 v2 v3 -- three points construct a plane
 @return: the normal vector
 */
z_Point Cal_Normal_3D(z_Point &v1,z_Point &v2,z_Point &v3)
{
	//v1(n1,n2,n3);
	//平面方程: na * (x – n1) + nb * (y – n2) + nc * (z – n3) = 0 ;
	double na = (v2.geo_y - v1.geo_y)*(v3.elevation - v1.elevation) - (v2.elevation - v1.elevation)*(v3.geo_y - v1.geo_y);
	double nb = (v2.elevation - v1.elevation)*(v3.geo_x - v1.geo_x) - (v2.geo_x - v1.geo_x)*(v3.elevation - v1.elevation);
	double nc = (v2.geo_x - v1.geo_x)*(v3.geo_y - v1.geo_y) - (v2.geo_y - v1.geo_y)*(v3.geo_x - v1.geo_x);

	//平面法向量
	return z_Point(na,nb,nc);
}


//reset the elevation of groof and floor to the average value and get the normalization vector of each plane
/*
 @pram: vector<z_Point>&z_point_vec -- the vec records the 3d points of building corner points in order (groof -> floor)
 @prama: vector<z_Point>&normal_est_vec -- the vec records the normal vector of each planes
 @prama: vector<z_Plane>&z_planes_vec -- the vec record the idex of each plane(normal vector index  and   corner points index)
 @prama: int former_point_count,int former_normal_count -- there are many buildings ,add the former count before cal the index
 @prama: bool is_ground -- check if the plane is a single ground;
 */
void resize_ele_and_get_normal_vec(vector<z_Point>&z_point_vec, vector<z_Point>&normal_est_vec,vector<z_Plane>&z_planes_vec,int former_point_count,int former_normal_count,bool is_ground){
    
    //if it is  the ground then set plane and return 
    if(is_ground){
        //reset the elevatin
         int count = -1;
      float ground_ele_sum = 0;
    while((count++) != z_point_vec.size()){
        
       ground_ele_sum+=z_point_vec[count].elevation;
    }
    
    float ground_ele_average = ground_ele_sum/z_point_vec.size();
   for(int i = 0; i < z_point_vec.size(); i++){
        
       z_point_vec[i].elevation = ground_ele_average;
    }
        
        
        normal_est_vec.push_back(z_Point(0,0,1));
        vector<int> temp_groundpoints_index_vec;
        vector<int> temp_groundcontext_index_vec;
            for(int z = 1; z <= z_point_vec.size();z++){
                temp_groundpoints_index_vec.push_back(z + former_point_count);
                temp_groundcontext_index_vec.push_back(z+1);
            }
            z_planes_vec.push_back(z_Plane(temp_groundpoints_index_vec,1+former_normal_count,temp_groundcontext_index_vec));
            return;
        
    }
    
//reset the average elevation of groof and floor (not considering the tilt buildings)
    int count = 0;
    float dem_ele_sum = 0;
    float oesm_ele_sum = 0;
    while(count != z_point_vec.size()){
        if(count < z_point_vec.size()/2)
            oesm_ele_sum+=  z_point_vec[count].elevation;
        else
            dem_ele_sum += z_point_vec[count].elevation;
       
        count++;
    }
    
    float dem_average_ele = 2*dem_ele_sum/z_point_vec.size();
    float oesm_average_ele = 2*oesm_ele_sum/z_point_vec.size();
    
    
    std::cout<<"dem_average_ele: "<<dem_average_ele<<"    "<<"oesm_average_ele: "<<oesm_average_ele<<endl;
    
    /****************cal normal of each plane***************/
    for(int i = 0; i < z_point_vec.size(); i++){
        
        if(i < z_point_vec.size()/2)
            z_point_vec[i].elevation = oesm_average_ele;
        else
            z_point_vec[i].elevation = dem_average_ele;
    }
    
    
    //groof, as for now, it is regarded as one single plane(not considering the details ,nor to the floor)
    normal_est_vec.push_back(z_Point(0,0,1));
    //floor
    normal_est_vec.push_back(z_Point(0,0,-1));
    
    //wall
    for(int j =0; j < z_point_vec.size()/2; j++){
        
        if(j!=z_point_vec.size()/2-1){
            normal_est_vec.push_back(Cal_Normal_3D(z_point_vec[j+1],z_point_vec[j],z_point_vec[j+z_point_vec.size()/2]));
        }else{
            normal_est_vec.push_back(Cal_Normal_3D(z_point_vec[0],z_point_vec[j],z_point_vec[j+z_point_vec.size()/2]));
        }
    }
    
    /***********set plane args**********/
    
    
    for(int k= 0; k < normal_est_vec.size(); k++){
        
        std::vector<int> points_index;
        if(k == 0){
            //groof
            vector<int> temp_points_index_vec;
            std::vector<int> temp_context_index_vec;
            for(int z = 1; z <= z_point_vec.size()/2;z++){
                temp_points_index_vec.push_back(z + former_point_count);
                temp_context_index_vec.push_back(1);
            }
            z_planes_vec.push_back(z_Plane(temp_points_index_vec,k+1+former_normal_count,temp_context_index_vec));
        }else if(k == 1){
            //floor
            vector<int> temp_points_index_vec;
            std::vector<int> temp_context_index_vec;
            for(int z = z_point_vec.size()/2+1; z <= z_point_vec.size();z++){
                temp_points_index_vec.push_back(z +former_point_count);
                temp_context_index_vec.push_back(1);
            }
            z_planes_vec.push_back(z_Plane(temp_points_index_vec,k+1+former_normal_count,temp_context_index_vec));
        }else{
            //wall
            //floor
            vector<int> temp_points_index_vec;
            std::vector<int> temp_context_index_vec{1,1,1,1};
            if(k!=normal_est_vec.size()-1){
                temp_points_index_vec.push_back(k+former_point_count);
                temp_points_index_vec.push_back(k-1+former_point_count);
                temp_points_index_vec.push_back(k-1+z_point_vec.size()/2 + former_point_count);
                temp_points_index_vec.push_back(k+z_point_vec.size()/2 + former_point_count);
            }else{
                temp_points_index_vec.push_back(1+ former_point_count);
                temp_points_index_vec.push_back(k-1+ former_point_count);
                temp_points_index_vec.push_back(k-1+z_point_vec.size()/2+ former_point_count);
                temp_points_index_vec.push_back(1+z_point_vec.size()/2+ former_point_count);
            }
            z_planes_vec.push_back(z_Plane(temp_points_index_vec,k+1+former_normal_count,temp_context_index_vec));
        }
            
            
            
    }
    
}

//create the obj file
/*
 @prama: const char * obj_file_path -- out file path
 @pram: vector<<z_Point>>&z_point_vec -- the v_vec records the 3d points of building corner points in order (groof -> floor)
 @prama: vector<<z_Point>>&normal_est_vec -- the v_vec records the normal vector of each planes
 @prama: vector<z_Plane>&z_planes_vec -- the vec record the idex of each plane(normal vector index  and   corner points index) 
 @prama: cv::Size size -- the input dom size
 */
void write_to_objfile(const char * obj_file_path, vector<vector<z_Point>>&z_point_v_vec, vector<vector<z_Point>>&normal_est_v_vec,vector<z_Plane>&z_planes_vec,cv::Size size){
    
    ofstream out_obj_file(obj_file_path);
    int precision = 10;
    if(out_obj_file){
        //write points
        out_obj_file<<"g default"<<std::endl;
        
        for(vector<z_Point> z_point_vec : z_point_v_vec){
        for(z_Point zz : z_point_vec){
            out_obj_file<<"v "<<setprecision(precision)<<zz.geo_x<<" "<<setprecision(precision)<<zz.geo_y<<" "<<setprecision(precision)<<zz.elevation<<std::endl;
        }
        }
        
        //write context
        out_obj_file<<"vt -1 -1"<<std::endl;
        out_obj_file<<"vt 0 0"<<std::endl;
        out_obj_file<<"vt "<<size.width-1<<" 0"<<std::endl;
        out_obj_file<<"vt "<<size.width-1<<" "<<size.height-1<<std::endl;
        out_obj_file<<"vt 0 "<<size.height-1 <<std::endl;

        
        //write normal vector
        for(vector<z_Point> normal_est_vec : normal_est_v_vec){
        for(z_Point zz : normal_est_vec){
        
            out_obj_file<<"vn "<<setprecision(precision)<<zz.geo_x<<" "<<setprecision(precision)<<zz.geo_y<<" "<<setprecision(precision)<<zz.elevation<<std::endl;
            
        }
        }
        
        //write index
        for(int i =0; i< z_planes_vec.size();i++){
            out_obj_file<<"s "<<i+1<<std::endl;
            out_obj_file<<"f ";
            for(int k = 0; k<z_planes_vec[i].points_index_vec.size();k++){
                int point_index = z_planes_vec[i].points_index_vec[k];
                int context_index = z_planes_vec[i].context_index_vec[k];
                out_obj_file<<point_index<<"/"<<context_index<<"/"<<z_planes_vec[i].nor_index<<" ";
            }
            out_obj_file<<std::endl<<std::endl;
        }
        
    }
}


#endif