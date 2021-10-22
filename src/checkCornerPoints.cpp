#include "checkCornerPoints.hpp"


#if 0


//dictance between two points
/*
 prama: Point p_point1-- input point 1
 prama: Point p_point2-- input point 2
 return: float distance -- distance between two points 
 */
float distance_btw_two_points(Point p_point1, Point p_point2){
     float distance = sqrt(pow((p_point1.x - p_point2.x),2) + pow((p_point1.y - p_point2.y),2));
     return distance;
}


//decide two lines 's similarity
/*
 prama: Vec4f &vec1 -- input line 1
 prama: Vec4f &vec2 -- input line 2
 prama: float distance_limit -- the limit to describe the distance btween two lines
 prama: float line_cos_limit -- the limit to describe the angle between two lines
 prama: float line_b_limit -- the limit to describe the nodal increment of two lines
 return: bool decide_two_lines_similarity -- true:is_similary false:not_similary
 */
bool decide_two_lines_similarity(Vec4f &vec1, Vec4f &vec2,float distance_limit,float line_cos_limit,float line_b_limit){
    float fenmu_buwei_0_point_1 = (vec1[2] - vec1[0]);
    float fenmu_buwei_0_point_2 = (vec2[2] - vec2[0]);
    
    float k1 = fenmu_buwei_0_point_1 == 0?(vec1[3] - vec1[1])/0.01:(vec1[3] - vec1[1])/(vec1[2] - vec1[0]), k2 = fenmu_buwei_0_point_2 == 0?(vec2[3] - vec2[1])/0.01:(vec2[3] - vec2[1])/(vec2[2] - vec2[0]);
    float b1 = vec1[1] - k1*vec1[0], b2 = vec2[1] - k2*vec2[0];
    
    float temp_x_1 = vec1[2] > vec1[0]? vec1[0] : vec1[2];
    float temp_x_2 = vec2[2] > vec2[0]? vec2[0] : vec2[2];
    float temp_x = temp_x_1>temp_x_2?temp_x_1:temp_x_2;
    
    float temp_y_1 = vec1[3] > vec1[1]? vec1[1] : vec1[3];
    float temp_y_2 = vec2[3] > vec2[1]? vec2[1] : vec2[3];
    float temp_y = temp_y_1>temp_y_2?temp_y_1:temp_y_2;
    
    float temp_1 = ((k1+k2)/2 > 1) || ((k1+k2)/2 < -1)? (temp_y-b1)/k1 : k1*temp_x + b1; 
    float temp_2 = ((k1+k2)/2 > 1) || ((k1+k2)/2 < -1)? (temp_y-b2)/k2 : k2*temp_x + b2; 
    
    Point line1_begin = Point(vec1[0],vec1[1]);
    Point line1_end = Point(vec1[2],vec1[3]);
    Point line1_mid = Point((vec1[2] + vec1[0])/2,(vec1[1]+vec1[3])/2);
    
    Point line2_begin = Point(vec2[0],vec2[1]);
    Point line2_end = Point(vec2[2],vec2[3]);
    Point line2_mid = Point((vec2[2] + vec2[0])/2,(vec2[1]+vec2[3])/2);
    
    //k of line between mid points should be like with k1 and k2
    
    float line1_length = distance_btw_two_points(line1_begin,line1_end);
    float line2_length = distance_btw_two_points(line2_begin,line2_end);
    float limit_decide_neborLines_or_not = (line1_length+line2_length)/2 + distance_limit;
    
    Point point1_vec = Point((vec1[2] - vec1[0]),(vec1[3]-vec1[1]));
    Point point2_vec = Point((vec2[2] - vec2[0]),(vec2[3]-vec2[1]));

    float cos_point_1_2 = (point1_vec.x*point2_vec.x + point1_vec.y * point2_vec.y)/(sqrt(pow(point1_vec.x,2) + pow(point1_vec.y,2))*sqrt(pow(point2_vec.x,2) + pow(point2_vec.y,2)));
    cos_point_1_2 = cos_point_1_2>0?cos_point_1_2:-cos_point_1_2;
    
    bool res_length = !((distance_btw_two_points(line1_mid,line2_mid) > limit_decide_neborLines_or_not));
    bool res_k_b =  cos_point_1_2>line_cos_limit&&sqrt(pow((temp_1 - temp_2), 2)) < line_b_limit;
    
    //if(res_length&&res_k_b){
    //cout<<"k1 , k2 :"<<k1<<" "<<k2<<endl;
    //cout<<"b1 , b2 :"<<b1<<" "<<b2<<endl<<endl;;
    //}
    
    return res_length&&res_k_b;
}

//get the similary lines together(combine the similary lines)
/*
 prama: std::vector<Vec4f> &similar_lines -- the input similary lines vector
 return: the line resulting in input lines combined
 */
Vec4f  get_lines_togethor(std::vector<Vec4f> &similar_lines){
    std::vector<cv::Point> point_vec;
    Vec4f together_line_para;    

    //get the begin and end point of similar_lines
    cv::Point beg_point;
    cv::Point ed_point;
    int i = 0;
    for(Vec4f vec : similar_lines){
        cv::Point begin_point = cv::Point(vec[0],vec[1]);
        cv::Point end_point = cv::Point(vec[2],vec[3]);
        
        point_vec.push_back(begin_point);
        point_vec.push_back(end_point);
        
        if(i == 0){
            beg_point = cv::Point(vec[0],vec[1]);
            ed_point = cv::Point(vec[2],vec[3]);
        }else{
            beg_point = beg_point.x > vec[0]||(beg_point.x== vec[0] && !(beg_point.y> vec[1]))? cv::Point(vec[0],vec[1]) : beg_point;
            ed_point = ed_point.x < vec[2]||(ed_point.x== vec[2] && !(ed_point.y< vec[3]))? cv::Point(vec[2],vec[3]) : ed_point;
        }
        
        i++;
    }
    
    	cv::fitLine(point_vec, together_line_para, cv::DIST_L2, 0, 1e-2, 1e-2);

        double k = together_line_para[1] / together_line_para[0];
        double b = together_line_para[3] - k*together_line_para[2];

        float x0 = (k*(beg_point.y - b) + beg_point.x)/(pow(k,2) + 1);
        float y0 = k*x0 + b;
        float x1 = (k*(ed_point.y - b) + ed_point.x)/(pow(k,2) + 1);
        float y1 = k*x1 + b;
        
        return Vec4f(x0,y0,x1,y1);
}



//decide which line is within the pcd boundary area(extended by blur)
/*
 prama: Mat &pcd_boudary_mat -- input mat describe the boundary points area with OpenCV method(instead of pcd method)
 prama: Vec4f &vec -- input line
  prama: float prameter -- prameter decide in or out
 return: bool decide_line_within_pcdBoudaryArea_or_not -- true:line within boundary area false: without
 */
bool decide_line_within_pcdBoudaryArea_or_not(Mat &pcd_boudary_mat,Vec4f &vec,float prameter){
    uchar gray_0 = pcd_boudary_mat.at<uchar>(vec[1],vec[0]);
    uchar gray_4 = pcd_boudary_mat.at<uchar>(vec[3],vec[2]);
    uchar gray_2 = pcd_boudary_mat.at<uchar>((vec[1]+vec[3])/2,(vec[0]+ vec[2])/2);
    uchar gray_1 = pcd_boudary_mat.at<uchar>((3*vec[1]+vec[3])/4,(3*vec[0]+ vec[2])/4);
    uchar gray_3 = pcd_boudary_mat.at<uchar>((vec[1]+3*vec[3])/4,(vec[0]+3*vec[2])/4);
    
    uchar gray_0_m_1 = pcd_boudary_mat.at<uchar>((7*vec[1]+vec[3])/8,(7*vec[0]+ vec[2])/8);
    uchar gray_1_m_2 = pcd_boudary_mat.at<uchar>((5*vec[1]+3*vec[3])/8,(5*vec[0]+ 3*vec[2])/8);
    uchar gray_2_m_3 = pcd_boudary_mat.at<uchar>((3*vec[1]+5*vec[3])/8,(3*vec[0]+ 5*vec[2])/8);
    uchar gray_3_m_4 = pcd_boudary_mat.at<uchar>((1*vec[1]+7*vec[3])/8,(1*vec[0]+ 7*vec[2])/8);
    
    /*float count = sqrt(pow((int)(vec[2] - vec[0]),2)) > sqrt(pow((int)(vec[3] - vec[1]),2))? sqrt(pow((int)(vec[2] - vec[0]),2)):sqrt(pow((int)(vec[3] - vec[1]),2));
    int type = sqrt(pow((int)(vec[2] - vec[0]),2)) > sqrt(pow((int)(vec[3] - vec[1]),2))? 0:1;
    
    float min_x = vec[0] >= vec[2]?vec[0]:vec[2];
    float min_y = vec[1] >= vec[3]?vec[1]:vec[3];
    
    float in_num = 0;
    //int max_in_fluent_num = 0;
    
    for(float i = 0; i< count; i++){
        uchar gray = type == 0?pcd_boudary_mat.at<uchar>(min_y + (i/count)*sqrt(pow((int)(vec[3] - vec[1]),2)),min_x + i) : pcd_boudary_mat.at<uchar>(min_y + i,min_x + (i/count)*sqrt(pow((int)(vec[2] - vec[0]),2)));
        
        in_num = gray > 254? in_num+1:in_num;
    }*/
    
    bool res = (gray_0+gray_1+gray_2+gray_3+gray_4+gray_0_m_1+gray_1_m_2+gray_2_m_3+gray_3_m_4) > (255-1)*(prameter*9);
    //bool res = in_num/count > prameter;
    return res;
    
}





//mat to pcd (to use the pcd methods)
/*
 @prama: Mat &inputmat -- input mat
 @prama: pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud -- output point cloud
 @prama: int multiple_num -- zoom the z value(defualt 1)
 */
void mat_to_pcd(Mat &inputmat,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,int multiple_num){
    
     // Fill in the cloud data
  cloud->width  = inputmat.cols;
  cloud->height = inputmat.rows;
  cloud->points.resize (cloud->width * cloud->height);

  // Generate the data

      
       MatIterator_<float> it, end;
       int i = 0;
        for (it = inputmat.begin<float>(), end = inputmat.end<float>(); it != end; it++)
        {
            cloud->points[i].x = i%inputmat.cols;
            cloud->points[i].y = i/inputmat.cols;
            cloud->points[i].z = (*it) * multiple_num;
            i++;
        } 
}







//find the boundary in mat with sobel kernals in 8 directions
/*
 @prama: Mat &img -- input mat
 @prama: Mat &sobel_out -- output mat after convolution with sobel kernals 
 */
int sobel_find_boundary(Mat &img, Mat &sobel_out){
    
    Mat gray;
    gaussianBlur_and_ctgray(img, gray);
    
    Mat gray_out_1;
    Mat kernel_1 = (Mat_<int>(5,5)<<0,0,0,0,0,-1,-2,-4,-2,-1,0,0,0,0,0,1,2,4,2,1,0,0,0,0,0);
    filter2D(gray,gray_out_1,-1,kernel_1,Point(-1,-1),0,0);
    
    Mat gray_out_2;
    Mat kernel_2 = (Mat_<int>(5,5)<<0,0,0,0,0,0,-2,-4,-2,0,-1,-4,0,4,1,0,2,4,2,0,0,0,0,0,0);
    filter2D(gray,gray_out_2,-1,kernel_2,Point(-1,-1),0,0);
    
    Mat gray_out_3;
    Mat kernel_3 = (Mat_<int>(5,5)<<0,0,0,-1,0,0,-2,-4,0,1,0,-4,0,4,0,-1,0,4,2,0,0,1,0,0,0);
    filter2D(gray,gray_out_3,-1,kernel_3,Point(-1,-1),0,0);
    
    Mat gray_out_4;
    Mat kernel_4 = (Mat_<int>(5,5)<<0,0,-1,0,0,0,-2,-4,2,0,0,-4,0,4,0,0,-2,4,2,0,0,0,1,0,0);
    filter2D(gray,gray_out_4,-1,kernel_4,Point(-1,-1),0,0);
    
    Mat gray_out_5;
    Mat kernel_5 = (Mat_<int>(5,5)<<0,-1,0,1,0,0,-2,0,2,0,0,-4,0,4,0,0,-2,0,2,0,0,-1,0,1,0);
    filter2D(gray,gray_out_5,-1,kernel_5,Point(-1,-1),0,0);
    
    Mat gray_out_6;
    Mat kernel_6 = (Mat_<int>(5,5)<<0,0,1,0,0,0,-2,4,2,0,0,-4,0,4,0,0,-2,-4,2,0,0,0,-1,0,0);
    filter2D(gray,gray_out_6,-1,kernel_6,Point(-1,-1),0,0);
    
    Mat gray_out_7;
    Mat kernel_7 = (Mat_<int>(5,5)<<0,1,0,0,0,-1,0,4,2,0,0,-4,0,4,0,0,-2,-4,0,1,0,0,0,-1,0);
    filter2D(gray,gray_out_7,-1,kernel_7,Point(-1,-1),0,0);
    
    Mat gray_out_8;
    Mat kernel_8 = (Mat_<int>(5,5)<<0,0,0,0,0,0,2,4,2,0,-1,-4,0,4,1,0,-2,-4,-2,0,0,0,0,0,0);
    filter2D(gray,gray_out_8,-1,kernel_8,Point(-1,-1),0,0);


    Mat res_1_2;
    cv::addWeighted(gray_out_1,0.5,gray_out_2,0.5,0,res_1_2);
    Mat res_3_4;
    cv::addWeighted(gray_out_3,0.5,gray_out_4,0.5,0,res_3_4);
    Mat res_5_6;
    cv::addWeighted(gray_out_5,0.5,gray_out_6,0.5,0,res_5_6);
    Mat res_7_8;
    cv::addWeighted(gray_out_7,0.5,gray_out_8,0.5,0,res_7_8);
    
    Mat res_1_2_3_4;
    cv::addWeighted(res_1_2,0.5,res_3_4,0.5,0,res_1_2_3_4);
    Mat res_5_6_7_8;
    cv::addWeighted(res_5_6,0.5,res_7_8,0.5,0,res_5_6_7_8);
    

    cv::addWeighted(res_1_2_3_4,0.5,res_5_6_7_8,0.5,0,sobel_out);
    cv::threshold(sobel_out, sobel_out, 50, 255,THRESH_BINARY);
    
    //mat_show("sobel_out",sobel_out,500);
    
    return 1;

    
}
 

 
 
//get each boundary in img_out after canny or sobel convolution and regular the boundary  with DP method
/*
@prama: vector<vector<Point>> &contours -- vector to record the contours of each boundary
@prama: int level -------dp level
 */
int dp_find_boundary_keypoints(vector<Point> &contours,int level, vector<Vec4f> &res_vec){
    
	
	//
	vector<Point> contours_ploy(contours.size());
    
 
      approxPolyDP(contours, contours_ploy, level, true);
      
      for(int i =0 ; i<contours_ploy.size()-1; i++){
          res_vec.push_back(Vec4f(contours_ploy[i].x,contours_ploy[i].y,contours_ploy[i+1].x,contours_ploy[i+1].y));
    }
      
	return 1;

}


//find the similar lines in input lines vector and group them
/*
 prama: float length_limit_each_line -- min length limit of lines, lines shorter than it will be ignored
 prama: float distance_limit -- the limit to describe the distance btween two lines
 prama: float line_cos_limit -- the limit to describe the angle between two lines
 prama: float line_b_limit -- the limit to describe the nodal increment of two lines
 prama: vector<Vec4f> &within_lines -- input lines vec
 prama: vector<vector<Vec4f>> &lines_v_vec -- output lines vec's vec(similar lines in groups)
 */
void find_the_similar_lines_in_lines_vec(float length_limit_each_line,float length_limit_between_two_lines,float lines_cos_limit,float lines_b_limit,vector<Vec4f> &within_lines,vector<vector<Vec4f>> &lines_v_vec){
    while(!within_lines.empty())
    {
        Vec4f vec_1 = within_lines[0];
        float length = distance_btw_two_points(Point(vec_1[0],vec_1[1]),Point(vec_1[2],vec_1[3]));
        if(length < length_limit_each_line){
            within_lines.erase(within_lines.begin());
            continue;
        }
            
        if(lines_v_vec.empty()){
            vector<Vec4f> nnew_lines_vec;
            nnew_lines_vec.push_back(vec_1);
            lines_v_vec.push_back(nnew_lines_vec);
            within_lines.erase(within_lines.begin());
            continue;
        }
        
        int similar_found_or_not = 0;
        for(int i =0 ; i< lines_v_vec.size();i++){
            for(Vec4f vec_2 : lines_v_vec[i]){
                if(decide_two_lines_similarity(vec_1,vec_2,length_limit_between_two_lines,lines_cos_limit,lines_b_limit)){
                    lines_v_vec[i].push_back(vec_1);
                    similar_found_or_not = 1;
                    break;
                }
            }
            
            if(similar_found_or_not == 1){
                break;
            }
        }
        
        if(similar_found_or_not == 0){
            vector<Vec4f> new_lines_vec;
            new_lines_vec.push_back(vec_1);
            lines_v_vec.push_back(new_lines_vec);
        }
        
        within_lines.erase(within_lines.begin());
    }    
    
    cout<<"lines_v_vec size :"<<lines_v_vec.size()<<endl;
}



 //get the cross over point of two lines
 /*
  prama: Vec4f vec1 -- input line 1
  prama: Vec4f vec2 -- input line 2
  return: the crossover point 
  */
 
Point get_common_point_of_two_lines(Vec4f vec1,Vec4f vec2){
    if(vec1[2] == vec1[0] || vec2[2] == vec2[0]){
        Point res_poi = vec1[2] == vec1[0]?Point(vec1[0],((vec2[3] - vec2[1])/(vec2[2]-vec2[0]))*vec1[0] + vec2[1] - ((vec2[3] - vec2[1])/(vec2[2]-vec2[0]))*vec2[0]):Point(vec2[0],((vec1[3] - vec1[1])/(vec1[2]-vec1[0]))*vec2[0] + vec1[1] - ((vec1[3] - vec1[1])/(vec1[2]-vec1[0]))*vec1[0]);
        return res_poi;
    }
    float k1 = (vec1[3] - vec1[1])/(vec1[2]-vec1[0]);
    float b1 = vec1[1] - k1*vec1[0];
    float k2 = (vec2[3] - vec2[1])/(vec2[2]-vec2[0]);
    float b2 = vec2[1] - k2*vec2[0];
    

    if(k1 == k2){
        cout<<"cant't get common point of parallel lines"<<endl;
        return Point((vec1[2]+vec2[0])/2,(vec1[3] + vec2[1])/2);
    }
    
    return Point((b2-b1)/(k1-k2),k1*(b2-b1)/(k1-k2)+b1);
}
 
//order the line's begin and end point with foot_vec_points's order
void order_line_begin_end(Vec4f &line_vec,Point begin_foot,Point end_foot){
    Point line_begin = Point(line_vec[0],line_vec[1]);
    Point line_end = Point(line_vec[2],line_vec[3]);
    line_vec =  distance_btw_two_points(line_begin,begin_foot) < distance_btw_two_points(line_begin,end_foot)? line_vec:Vec4f(line_end.x,line_end.y,line_begin.x,line_begin.y);
}



//get the longest line index in a lines vec
/*
 @prama: std::vector<Vec4f> lines_vec : input lines vector
 @return: longest line's index in vector
 */
int get_longest_line_index_in_vec(std::vector<Vec4f> lines_vec){
    
    float dis = 0.0;
    int index = 0;
    for(int i = 0; i < lines_vec.size();i++){
        float temp_dis = distance_btw_two_points(Point(lines_vec[i][0],lines_vec[i][1]),Point(lines_vec[i][2],lines_vec[i][3]));
        
        index  = dis>temp_dis?index:i;
        dis = dis>temp_dis?dis:temp_dis;
    }
    return index;
}


//get the cos angle of two lines 
/*
 @prama: Vec4f line1 : input line 1
 @prama: Vec4f line2 : input line 2
 @return: cos angle value btw line1 and line2
 */
float get_cos_btw_two_lines(Vec4f line1,Vec4f line2){
    
        Point point1_vec = Point((line1[2] - line1[0]),(line1[3]-line1[1]));
        Point point2_vec = Point((line2[2] - line2[0]),(line2[3]-line2[1]));

        return (point1_vec.x*point2_vec.x + point1_vec.y * point2_vec.y)/(sqrt(pow(point1_vec.x,2) + pow(point1_vec.y,2))*sqrt(pow(point2_vec.x,2) + pow(point2_vec.y,2)));
}


//get the wrong direction and short lines indexes in a lines vec
/*
 @prama: vector<Vec4f> lines_vec -- input lines vec
 @prama: float k_difrence -- a line regarded as wrong line when it's k value is diffrent more than k-limit with other lines
 @prama: float length_limit -- a wrong line can't longer than length-limit
 @prama: vector<int> &delete_index_vec -- vector record the wrong lines' indexes
 */
void get_wrong_lines_indexes(vector<Vec4f> lines_vec,float k_difrence,float length_limit,vector<int> &delete_index_vec){
    
    vector<vector<int>> index_v_vec; 
    
    
    for(int i = 0; i < lines_vec.size(); i++){
        
        if(i == 0){
            index_v_vec.push_back(vector<int>{i});
            continue;
        }
        
        int fount_or_not = 0;

        Vec4f ll = lines_vec[i];
        for(int j = 0; j< index_v_vec.size();j++){
            
                Vec4f mm = lines_vec[index_v_vec[j][0]];
                float k_m = (mm[3]-mm[1])/(mm[2]-mm[0]);
                float k_l = (ll[3]-ll[1])/(ll[2]-ll[0]);
                if(fabs(k_m - k_l) < k_difrence){
                    index_v_vec[j].push_back(i);
                    fount_or_not = 1;
                    break;
                }
        }
        
        if(fount_or_not == 0){
         index_v_vec.push_back(vector<int>{i});
        }
    }
    
    for(vector<int> vv : index_v_vec){
        if(vv.size() == 1 && distance_btw_two_points(Point(lines_vec[vv[0]][0],lines_vec[vv[0]][1]),Point(lines_vec[vv[0]][2],lines_vec[vv[0]][3])) < length_limit){
            delete_index_vec.push_back(vv[0]);
        }
    }
}

//read json
/*
 @prama:std::string json_file_path -- json file path
 @prama:std::vector<std::string> &str_vec -- json string vec
 */

void readFileJson(std::string json_file_path, std::vector<std::string> &str_vec)
{
	Json::Reader reader;
	Json::Value root;
 
	//从文件中读取，保证当前文件有demo.json文件  
	ifstream in(json_file_path, ios::binary);
 
	if (!in.is_open())
	{
		cout << "Error opening file\n";
		return;
	}
 
	if (reader.parse(in, root))
	{

			std::string input_image_tif = root["inputdata"][0].asString();
			std::string input_cloud_pcd = root["inputdata"][1].asString();
            std::string input_oesm_tif = root["inputdata"][2].asString();
			std::string input_dem_tif = root["inputdata"][3].asString();
            std::string output = root["output"].asString();
            str_vec.push_back(input_image_tif);
            str_vec.push_back(input_cloud_pcd);
            str_vec.push_back(input_oesm_tif);
            str_vec.push_back(input_dem_tif);
            str_vec.push_back(output);
	
 
		cout << "Reading JSON Complete!" << endl;
	}
	else
	{
		cout << "JSON parse error\n" << endl;
	}
 
	in.close();
}


#endif 