#if 0

#include "pcl_methods.hpp"

//show cloud in PCLviwer
/*
 @pram visualization::PCLVisualizer &viewer --PCLviewer for output cloud
 @pram std::string ss --string description of  each output cloud in viewer
 @pram PointCloud<pcl::PointXYZ>::Ptr &cloud -- cloud should be showed in viwer
 */
void D3_view(visualization::PCLVisualizer &viewer, std::vector<PointCloud<PointXYZ>::Ptr> &cloud_vec,PointCloud<PointXYZ>::Ptr &all_cloud){
    
    int i = 0;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb1(all_cloud, 255, 0, 0);
    std::string s = "all_coud";
     //viewer.addPointCloud(all_cloud,rgb1,s);
    for(PointCloud<PointXYZ>::Ptr cloud : cloud_vec){
                //if(i!=0)
                    //break;
        std::string ss = "ss" + std::to_string(i++);
        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> randomColor(cloud);
        viewer.addPointCloud(cloud, randomColor, ss);
        
//         if(i==1){
//             pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(cloud, 0,255,0);
//             viewer.addPointCloud(cloud, rgb, ss);
//         }else{
//             pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(cloud, 0,0,255);
//             viewer.addPointCloud(cloud, rgb, ss);
//         }
        

    }

}

//show cloud in PCLviwer
/*
 @pram visualization::PCLVisualizer &viewer --PCLviewer for output cloud
 @pram std::string ss --string description of  each output cloud in viewer
 @pram std::vector<std::vector<PointCloud<PointXYZ>::Ptr>> &cloud_vec -- lines of each boundary should be showed in viwer
 */
void L3_view(visualization::PCLVisualizer &viewer, std::vector<std::vector<PointCloud<PointXYZ>::Ptr>> &cloud_vec){
    
    int i = 0;
    for(std::vector<PointCloud<PointXYZ>::Ptr> line_vec : cloud_vec){
        for(PointCloud<PointXYZ>::Ptr cloud : line_vec){
        std::string ss = "ss" + std::to_string(i++);
        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> randomColor(cloud);
        viewer.addPointCloud(cloud, randomColor, ss);
     }
    }

}



//get the cloud points Resolution
/*
 @pram const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud -- input cloud
 @pram int k -- neighbor radius
 */
float computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, int k)
{
	double res = 0.0;
	int n_points = 0;
	pcl::KdTreeFLANN<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);
	//?what means size_t
	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (!pcl_isfinite((*cloud)[i].x))
			//pcl_isfinite函数返回一个布尔值，检查某个值是不是正常数值
		{
			continue;
		}
		std::vector<int> indices(k);
		//创建一个动态数组，存储查询点近邻索引
		std::vector<float> sqr_distances(k);
		//存储近邻点对应平方距离
		if (tree.nearestKSearch(i, k, indices, sqr_distances) == k)
		{
			for (int i = 1; i < k; i++)
			{
				res += sqrt(sqr_distances[i]);
				++n_points;
			}
		}
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;
}

//get the boundarys of palanars
/*
 @pram std::vector<PointCloud<PointXYZ>::Ptr> &planars_cloud_vec --input cloud vec
 @pram std::vector<PointCloud<PointXYZ>::Ptr> &boundaries_cloud_vec --output boundarycloud vec
 */
int estimateBorders(std::vector<PointCloud<PointXYZ>::Ptr> &planars_cloud_vec,std::vector<PointCloud<PointXYZ>::Ptr> &boundaries_cloud_vec) 
{ 
    
    
 for(int i = 0; i<planars_cloud_vec.size();i++){
     
    PointCloud<PointXYZ>::Ptr cloud = planars_cloud_vec[i]; 
    for (int i = 0; i < (*cloud).size(); i++){
      cloud->points[i].z = 0;
    }
    
    float reforn = 10*computeCloudResolution(cloud,4);
	pcl::PointCloud<pcl::Boundary> boundaries; //保存边界估计结果
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst; //定义一个进行边界特征估计的对象
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst; //定义一个法线估计的对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); //保存法线估计的结果 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary (new pcl::PointCloud<pcl::PointXYZ>);
	normEst.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(cloud)); 
	normEst.setRadiusSearch(reforn); //设置法线估计的半径
	normEst.compute(*normals); //将法线估计结果保存至normals
	//输出法线的个数
	//std:cout<<"reforn: "<<reforn<<std::endl;
	//std::cerr << "normals: " << normals->size() << std::endl;
 
	boundEst.setInputCloud(cloud); //设置输入的点云
	boundEst.setInputNormals(normals); //设置边界估计的法线，因为边界估计依赖于法线
	boundEst.setRadiusSearch(reforn); //设置边界估计所需要的半径
	boundEst.setAngleThreshold(M_PI/6); //边界估计时的角度阈值
	boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>)); //设置搜索方式KdTree
	boundEst.compute(boundaries); //将边界估计结果保存在boundaries
 
    
    
	//存储估计为边界的点云数据，将边界结果保存为pcl::PointXYZ类型
	for(int j = 0; j < cloud->points.size(); j++) 
	{ 
		
		if(boundaries[j].boundary_point > 0) 
		{ 
            pcl::PointXYZ t_point = cloud->points[j];
            //t_point.z = 0;
			cloud_boundary->push_back(t_point); 
		} 
	} 

	boundaries_cloud_vec.push_back(cloud_boundary);
    		//输出边界点的个数
	std::cerr << "boundarie_points: " <<cloud_boundary->points.size() << std::endl;

 }
	return 0; 
}






//check the plane is top or not
/*
 @pram std::vector<float> &tmp) -- vector includes the ModelCoefficients of planars
 */
int check_plane(std::vector<float> &tmp){
    
    float length = std::sqrt(std::pow(tmp[0],2) + std::pow(tmp[1],2) + std::pow(tmp[2],2));
    
    //(0,0,1) tmp_north -> cos

    float diancheng = tmp[2] > 0? tmp[2] : -tmp[2];
    
    float cos = diancheng / length ;
    
    int res = cos > sqrt(2)/2 ? 1:0;
    return res;
}


//get the cloud average height(z)
/*
 @pram PointCloud<PointXYZ>::Ptr &cloud -- input cloud
 */
float get_planar_height(PointCloud<PointXYZ>::Ptr &cloud){
    int step = 10;
    float height = 0;
    int num = 0;
    for(int i = 0; i< cloud->points.size();i=i+step){
        float temp = cloud->points[i].z < 0? -cloud->points[i].z:cloud->points[i].z;
        height += temp;
        num++;
    }
    
    return height/num;
    
   /* pcl::PointXYZ min_p, max_p;
	pcl::getMinMax3D(*cloud, min_p, max_p);
    float res = (min_p.z + max_p.z)/2;
    res = res>0? res : -res;
    return res;*/
}

//ransac method to get the biggest plane in input cloud
/* 
 @prama: PointCloud<PointXYZ>::Ptr &cloud -- InputCloud
 @prama: PointCloud<PointXYZ>::Ptr &output -- output cloud to record the biggest plane points
 @prama: PointCloud<PointXYZ>::Ptr &cloud_other -- out cloud to record the points except the biggest plane points
 @prama: std::vector<float> &tmp -- the parameters to record the output biggest plane
 @prama: int maxIterrations,float distanceThreshold -- ransac parameters 
 @prama: SacModel model -- ransac seg model type 
 */
void ransac(PointCloud<PointXYZ>::Ptr &cloud,int maxIterrations,float distanceThreshold,PointCloud<PointXYZ>::Ptr &output,PointCloud<PointXYZ>::Ptr &cloud_other,std::vector<float> &tmp,SacModel model){
    
     //
        //pcl::search::KdTree<pcl::PointXYZ>::Ptr search(new pcl::search::KdTree<pcl::PointXYZ>);
        ModelCoefficients::Ptr coefficients(new ModelCoefficients); //
        PointIndices::Ptr inliers(new PointIndices); //
        SACSegmentation<PointXYZ> seg;
        //可选设置
        seg.setOptimizeCoefficients(true);
        //必须设置
        seg.setModelType(model); //
        seg.setMethodType(SAC_RANSAC);      //
        seg.setMaxIterations(maxIterrations);
        seg.setDistanceThreshold(distanceThreshold);
        //seg.setSamplesMaxDist(0.1, search);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);    //
        //search->setInputCloud(cloud);
        if (inliers->indices.size() == 0){
            PCL_ERROR("Could not estimate a planar model for the given dataset.");
            return ;
         }
         
         
        tmp.push_back(coefficients->values[0]);
        tmp.push_back(coefficients->values[1]);
        tmp.push_back(coefficients->values[2]);
        tmp.push_back(coefficients->values[3]);
    
            //提取平面
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.filter(*output);//
        
      

        
        // 移去平面局内点，提取剩余点云
        extract.setNegative(true);
        extract.filter(*cloud_other);
        
//         boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
// 	    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(output, 0, 255, 255);
//         pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb1(cloud_other, 255, 0, 255);
// 	    viewer->addPointCloud(output, rgb, "true cloud");
//         viewer->addPointCloud(cloud_other, rgb1, "false cloud");
//           while (!viewer->wasStopped ())
//           {
//               viewer->spinOnce();
//               
//         }
    
}


//get the building top planars
/*
 @pram PointCloud<PointXYZ>::Ptr &cloud -- input cloud
 @pram std::vector<std::vector<float>> &Coffis -- vector records the ModelCoefficients of planars
 @pram int threshold -- min points num in cloud for planar_seg
 @pram int maxIterrations,float distanceThreshold,int minSizeofPlanar -- max Iterrations,distanceThreshold, of RANSAC 
 */
void forcircle_ransac_seg( PointCloud<PointXYZ>::Ptr &cloud, std::vector<std::vector<float>> &Coffis,int threshold,int maxIterrations,float distanceThreshold,int minSizeofPlanar, std::vector<PointCloud<PointXYZ>::Ptr> &res_vec, SacModel model){
    
    float general_planar_height = model==SACMODEL_PLANE? get_planar_height(cloud) : 0.0;
    
    while (cloud->points.size() >= threshold){
 
        //平面参数
        std::vector<float> tmp;
        //提取平面
        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
        // 移去平面局内点，提取剩余点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_other(new pcl::PointCloud<pcl::PointXYZ>);
        ransac(cloud, maxIterrations, distanceThreshold,output,cloud_other,tmp,model);
        
        if (output->points.size()<minSizeofPlanar){
           return ;
        }
         if((model == SACMODEL_PLANE && check_plane(tmp) && get_planar_height(output) > general_planar_height) || model == SACMODEL_LINE){
        
        //record the planars
        res_vec.push_back(output);
             
        
        //planar coefficients
        Coffis.push_back(tmp);
        std::cerr << "Model coefficients: " << tmp[0] << " " <<tmp[1] << " "
        << tmp[2] << " " << tmp[3] << std::endl;
        
        //planar inliers
        std::cerr << "output point size : " << output->points.size() << std::endl;
        
             
        std::cerr << "other point size : " << cloud_other->points.size() << std::endl;
         }
        cloud = cloud_other;
    }
    
}





//create pcd form tiff data(x-col y-row z-tiff data)
/*@prama: float* paf -- array to store input tiff data
 *@prama: int nImgSizeX -- the length(x direction) of pcd ready to create
 *@prama: int nImgSizeY -- the height(y direction) of pcd ready to create
 *@prama: pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud -- empty pcd to record the points will be created 
 *@param: float col -- the begin col of tiff data which decide the x of points in cloud
 *@prama: float row -- the begin row of tiff data which decide the y of points in cloud
 */
void row_col_oesmZ_2_pcd(float* paf,int nImgSizeX, int nImgSizeY,pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,float col, float row){
    
    // Fill in the cloud data
  cloud->width  = nImgSizeX;
  cloud->height = nImgSizeY;
  cloud->points.resize (cloud->width * cloud->height);

  // Generate the data
  for (std::size_t i = 0; i < cloud->points.size (); ++i)
  {
      int x = col + i%nImgSizeX;
      int y = row + i/nImgSizeX;
      
    cloud->points[i].x = x;
    cloud->points[i].y = y;
    cloud->points[i].z = paf[i];
  }
    
}



//filter the points in cloud with z
void filter_pcl_with_z(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,float z1,float z2){
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  for (int i = 0; i < (*cloud).size(); i++)
  {
    if (cloud->points[i].z < z1 || cloud->points[i].z > z2) // e.g. remove all pts below zAvg
    {
      inliers->indices.push_back(i);
    }
  }
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud);
  
  
}



//get the planes with region grow method 
/*
 @prama:pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud -- input cloud
 @prama:std::vector<PointCloud<PointXYZ>::Ptr> &res_vec -- vector record the planes output
 @prama:general_planar_height -- input cloud's gerneral elevation
 */
int region_grow (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,std::vector<PointCloud<PointXYZ>::Ptr> &res_vec)
{


  pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (50);
  reg.setMaxClusterSize (100000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (10.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (5.0);
  /*reg.setMinClusterSize (50);
  reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (5.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.5);*/

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

 
//  float general_planar_height =  get_planar_height(cloud);

  for(pcl::PointIndices inliers : clusters){
      
        pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices(inliers));
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers_ptr);
        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter(*output);//提取对于索引的点云 内点
        
        res_vec.push_back(output);
        std::cout<<"planar_points: "<<output->points.size()<<std::endl;

    
}

  std:cout<<"groof planes num is(after region grow seg) "<<res_vec.size()<<endl;

  return (0);
}


#endif
