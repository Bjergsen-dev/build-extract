#include "eb_lidar_detect.hpp"
#include "eb_common_defines.hpp"


//filter the points in cloud with z
void filter_pcl_with_z(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,eb_config_t *eb_config_ptr){
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  for (int i = 0; i < (*cloud).size(); i++)
  {
    if (cloud->points[i].z < eb_config_ptr->pcl_filter_z1 || cloud->points[i].z > eb_config_ptr->pcl_filter_z2) // e.g. remove all pts below zAvg
    {
      inliers->indices.push_back(i);
    }
  }
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud);
  
  
}


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
int estimateBorders(std::vector<PointCloud<PointXYZ>::Ptr> &planars_cloud_vec,std::vector<PointCloud<PointXYZ>::Ptr> &boundaries_cloud_vec,eb_config_t *eb_config_ptr) 
{ 
    
    
 for(int i = 0; i<planars_cloud_vec.size();i++){
     
    PointCloud<PointXYZ>::Ptr cloud = planars_cloud_vec[i]; 
    
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
	boundEst.setAngleThreshold(eb_config_ptr->pcl_estimate_thd); //边界估计时的角度阈值
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
    EB_LOG("[PCL::INFO] estimateBorders points is %ld\n",cloud_boundary->points.size());



 }
	return 0; 
}


int region_grow (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,std::vector<PointCloud<PointXYZ>::Ptr> &res_vec)
{
  EB_LOG("[PCL::INFO] input cloud points num is %ld\n",(*cloud).size());
  pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (10);
  normal_estimator.compute (*normals);

  // pcl::IndicesPtr indices (new std::vector <int>);
  // pcl::PassThrough<pcl::PointXYZ> pass;
  // pass.setInputCloud (cloud);
  // pass.setFilterFieldName ("z");
  // pass.setFilterLimits (0.0, 1.0);
  // pass.filter (*indices);

  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (20);
  reg.setMaxClusterSize (100000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (10);
  reg.setInputCloud (cloud);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (10.0 / 180.0 * M_PI);
  //reg.setCurvatureThreshold (5.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);
  for(pcl::PointIndices inliers : clusters){
      
        pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices(inliers));
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers_ptr);
        pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter(*output);//提取对于索引的点云 内点

        res_vec.push_back(output);
       // std::cout<<"planar_points: "<<output->points.size()<<std::endl;
    
}

  EB_LOG("[PCL::INFO] groof planes num is(after region grow seg) %ld\n",res_vec.size());

  return (0);
}


void get_min_max_z(double *res,const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
  pcl::PointXYZ min;
  pcl::PointXYZ max;
  pcl::getMinMax3D(*cloud,min,max);
  
  res[0] = min.z;
  res[1] = max.z;
  EB_LOG("[PCL::INFO] min z is %lf, max z is %lf\n",res[0],res[1]);
}


