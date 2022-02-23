#include "eb_transform.hpp"
#include "opencv2/opencv.hpp"


void shp_reader(char * shp_file,std::vector<std::vector<cv::Vec2f>> &polygons_vec){
    
    //主要修改
    GDALAllRegister();
    
    GDALDataset* poDS;
    
    poDS = (GDALDataset*) GDALOpenEx( shp_file, GDAL_OF_VECTOR, NULL, NULL, NULL );
    if( poDS == NULL )
    {
        printf( "Open failed.\n" );
        exit( 1 );
    }
    
    //后面与1.x版本一致
    OGRLayer  *poLayer;
   // poLayer = poDS->GetLayerByName("building_polygons");
    poLayer = poDS->GetLayer(0);


    OGRFeature *poFeature;
    poLayer->ResetReading();
    while( (poFeature = poLayer->GetNextFeature()) != NULL )
    {
 
        OGRGeometry *poGeometry;
        poGeometry = poFeature->GetGeometryRef();
        if( poGeometry != NULL
                && wkbFlatten(poGeometry->getGeometryType()) == wkbPolygon )
        {
            OGRPolygon *poPolygon = (OGRPolygon *) poGeometry;
			OGRLinearRing *pOGRLinearRing = poPolygon->getExteriorRing();
            int num = pOGRLinearRing->getNumPoints();
            
            OGRPoint point;
            std::vector<cv::Vec2f> temp_vec;
            for(int j = 0; j < num-1;j++){
                pOGRLinearRing->getPoint(j,&point);
                cv::Vec2f vec(point.getX(),point.getY());
                temp_vec.push_back(vec);
            }
            
            polygons_vec.push_back(temp_vec);

        }
        else
        {
            printf( "no polygon geometry\n" );
        }
        OGRFeature::DestroyFeature( poFeature );
    }
    GDALClose( poDS );
    std::cout<<"..........Read shp file complete"<<std::endl;
}



void open_Gdal(GDALDataset *poDataset,char *file_path_name)
{
    
    GDALAllRegister();  //注册所有的驱动
    poDataset = (GDALDataset *) GDALOpen(file_path_name, GA_ReadOnly );
    if( poDataset == NULL )
    {
        EB_LOG("GDAL::ERROR :%s open failed!\n",file_path_name);

    }
}
//get trans of tiff file
/*
 @prama: const char * file_path_name -- file path to load the tif
 @prama: double* trans -- array to record the trans
 */
void getTrans_of_TiffFile(const char * file_path_name, double* trans){
    GDALDataset *poDataset;   //GDAL数据集
    GDALAllRegister();  //注册所有的驱动
    poDataset = (GDALDataset *) GDALOpen(file_path_name, GA_ReadOnly );
    if( poDataset == NULL )
    {
        EB_LOG("GDAL::ERROR :%s open failed!\n",file_path_name);

    }
    
       //获取坐标变换系数
    //double trans[6];
    CPLErr aaa=poDataset->GetGeoTransform(trans);
    
    delete poDataset;

    
}

void save_Trans_to_File(eb_config_t * config_ptr)
{
    char trans_path[256];
    snprintf(trans_path,strlen(config_ptr->file_config.out_path)+1,config_ptr->file_config.out_path);
    snprintf(trans_path+strlen(trans_path),11,"/trans.txt");

    FILE *fp = fopen(trans_path,"w");
    for(int i = 0; i < 6; i++)
    {
        fprintf(fp,"%lf\n",config_ptr->trans[i]);
    }

    fclose(fp);
    EB_LOG("save trans to %s\n",trans_path);

}

#ifdef MID_RESULT
void trans_dx_dy_to_all(cv::Point poi,eb_config_t *config_ptr,double *res)
{
    double geo_x = 0.;
    double geo_y = 0.;
    get_geoX_geoY_frm_row_column(config_ptr->trans,poi.x,poi.y,&geo_x,&geo_y);
    float dx = get_row_column_frm_geoX_geoY(config_ptr->all_trans,geo_x,geo_y,1);
    float dy = get_row_column_frm_geoX_geoY(config_ptr->all_trans,geo_x,geo_y,2);

    res[0] = dx;
    res[1] = dy;
}
#endif


//get the row,column subnum in tiff file from spacial loacation geoX geoY
/*
 @prama: double *trans -- array record thr trans data
 @prama: double geoX -- input the spacial location_X
 @prama: double geoY -- input the spacial location_Y
 @prama: int type -- decide the return type 2--row other--column
 return : the row or column for input spacial XY
 */
float get_row_column_frm_geoX_geoY(double *trans,double geoX, double geoY,int type){
    
    double dTemp = trans[1] * trans[5] - trans[2] * trans[4];
    
    float dCol = (trans[5] * (geoX - trans[0]) - trans[2] * (geoY - trans[3])) / dTemp + 0.5;//移到像素中心
	float dRow = (trans[1] * (geoY - trans[3]) - trans[4] * (geoX - trans[0])) / dTemp + 0.5;
    
    if(type == 2){
        
        return dRow;
    }
    
    return dCol;
}



//get the geoX,geoY subnum in tiff file from row column
/*
 @prama: double *trans -- array record thr trans data
 @prama: double x -- input the clomn
 @prama: double y -- input the row
 @prama: int type -- decide the return type 2--geo_X other--geo_Y
 return : the spacial XY for input row or column
 */
void get_geoX_geoY_frm_row_column(double*trans, double x,double y,double *geo_X, double *geo_Y){
    
     *geo_X = trans[0] + (x+0.5) * trans[1] + (y+0.5) * trans[2];
     *geo_Y = trans[3] + (x+0.5) * trans[4] + (y+0.5) * trans[5];

}

//get the elevation of raster tif in paticular row_column
/*
 @prama: double *trans -- array record thr trans data
 @prama: double x -- input the clomn
 @prama: double y -- input the row
 return : the elevation for input row or column
 */
float get_elevation_frm_row_column(GDALDataset* poDataset, float* inBuf,int x, int y){
    
    GDALRasterBand* pInRasterBand = poDataset->GetRasterBand(1);
	 CPLErr err;
	 err = pInRasterBand->RasterIO(GF_Read, x, y, 1, 1, inBuf, 1, 1, GDT_Float32, 0, 0);
     if (err == CE_Failure)
	{
		std::cout<<"读取输入数据失败！"<<std::endl;
		return -1;
	}
	
	
	return *inBuf;
	
}

float get_elevation_frm_geo_XY(GDALDataset* poDataset,double geo_x, double geo_y, double *trans){
    
    float inBuf;

    double dTemp = trans[1] * trans[5] - trans[2] * trans[4];
    
    float dCol = (trans[5] * (geo_x - trans[0]) - trans[2] * (geo_y - trans[3])) / dTemp + 0.5;//移到像素中心
	float dRow = (trans[1] * (geo_y - trans[3]) - trans[4] * (geo_x - trans[0])) / dTemp + 0.5;

    
    if(poDataset == NULL)
    {
        EB_LOG("[EB_ERROR:: ] open gdal file failed!\n");
    }
    GDALRasterBand* pInRasterBand = poDataset->GetRasterBand(1);
    EB_LOG("[EB_DEBUG:: ]%f %f\n",dCol,dRow);
	 CPLErr err;
	 err = pInRasterBand->RasterIO(GF_Read, dCol, dRow, 1, 1, &inBuf, 1, 1, GDT_Float32, 0, 0);
     if (err == CE_Failure)
	{
		std::cout<<"读取输入数据失败！"<<std::endl;
		return -1;
	}
	
	
	return inBuf;
	
}



//pcd to mat chansform
/*
@prama: vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &boundary_cloud_vec -- the vector record bundary points in pcd
@prama: double *trans -- localization transform prameters of tiff file (used in gdal)
@prama: float translation_x,float translation_y -- the translation of x,y between pcd and tif file
 */
void pcd_to_mat(pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_cloud,double *trans,eb_config_t *eb_config_ptr,eb_points_t  *boundary_pois){
    
        boundary_pois->points = (eb_point_t *)malloc(sizeof(eb_point_t)*boundary_cloud->points.size());
        boundary_pois->point_size = boundary_cloud->points.size();
        for(int i = 0 ; i < boundary_cloud->points.size(); i++){
            boundary_pois->points[i].point_x = boundary_cloud->points[i].x + eb_config_ptr->transform_x;
            boundary_pois->points[i].point_y = boundary_cloud->points[i].y + eb_config_ptr->transform_y;
            boundary_pois->points[i].point_z = boundary_cloud->points[i].z + eb_config_ptr->transform_z;

            boundary_pois->points[i].dx = get_row_column_frm_geoX_geoY(trans,boundary_pois->points[i].point_x,boundary_pois->points[i].point_y,1);
            boundary_pois->points[i].dy = get_row_column_frm_geoX_geoY(trans,boundary_pois->points[i].point_x,boundary_pois->points[i].point_y,2);

            boundary_pois->points[i].is_delaunay = 0;
            boundary_pois->points[i].is_adsorbed = 0;
            
        }
    
    
}

#if 0
void lidar_planar_to_image(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,cv::Mat &image,double *trans,double transform_x,double transform_y)
{
  for(int i = 0; i < cloud->points.size(); i++)
  {
    float dx = get_row_column_frm_geoX_geoY(trans,cloud->points[i].x + transform_x ,cloud->points[i].y + transform_y,1);
    float dy = get_row_column_frm_geoX_geoY(trans,cloud->points[i].x + transform_x,cloud->points[i].y + transform_y,2);

    cv::circle(image,
                cv::Point(dx,dy),
                1,
                cv::Scalar(0),
                1,
                CV_AA);
  }
}
#endif