#include "gdal_methods.hpp"


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
        cout<<"fail in open files!!!"<<endl;

    }
    
       //获取坐标变换系数
    //double trans[6];
    CPLErr aaa=poDataset->GetGeoTransform(trans);
    
    delete poDataset;

    
}

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
float get_geoX_geoY_frm_row_column(double*trans, double x,double y,int type){
    
    float geo_X = trans[0] + (x+0.5) * trans[1] + (y+0.5) * trans[2];
    float geo_Y = trans[3] + (x+0.5) * trans[4] + (y+0.5) * trans[5];
    
     if(type == 2){
        
        return geo_X;
    }
    
    return geo_Y;
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

