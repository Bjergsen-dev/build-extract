/**************************************************************************

Copyright:Best_Intelligence in LiesMars

Author: ZZC_Bjergsen

Date:2020-08-18

Description:Provide  functions  of GDAL

**************************************************************************/
#pragma once

#include<iostream>
#include"gdal_priv.h"
#include "ogrsf_frmts.h"
using namespace std;

//get the data of some part of tiff file
/*
 @prama: const char * file_path_name -- input tiff file path
 @prama: int row -- the row of begin pixel(left top of data area)
 @prama: int column -- the column of begin pixel(left top of data area)
 @prama: int nImgSizeX -- the length (column direction) of data area
 @prama: int nImgSizeY -- the height (row direction) of data area
 @prama: BYTE* pafScanblock1 -- array to record the data
 @prama: int bandnum -- which band of tiff to fetch data
 */
template<class BYTE>
bool get_data_in_tiff(const char * file_path_name, int row, int column,int nImgSizeX,int nImgSizeY,BYTE* pafScanblock1,int bandnum){
    GDALDataset *poDataset;   //GDAL数据集
    GDALAllRegister();  //注册所有的驱动
    poDataset = (GDALDataset *) GDALOpen(file_path_name, GA_ReadOnly );
    if( poDataset == NULL )
    {
        cout<<"fail in open files!!!"<<endl;

    }

    //获取图像波段
    GDALRasterBand *poBand1;
    poBand1=poDataset->GetRasterBand(bandnum);


    //读取图像高程数据
    //pafScanblock1 = (BYTE *) CPLMalloc(sizeof(BYTE)*(nImgSizeX)*(nImgSizeY));
    CPLErr aaa = poBand1->RasterIO( GF_Read, column, row,nImgSizeX,nImgSizeY,pafScanblock1,nImgSizeX,nImgSizeY,GDALDataType(poBand1->GetRasterDataType()),0, 0 );
    delete poDataset;
    
    if(aaa == CE_Failure){
        
        std::cout<<"read fail!"<<endl;
        return false;
    }
    
    return true;

}

//get trans of tiff file
/*
 @prama: const char * file_path_name -- file path to load the tif
 @prama: double* trans -- array to record the trans
 */
void getTrans_of_TiffFile(const char * file_path_name, double* trans);

//get the row,column subnum in tiff file from spacial loacation geoX geoY
/*
 @prama: double *trans -- array record thr trans data
 @prama: double geoX -- input the spacial location_X
 @prama: double geoY -- input the spacial location_Y
 @prama: int type -- decide the return type 2--row other--column
 return : the row or column for input spacial XY
 */
float get_row_column_frm_geoX_geoY(double *trans,double geoX, double geoY,int type);

//get the geoX,geoY subnum in tiff file from row column
/*
 @prama: double *trans -- array record thr trans data
 @prama: double x -- input the clomn
 @prama: double y -- input the row
 @prama: int type -- decide the return type 2--geo_X other--geo_Y
 return : the spacial XY for input row or column
 */
float get_geoX_geoY_frm_row_column(double*trans, double x,double y,int type);

//get the elevation of raster tif in paticular row_column
/*
 @prama: double *trans -- array record thr trans data
 @prama: double x -- input the clomn
 @prama: double y -- input the row
 return : the elevation for input row or column
 */
float get_elevation_frm_row_column(GDALDataset* poDataset, float* inBuf,int x, int y);
