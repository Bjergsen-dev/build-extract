
#pragma once

#include "gdal_methods.hpp"
#include "fetch_building_three_d_points.hpp"

void create_shp(const char* output_path,std::vector<std::vector<z_Point>> z_point_v_vec,int utm_num)
{
	
	const char *pszDriverName="ESRI Shapefile";
	GDALDriver *poDriver;
    GDALAllRegister();
	poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName );
	if (poDriver==NULL)
	{
		std::cout<<pszDriverName<<" driver not available."<<std::endl;
		//exit(1);
	}
	GDALDataset *poDS;
	//poDS=poDriver->CreateDataSource(output_path,NULL);
    poDS = poDriver->Create( output_path, 0, 0, 0, GDT_Unknown, NULL );
	if (poDS==NULL)
	{
		std::cout<<"Creation of shp file failed."<<std::endl;
		//exit(1);
	}
	
	
	OGRSpatialReference oSRS;
    //oSRS.importFromWkt(wkt);
	oSRS.SetProjCS("UTM /WGS84");
	oSRS.SetUTM(utm_num, TRUE);
	oSRS.SetWellKnownGeogCS("WGS84");

	
	OGRLayer *poLayer;
	poLayer=poDS->CreateLayer("building_polygons",&oSRS,wkbPolygon,NULL);
 
	if (poLayer==NULL)
	{
		std::cout<<"Layer creation failed."<<std::endl;
		//exit(1);
	}
	OGRFieldDefn firstField("POLY_ID",OFTInteger);
	OGRFieldDefn secondField("X",OFTReal);
	OGRFieldDefn thirdField("Y",OFTReal);
	firstField.SetWidth(32);
	secondField.SetWidth(32);
	thirdField.SetWidth(32);
	poLayer->CreateField(&firstField);
	poLayer->CreateField(&secondField);
	poLayer->CreateField(&thirdField);
 

	for(int i=0;i<z_point_v_vec.size();i++)
	{
        
        
        OGRFeature *poFeature;
		poFeature=OGRFeature::CreateFeature(poLayer->GetLayerDefn());
		poFeature->SetField("POLY_ID",i+1);
		poFeature->SetField("X",i+1);
		poFeature->SetField("Y",i+1);
        
        OGRLinearRing ring;
        OGRPolygon poly;
        for(int j = 0; j<z_point_v_vec[i].size();j++){
            ring.addPoint(z_point_v_vec[i][j].geo_x,z_point_v_vec[i][j].geo_y);
        }
        
        ring.closeRings();
        poly.addRing(&ring);
		poFeature->SetGeometry(&poly);
		if (poLayer->CreateFeature(poFeature)!=OGRERR_NONE)
		{
			std::cout<<"Failed to create feature in shapefile."<<std::endl;
			//exit(1);
		}
		OGRFeature::DestroyFeature(poFeature);
	}
	GDALClose( poDS );

    std::cout<<"..........Write to shp file complete!"<<std::endl;
}
