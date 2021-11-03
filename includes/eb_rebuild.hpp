#ifndef EB_REBUILD_H
#define EB_REBUILD_H
#include <iostream>
#include "eb_features.hpp"
#include "eb_config.hpp"

typedef struct rebuilt_poi
{
    double poi_x;
    double poi_y;
    double poi_z;

}rebuild_poi_t;

void roof_rebuild(eb_roof_t *roof,eb_config_t *eb_config_ptr, cv::Mat &roof_lidar_image,double *cloud_z);

#endif