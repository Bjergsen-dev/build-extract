#include "eb_config.hpp"
#include "eb_common_defines.hpp"


static void print_file_config(file_config_t * file_config_ptr)
{
    EB_LOG("\n*******************EB FILE CONFIG*********************\n");
    EB_LOG("[EB::INFO] name: %s\n",file_config_ptr->name);
    EB_LOG("[EB::INFO] image_path: %s\n",file_config_ptr->image_path);
    EB_LOG("[EB::INFO] lidar_path: %s\n",file_config_ptr->lidar_path);
    EB_LOG("[EB::INFO] oesm_path: %s\n",file_config_ptr->oesm_path);
    EB_LOG("[EB::INFO] dem_path: %s\n",file_config_ptr->dem_path);
    EB_LOG("[EB::INFO] output_path: %s\n",file_config_ptr->out_path);
    EB_LOG("*******************EB FILE CONFIG*********************\n\n");
}

static void print_eb_config(eb_config_t * eb_config_ptr)
{
    EB_LOG("\n*******************EB PARAM CONFIG*********************\n");
    EB_LOG("[EB::INFO] canny threshold 1: %lf\n",eb_config_ptr->canny_thd_1);
    EB_LOG("[EB::INFO] canny threshold 2: %lf\n",eb_config_ptr->canny_thd_1);
    EB_LOG("[EB::INFO] hough threshold 1: %lf\n",eb_config_ptr->hough_thd_1);
    EB_LOG("[EB::INFO] hough threshold 2: %lf\n",eb_config_ptr->hough_thd_2);
    EB_LOG("[EB::INFO] hough threshold 3: %d\n",eb_config_ptr->hough_thd_3);
    EB_LOG("[EB::INFO] hough threshold 4: %lf\n",eb_config_ptr->hough_thd_4);
    EB_LOG("[EB::INFO] hough threshold 5: %lf\n",eb_config_ptr->hough_thd_5);
    EB_LOG("[EB::INFO] pcl filter z1: %lf\n",eb_config_ptr->pcl_filter_z1);
    EB_LOG("[EB::INFO] pcl filter z2: %lf\n",eb_config_ptr->pcl_filter_z2);
    EB_LOG("[EB::INFO] pcl estimate threshold: %lf\n",eb_config_ptr->pcl_estimate_thd);
    EB_LOG("[EB::INFO] transform x: %lf\n",eb_config_ptr->transform_x);
    EB_LOG("[EB::INFO] transform y: %lf\n",eb_config_ptr->transform_y);
    EB_LOG("[EB::INFO] transform z: %lf\n",eb_config_ptr->transform_z);
    EB_LOG("[EB::INFO] buffer_matrix_size: %d\n",eb_config_ptr->buffer_matrix_size);
    EB_LOG("[EB::INFO] buffer_filter_f: %f\n",eb_config_ptr->buffer_filter_f);
    EB_LOG("[EB::INFO] min_adsorb_num: %d\n",eb_config_ptr->min_adsorb_num);
    EB_LOG("[EB::INFO] min_adsorb_dis: %lf\n",eb_config_ptr->min_adsorb_dis);
    EB_LOG("[EB::INFO] min_direct_trd: %lf\n",eb_config_ptr->min_direct_trd);
    EB_LOG("[EB::INFO] ground_z: %lf\n",eb_config_ptr->ground_z);
    EB_LOG("[EB::INFO] rebud_density_x: %d\n",eb_config_ptr->rebud_density_x);
    EB_LOG("[EB::INFO] rebud_density_y: %d\n",eb_config_ptr->rebud_density_y);
    EB_LOG("*******************EB PARAM CONFIG*********************\n\n");

    print_file_config(&eb_config_ptr->file_config);
}

void read_eb_config(eb_config_t * eb_config_ptr, const char * file_path)
{
    FILE *fp;
    fp = fopen(file_path, "rb");
    if(fp != NULL)
    {
        printf("open %s success!\n",file_path);
    }
    else
    {
        printf("open %s failed!\n",file_path);
        return;
    }

    fscanf(fp,"task: build_extract\n");
    fscanf(fp,"name: %s\n",eb_config_ptr->file_config.name);
    fscanf(fp,"image_path: %s\n",eb_config_ptr->file_config.image_path);
    fscanf(fp,"lidar_path: %s\n",eb_config_ptr->file_config.lidar_path);
    fscanf(fp,"oesm_path: %s\n",eb_config_ptr->file_config.oesm_path);
    fscanf(fp,"dem_path: %s\n",eb_config_ptr->file_config.dem_path);
    fscanf(fp,"output_path: %s\n",eb_config_ptr->file_config.out_path);

    fscanf(fp,"\n");
    fscanf(fp,"canny_thd_1: %lf\n",&eb_config_ptr->canny_thd_1);
    fscanf(fp,"canny_thd_2: %lf\n",&eb_config_ptr->canny_thd_2);
    fscanf(fp,"hough_thd_1: %lf\n",&eb_config_ptr->hough_thd_1);
    fscanf(fp,"hough_thd_2: %lf\n",&eb_config_ptr->hough_thd_2);
    fscanf(fp,"hough_thd_3: %d\n",&eb_config_ptr->hough_thd_3);
    fscanf(fp,"hough_thd_4: %lf\n",&eb_config_ptr->hough_thd_4);
    fscanf(fp,"hough_thd_5: %lf\n",&eb_config_ptr->hough_thd_5);
    fscanf(fp,"pcl_filter_z1: %lf\n",&eb_config_ptr->pcl_filter_z1);
    fscanf(fp,"pcl_filter_z2: %lf\n",&eb_config_ptr->pcl_filter_z2);
    fscanf(fp,"pcl_estimate_thd: %lf\n",&eb_config_ptr->pcl_estimate_thd);
    fscanf(fp,"transform_x: %lf\n",&eb_config_ptr->transform_x);
    fscanf(fp,"transform_y: %lf\n",&eb_config_ptr->transform_y);
    fscanf(fp,"transform_z: %lf\n",&eb_config_ptr->transform_z);
    fscanf(fp,"buffer_matrix_size: %d\n",&eb_config_ptr->buffer_matrix_size);
    fscanf(fp,"buffer_filter_f: %f\n",&eb_config_ptr->buffer_filter_f);
    fscanf(fp,"min_adsorb_num: %d\n",&eb_config_ptr->min_adsorb_num);
    fscanf(fp,"min_adsorb_dis: %lf\n",&eb_config_ptr->min_adsorb_dis);
    fscanf(fp,"min_direct_trd: %lf\n",&eb_config_ptr->min_direct_trd);
    fscanf(fp,"ground_z: %lf\n",&eb_config_ptr->ground_z);
    fscanf(fp,"rebud_density_x: %d\n",&eb_config_ptr->rebud_density_x);
    fscanf(fp,"rebud_density_y: %d\n",&eb_config_ptr->rebud_density_y);
 
    fclose(fp);

    print_eb_config(eb_config_ptr);
}