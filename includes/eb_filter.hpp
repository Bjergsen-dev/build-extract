#ifndef EB_FILTER_H
#define EB_FILTER_H
#include "eb_features.hpp"
#include "eb_config.hpp"

void buffer_filter(eb_features_t * eb_futures_ptr, eb_config_t *eb_config_ptr);

void adsorbent_filter(eb_features_t * eb_futures_ptr, eb_config_t *eb_config_ptr);

void eb_update_boundary_pois(eb_features_t * eb_futures_ptr,eb_config_t *eb_config);

void generate_basic_roof(eb_features_t *eb_features_ptr,eb_config_t *eb_config_ptr,eb_roof_t *roof_ptr);

void  generate_roofs(eb_features_t *eb_featur_ptr, eb_roof_t *roof_ptr, eb_config_t *eb_config_ptr);

#endif