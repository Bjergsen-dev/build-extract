#ifndef EB_FILTER_H
#define EB_FILTER_H
#include "eb_features.hpp"
#include "eb_config.hpp"

void buffer_filter(eb_features_t * eb_futures_ptr, eb_config_t *eb_config_ptr);

void adsorbent_filter(eb_features_t * eb_futures_ptr, eb_config_t *eb_config_ptr);

#endif