/*
 * StepFilter.cpp
 *
 *  Created on: Mar 11, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "filters/FuseFilter.hpp"
#include <pluginlib/class_list_macros.h>
#include <algorithm>

// Grid Map
#include <grid_map_ros/grid_map_ros.hpp>

using namespace grid_map;

namespace filters {

template<typename T>
FuseFilter<T>::FuseFilter()
    : max_allowed_step_depth_(0.15),
      sample_distance_cells_(10),
      //firstWindowRadius_(0.08),
      //secondWindowRadius_(0.08),
      nCellCritical_(3),
      type_("traversability")
{

}

template<typename T>
FuseFilter<T>::~FuseFilter()
{

}

template<typename T>
bool FuseFilter<T>::configure()
{
  if (!FilterBase<T>::getParam(std::string("max_allowed_step_depth"), max_allowed_step_depth_)) {
    ROS_ERROR("Step filter did not find param max_allowed_step_depth.");
    return false;
  }

  if (max_allowed_step_depth_ < 0.0) {
    ROS_ERROR("Max allowed step depth must be greater than zero.");
    return false;
  }

  ROS_DEBUG("Max allowed step depth = %f.", max_allowed_step_depth_);

  if (!FilterBase<T>::getParam(std::string("map_type"), type_)) {
    ROS_ERROR("Step filter did not find param map_type.");
    return false;
  }

  ROS_DEBUG("Negative Step map type = %s.", type_.c_str());

  return true;
}

template<typename T>
bool FuseFilter<T>::update(const T& mapIn, T& mapOut)
{


  mapOut = mapIn;
  mapOut.add(type_);


  const grid_map::Matrix& traversability_slope_data = mapOut["traversability_slope"];
  const grid_map::Matrix& traversability_step_data = mapOut["traversability_step"];
  const grid_map::Matrix& traversability_negative_step = mapOut["traversability_negative_step"];
  //const grid_map::Matrix& traversability_roughness_data = mapOut["traversability_roughness"];

  grid_map::Matrix& traversability_data = mapOut[type_];

  // First iteration through the elevation map.
  for (GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {

    //grid_map::wrapIndexToRange(cindex, mapOut.getSize());

    //const float curr_elevation_data = elevation_data(cindex.x(), cindex.y());
    //std::cout << " el: " << curr_elevation_data << "\n";



    if (!mapOut.isValid(*iterator, "elevation"))
      continue;

    grid_map::Index curr_index(*iterator);

    traversability_data(curr_index.x(), curr_index.y()) = std::min({
                                                            traversability_slope_data(curr_index.x(), curr_index.y()),
                                                            traversability_step_data(curr_index.x(), curr_index.y()),
                                                            traversability_negative_step(curr_index.x(), curr_index.y()),
                                                            //traversability_roughness_data(curr_index.x(), curr_index.y()),
                                                           });
  }
  return true;
}

} /* namespace */

PLUGINLIB_REGISTER_CLASS(FuseFilter, filters::FuseFilter<grid_map::GridMap>,
                         filters::FilterBase<grid_map::GridMap>)
