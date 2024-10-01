// Copyright 2018 Autoware Foundation. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "shape_estimation/filter/vehicle_filter.hpp"
#include <iostream>

bool VehicleFilter::filter(
  const autoware_auto_perception_msgs::msg::Shape & shape,
  [[maybe_unused]] const geometry_msgs::msg::Pose & pose,
  [[maybe_unused]] const ShapeParameters & shape_param)
{
  std::cout << "VehicleFilter::filter with " << shape_param.name << std::endl;
  float min_width = shape_param.shape_limitations.min_width;
  float max_width = shape_param.shape_limitations.max_width;
  float max_length = shape_param.shape_limitations.max_length;
  float min_height = shape_param.shape_limitations.min_height;
  float max_height = shape_param.shape_limitations.max_height;
  return utils::filterVehicleBoundingBox(shape, min_width, max_width, max_length, min_height, max_height);
}
