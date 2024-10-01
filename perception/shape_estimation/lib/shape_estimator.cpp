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

#include "shape_estimation/shape_estimator.hpp"

#include "shape_estimation/corrector/corrector.hpp"
#include "shape_estimation/filter/filter.hpp"
#include "shape_estimation/model/model.hpp"

#include <iostream>
#include <memory>

using Label = autoware_auto_perception_msgs::msg::ObjectClassification;

ShapeEstimator::ShapeEstimator(std::vector<ShapeParameters> & shapes, bool use_corrector, bool use_filter,  bool use_boost_bbox_optimizer)
: use_corrector_(use_corrector),
  use_filter_(use_filter),
  use_boost_bbox_optimizer_(use_boost_bbox_optimizer),
  shapes_(shapes)
{
}

bool ShapeEstimator::estimateShapeAndPose(
  const uint8_t label, const pcl::PointCloud<pcl::PointXYZ> & cluster,
  const boost::optional<ReferenceYawInfo> & ref_yaw_info,
  const boost::optional<ReferenceShapeSizeInfo> & ref_shape_size_info,
  autoware_auto_perception_msgs::msg::Shape & shape_output, geometry_msgs::msg::Pose & pose_output)
{
  autoware_auto_perception_msgs::msg::Shape shape;
  geometry_msgs::msg::Pose pose;

  //get str name of label
  std::string label_str = labelToString(label);
  ShapeParameters shape_limitation = getShapeLimitation(label_str);

  // estimate shape
  bool reverse_to_unknown = false;
  if (!estimateOriginalShapeAndPose(label, cluster, ref_yaw_info, shape, pose)) {
    reverse_to_unknown = true;
  }

  // rule based filter
  if (use_filter_) {
    if (!applyFilter(label, shape, pose, shape_limitation)) {
      reverse_to_unknown = true;
    }
  }

  // rule based corrector
  if (use_corrector_) {
    bool use_reference_yaw = ref_yaw_info ? true : false;
    if (!applyCorrector(label, use_reference_yaw, ref_shape_size_info, shape, pose)) {
      reverse_to_unknown = true;
    }
  }
  if (reverse_to_unknown) {
    estimateOriginalShapeAndPose(Label::UNKNOWN, cluster, ref_yaw_info, shape, pose);
    shape_output = shape;
    pose_output = pose;
    return false;
  }
  shape_output = shape;
  pose_output = pose;
  return true;
}

bool ShapeEstimator::estimateOriginalShapeAndPose(
  const uint8_t label, const pcl::PointCloud<pcl::PointXYZ> & cluster,
  const boost::optional<ReferenceYawInfo> & ref_yaw_info,
  autoware_auto_perception_msgs::msg::Shape & shape_output, geometry_msgs::msg::Pose & pose_output)
{
  // estimate shape
  std::unique_ptr<ShapeEstimationModelInterface> model_ptr;
  if (
    label == Label::CAR || label == Label::TRUCK || label == Label::BUS ||
    label == Label::TRAILER || label == Label::MOTORCYCLE || label == Label::BICYCLE) {
    model_ptr.reset(new BoundingBoxShapeModel(ref_yaw_info, use_boost_bbox_optimizer_));
  } else if (label == Label::PEDESTRIAN) {
    model_ptr.reset(new CylinderShapeModel());
  } else {
    model_ptr.reset(new ConvexHullShapeModel());
  }

  return model_ptr->estimate(cluster, shape_output, pose_output);
}

bool ShapeEstimator::applyFilter(
  const uint8_t label, const autoware_auto_perception_msgs::msg::Shape & shape,
  const geometry_msgs::msg::Pose & pose, const ShapeParameters & shape_limitation)
{
  std::unique_ptr<ShapeEstimationFilterInterface> filter_ptr;
  if (label == Label::CAR || label == Label::BUS || label == Label::TRUCK || label == Label::TRAILER) {
    filter_ptr.reset(new VehicleFilter);
  } else {
    filter_ptr.reset(new NoFilter);
  }
  return filter_ptr->filter(shape, pose, shape_limitation);
}

bool ShapeEstimator::applyCorrector(
  const uint8_t label, const bool use_reference_yaw,
  const boost::optional<ReferenceShapeSizeInfo> & ref_shape_size_info,
  autoware_auto_perception_msgs::msg::Shape & shape, geometry_msgs::msg::Pose & pose)
{
  std::unique_ptr<ShapeEstimationCorrectorInterface> corrector_ptr;

  if (ref_shape_size_info && use_reference_yaw) {
    corrector_ptr.reset(new ReferenceShapeBasedVehicleCorrector(ref_shape_size_info.get()));
  } else if (label == Label::CAR) {
    corrector_ptr.reset(new CarCorrector(use_reference_yaw));
  } else if (label == Label::BUS) {
    corrector_ptr.reset(new BusCorrector(use_reference_yaw));
  } else if (label == Label::TRUCK) {
    corrector_ptr.reset(new TruckCorrector(use_reference_yaw));
  } else if (label == Label::TRAILER) {
    corrector_ptr.reset(new TrailerCorrector(use_reference_yaw));
  } else if (label == Label::MOTORCYCLE || label == Label::BICYCLE) {
    corrector_ptr.reset(new BicycleCorrector(use_reference_yaw));
  } else {
    corrector_ptr.reset(new NoCorrector);
  }

  return corrector_ptr->correct(shape, pose);
}

// get shape_limitation by name
ShapeParameters ShapeEstimator::getShapeLimitation(const std::string & name)
{
  for (const auto & param : shapes_) {
    if (param.name == name) {
      // debug print
      std::cout << "Shape Limitation in Query: " << param.name << std::endl;
      // std::cout << "min_width: " << param.shape_limitations.min_width << std::endl;
      // std::cout << "max_width: " << param.shape_limitations.max_width << std::endl;
      // std::cout << "min_length: " << param.shape_limitations.min_length << std::endl;
      // std::cout << "max_length: " << param.shape_limitations.max_length << std::endl;
      // std::cout << "min_height: " << param.shape_limitations.min_height << std::endl;
      // std::cout << "max_height: " << param.shape_limitations.max_height << std::endl;
      return param;
    }
  }
  ShapeParameters shape_limitation;
  shape_limitation.name = "unknown";
  shape_limitation.shape_limitations.min_width = 0.0;
  shape_limitation.shape_limitations.max_width = 99.0;
  shape_limitation.shape_limitations.min_length = 0.0;
  shape_limitation.shape_limitations.max_length = 99.0;
  shape_limitation.shape_limitations.min_height = 0.0;
  shape_limitation.shape_limitations.max_height = 99.0;
  return shape_limitation;
}


// convert label(uint8_t) to string
std::string ShapeEstimator::labelToString(const uint8_t label)
{
  if (label == Label::CAR) {
    return "car";
  } else if (label == Label::TRUCK) {
    return "truck";
  } else if (label == Label::BUS) {
    return "bus";
  } else if (label == Label::TRAILER) {
    return "trailer";
  } else if (label == Label::BICYCLE) {
    return "bicycle";
  } else if (label == Label::MOTORCYCLE) {
    return "bicycle";
  } else if (label == Label::PEDESTRIAN) {
    return "pedestrian";
  } else {
    return "unknown";
  }
}
