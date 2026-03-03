// Copyright 2020 TIER IV, Inc.
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

#include "autoware/image_projection_based_fusion/roi_cluster_fusion/node.hpp"

#include <autoware/image_projection_based_fusion/utils/geometry.hpp>
#include <autoware/image_projection_based_fusion/utils/utils.hpp>
#include <autoware_utils/system/time_keeper.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <cstring>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif

namespace autoware::image_projection_based_fusion
{
using autoware_utils::ScopedTimeTrack;

RoiClusterFusionNode::RoiClusterFusionNode(const rclcpp::NodeOptions & options)
: FusionNode<ClusterMsgType, RoiMsgType, ClusterMsgType>("roi_cluster_fusion", options)
{
  strict_iou_match_mode_ = declare_parameter<std::string>("strict_iou_match_mode");
  rough_iou_match_mode_ = declare_parameter<std::string>("rough_iou_match_mode");
  use_cluster_semantic_type_ = declare_parameter<bool>("use_cluster_semantic_type");
  only_allow_inside_cluster_ = declare_parameter<bool>("only_allow_inside_cluster");
  roi_scale_factor_ = declare_parameter<double>("roi_scale_factor");
  iou_threshold_ = declare_parameter<double>("iou_threshold");
  unknown_iou_threshold_ = declare_parameter<double>("unknown_iou_threshold");
  remove_unknown_ = declare_parameter<bool>("remove_unknown");
  fusion_distance_ = declare_parameter<double>("fusion_distance");
  strict_iou_fusion_distance_ = declare_parameter<double>("strict_iou_fusion_distance");
  enable_roi_cluster_splitting_ = declare_parameter<bool>("enable_roi_cluster_splitting");
  split_min_point_num_ = declare_parameter<int>("split_min_point_num");

  // publisher
  pub_ptr_ = this->create_publisher<ClusterMsgType>("output", rclcpp::QoS{1});
}

void RoiClusterFusionNode::preprocess(ClusterMsgType & output_cluster_msg)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  // reset cluster semantic type
  if (!use_cluster_semantic_type_) {
    for (auto & feature_object : output_cluster_msg.feature_objects) {
      feature_object.object.classification.front().label =
        autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
      feature_object.object.existence_probability = 0.0;
    }
  }
}

void RoiClusterFusionNode::fuse_on_single_image(
  const ClusterMsgType & input_cluster_msg, const Det2dStatus<RoiMsgType> & det2d_status,
  const RoiMsgType & input_rois_msg, ClusterMsgType & output_cluster_msg)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  const sensor_msgs::msg::CameraInfo & camera_info =
    det2d_status.camera_projector_ptr->getCameraInfo();

  // get transform from cluster frame id to camera optical frame id
  geometry_msgs::msg::TransformStamped transform_stamped;
  {
    const auto transform_stamped_optional = getTransformStamped(
      tf_buffer_, /*target*/ input_rois_msg.header.frame_id,
      /*source*/ input_cluster_msg.header.frame_id, input_rois_msg.header.stamp);
    if (!transform_stamped_optional) {
      RCLCPP_WARN_STREAM(
        get_logger(), "Failed to get transform from " << input_cluster_msg.header.frame_id << " to "
                                                      << input_rois_msg.header.frame_id);
      return;
    }
    transform_stamped = transform_stamped_optional.value();
  }

  std::map<std::size_t, RegionOfInterest> m_cluster_roi;
  // Map: cluster index -> vector of (raw_point_index, projected_2d_point)
  std::map<std::size_t, std::vector<std::pair<std::size_t, Eigen::Vector2d>>>
    m_cluster_projected_points;

  std::vector<sensor_msgs::msg::RegionOfInterest> debug_image_rois;
  std::vector<Eigen::Vector2d> debug_obstacle_points;
  std::vector<sensor_msgs::msg::RegionOfInterest> debug_obstacle_rois;
  std::vector<double> debug_max_iou_for_image_rois;

  // --- Pass 1: Project cluster points to 2D and compute bounding ROIs ---
  for (std::size_t i = 0; i < input_cluster_msg.feature_objects.size(); ++i) {
    if (input_cluster_msg.feature_objects.at(i).feature.cluster.data.empty()) {
      continue;
    }

    if (is_far_enough(input_cluster_msg.feature_objects.at(i), fusion_distance_)) {
      continue;
    }

    // filter point out of scope
    if (debugger_ && out_of_scope(input_cluster_msg.feature_objects.at(i))) {
      continue;
    }

    sensor_msgs::msg::PointCloud2 transformed_cluster;
    tf2::doTransform(
      input_cluster_msg.feature_objects.at(i).feature.cluster, transformed_cluster,
      transform_stamped);

    int min_x(camera_info.width), min_y(camera_info.height), max_x(0), max_y(0);
    std::vector<Eigen::Vector2d> projected_points;
    std::vector<std::pair<std::size_t, Eigen::Vector2d>> indexed_projected_points;
    projected_points.reserve(transformed_cluster.data.size());

    std::size_t raw_pt_idx = 0;
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_cluster, "x"),
         iter_y(transformed_cluster, "y"), iter_z(transformed_cluster, "z");
         iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++raw_pt_idx) {
      if (*iter_z <= 0.0) {
        continue;
      }

      Eigen::Vector2d projected_point;
      if (det2d_status.camera_projector_ptr->calcImageProjectedPoint(
            cv::Point3d(*iter_x, *iter_y, *iter_z), projected_point)) {
        const int px = static_cast<int>(projected_point.x());
        const int py = static_cast<int>(projected_point.y());

        min_x = std::min(px, min_x);
        min_y = std::min(py, min_y);
        max_x = std::max(px, max_x);
        max_y = std::max(py, max_y);

        projected_points.push_back(projected_point);
        indexed_projected_points.emplace_back(raw_pt_idx, projected_point);
        if (debugger_) debug_obstacle_points.push_back(projected_point);
      }
    }
    if (projected_points.empty()) {
      continue;
    }

    sensor_msgs::msg::RegionOfInterest roi;
    roi.x_offset = min_x;
    roi.y_offset = min_y;
    roi.width = max_x - min_x;
    roi.height = max_y - min_y;
    m_cluster_roi.insert(std::make_pair(i, roi));
    m_cluster_projected_points[i] = std::move(indexed_projected_points);
    if (debugger_) debug_obstacle_rois.push_back(roi);
  }

  // --- Pass 2: Match ROIs to clusters, accumulate matches ---
  // Map: cluster index -> vector of RoiMatch
  std::map<std::size_t, std::vector<RoiMatch>> cluster_roi_matches;

  for (const auto & feature_obj : input_rois_msg.feature_objects) {
    int index = -1;
    bool associated = false;
    double max_iou = 0.0;
    const bool is_roi_label_known =
      feature_obj.object.classification.front().label != ObjectClassification::UNKNOWN;
    for (const auto & cluster_map : m_cluster_roi) {
      double iou(0.0);
      bool use_rough_iou_match = is_far_enough(
        input_cluster_msg.feature_objects.at(cluster_map.first), strict_iou_fusion_distance_);
      auto image_roi = feature_obj.feature.roi;
      auto cluster_roi = cluster_map.second;
      sanitizeROI(image_roi, camera_info.width, camera_info.height);
      sanitizeROI(cluster_roi, camera_info.width, camera_info.height);
      if (use_rough_iou_match || (!is_roi_label_known)) {
        iou = cal_iou_by_mode(cluster_roi, image_roi, rough_iou_match_mode_);
      } else {
        iou = cal_iou_by_mode(cluster_roi, image_roi, strict_iou_match_mode_);
      }

      const bool passed_inside_cluster_gate =
        only_allow_inside_cluster_ ? is_inside(image_roi, cluster_roi, roi_scale_factor_) : true;
      if (max_iou < iou && passed_inside_cluster_gate) {
        index = cluster_map.first;
        max_iou = iou;
        associated = true;
      }
    }

    if (!associated) {
      continue;
    }

    const bool is_roi_iou_over_threshold =
      (is_roi_label_known && iou_threshold_ < max_iou) ||
      (!is_roi_label_known && unknown_iou_threshold_ < max_iou);

    if (is_roi_iou_over_threshold) {
      RoiMatch match;
      match.roi = feature_obj.feature.roi;
      match.classification = feature_obj.object.classification;
      match.existence_prob = feature_obj.object.existence_probability;
      match.iou = max_iou;
      cluster_roi_matches[index].push_back(match);
    }

    if (debugger_) debug_image_rois.push_back(feature_obj.feature.roi);
    if (debugger_) debug_max_iou_for_image_rois.push_back(max_iou);
  }

  // --- Pass 3: Apply matches — single label or split ---
  // Collect indices that need to be replaced by split results
  std::vector<std::size_t> indices_to_remove;
  std::vector<ClusterObjType> new_objects_to_add;

  for (auto & [cluster_idx, matches] : cluster_roi_matches) {
    if (output_cluster_msg.feature_objects.empty()) {
      break;
    }

    if (matches.size() == 1) {
      // Single match: apply label directly (original behavior)
      auto & fused_object = output_cluster_msg.feature_objects.at(cluster_idx).object;
      const auto & match = matches[0];
      if (fused_object.existence_probability <= match.existence_prob) {
        fused_object.classification = match.classification;
        fused_object.existence_probability =
          std::clamp(match.existence_prob, min_roi_existence_prob_, 1.0f);
      }
    } else if (matches.size() >= 2 && enable_roi_cluster_splitting_) {
      // Multiple matches with splitting enabled
      auto split_results = splitClusterByRois(
        output_cluster_msg.feature_objects.at(cluster_idx),
        m_cluster_projected_points[cluster_idx], matches);

      if (split_results.size() >= 2) {
        indices_to_remove.push_back(cluster_idx);
        for (auto & [sub_obj, roi_match] : split_results) {
          sub_obj.object.classification = roi_match.classification;
          sub_obj.object.existence_probability =
            std::clamp(roi_match.existence_prob, min_roi_existence_prob_, 1.0f);
          new_objects_to_add.push_back(std::move(sub_obj));
        }
      } else {
        // Fallback: splitting produced <2 clusters, apply best match
        auto & fused_object = output_cluster_msg.feature_objects.at(cluster_idx).object;
        const auto & best =
          *std::max_element(matches.begin(), matches.end(), [](const auto & a, const auto & b) {
            return a.existence_prob < b.existence_prob;
          });
        fused_object.classification = best.classification;
        fused_object.existence_probability =
          std::clamp(best.existence_prob, min_roi_existence_prob_, 1.0f);
      }
    } else {
      // Multiple matches but splitting disabled: apply best match
      auto & fused_object = output_cluster_msg.feature_objects.at(cluster_idx).object;
      const auto & best =
        *std::max_element(matches.begin(), matches.end(), [](const auto & a, const auto & b) {
          return a.existence_prob < b.existence_prob;
        });
      fused_object.classification = best.classification;
      fused_object.existence_probability =
        std::clamp(best.existence_prob, min_roi_existence_prob_, 1.0f);
    }
  }

  // Remove original clusters that were split (in reverse order to preserve indices)
  std::sort(indices_to_remove.rbegin(), indices_to_remove.rend());
  for (auto idx : indices_to_remove) {
    output_cluster_msg.feature_objects.erase(output_cluster_msg.feature_objects.begin() + idx);
  }
  // Add new split sub-clusters
  for (auto & obj : new_objects_to_add) {
    output_cluster_msg.feature_objects.push_back(std::move(obj));
  }

  // note: debug objects are safely cleared in fusion_node.cpp
  // TODO(badai-nguyen): revise the shared debugger_ usage
  if (debugger_) {
    debugger_->image_rois_ = debug_image_rois;
    debugger_->obstacle_rois_ = debug_obstacle_rois;
    debugger_->obstacle_points_ = debug_obstacle_points;
    debugger_->max_iou_for_image_rois_ = debug_max_iou_for_image_rois;
    debugger_->publishImage(det2d_status.id, input_rois_msg.header.stamp);
  }
}

std::vector<std::pair<ClusterObjType, RoiClusterFusionNode::RoiMatch>>
RoiClusterFusionNode::splitClusterByRois(
  const ClusterObjType & cluster_obj,
  const std::vector<std::pair<std::size_t, Eigen::Vector2d>> & projected_points,
  const std::vector<RoiMatch> & roi_matches)
{
  const auto & original_cloud = cluster_obj.feature.cluster;
  const std::size_t point_step = original_cloud.point_step;
  const std::size_t n_points = original_cloud.width * original_cloud.height;
  const std::size_t n_rois = roi_matches.size();

  // Assign each point to an ROI index
  // Default: assign to ROI 0 (for non-projected points)
  std::vector<std::size_t> point_roi_assignment(n_points, 0);

  // Build a lookup from raw_pt_idx -> projected 2D point
  std::map<std::size_t, Eigen::Vector2d> projected_map;
  for (const auto & [raw_idx, pt2d] : projected_points) {
    projected_map[raw_idx] = pt2d;
  }

  // Precompute ROI centers for nearest-ROI fallback
  std::vector<Eigen::Vector2d> roi_centers(n_rois);
  for (std::size_t r = 0; r < n_rois; ++r) {
    const auto & roi = roi_matches[r].roi;
    roi_centers[r] = Eigen::Vector2d(
      roi.x_offset + roi.width * 0.5, roi.y_offset + roi.height * 0.5);
  }

  for (std::size_t pt_idx = 0; pt_idx < n_points; ++pt_idx) {
    auto it = projected_map.find(pt_idx);
    if (it == projected_map.end()) {
      // Not projected (z<=0): assign to ROI 0
      point_roi_assignment[pt_idx] = 0;
      continue;
    }

    const Eigen::Vector2d & pt2d = it->second;

    // Check which ROI contains this point
    int best_roi = -1;
    for (std::size_t r = 0; r < n_rois; ++r) {
      if (isPointInsideRoi(roi_matches[r].roi, pt2d.x(), pt2d.y(), 1.0)) {
        best_roi = static_cast<int>(r);
        break;
      }
    }

    if (best_roi < 0) {
      // Point not inside any ROI: assign to nearest ROI center
      double min_dist = std::numeric_limits<double>::max();
      for (std::size_t r = 0; r < n_rois; ++r) {
        double dist = (pt2d - roi_centers[r]).squaredNorm();
        if (dist < min_dist) {
          min_dist = dist;
          best_roi = static_cast<int>(r);
        }
      }
    }

    point_roi_assignment[pt_idx] = static_cast<std::size_t>(best_roi);
  }

  // Group points by ROI assignment
  std::vector<std::vector<std::size_t>> roi_point_indices(n_rois);
  for (std::size_t pt_idx = 0; pt_idx < n_points; ++pt_idx) {
    roi_point_indices[point_roi_assignment[pt_idx]].push_back(pt_idx);
  }

  // Merge small sub-clusters into the largest one
  std::size_t largest_roi = 0;
  std::size_t largest_count = 0;
  for (std::size_t r = 0; r < n_rois; ++r) {
    if (roi_point_indices[r].size() > largest_count) {
      largest_count = roi_point_indices[r].size();
      largest_roi = r;
    }
  }
  for (std::size_t r = 0; r < n_rois; ++r) {
    if (r == largest_roi) continue;
    if (static_cast<int>(roi_point_indices[r].size()) < split_min_point_num_) {
      roi_point_indices[largest_roi].insert(
        roi_point_indices[largest_roi].end(), roi_point_indices[r].begin(),
        roi_point_indices[r].end());
      roi_point_indices[r].clear();
    }
  }

  // Count non-empty sub-clusters
  std::size_t non_empty_count = 0;
  for (std::size_t r = 0; r < n_rois; ++r) {
    if (!roi_point_indices[r].empty()) ++non_empty_count;
  }
  if (non_empty_count < 2) {
    return {};  // splitting not meaningful
  }

  // Build sub-cluster objects
  std::vector<std::pair<ClusterObjType, RoiMatch>> results;
  for (std::size_t r = 0; r < n_rois; ++r) {
    if (roi_point_indices[r].empty()) continue;

    ClusterObjType sub_obj;
    // Build sub-cluster PointCloud2
    auto & sub_cloud = sub_obj.feature.cluster;
    sub_cloud.header = original_cloud.header;
    sub_cloud.fields = original_cloud.fields;
    sub_cloud.point_step = point_step;
    sub_cloud.is_bigendian = original_cloud.is_bigendian;
    sub_cloud.is_dense = original_cloud.is_dense;
    sub_cloud.height = 1;
    sub_cloud.width = roi_point_indices[r].size();
    sub_cloud.row_step = sub_cloud.width * point_step;
    sub_cloud.data.resize(sub_cloud.width * point_step);

    double cx = 0.0, cy = 0.0, cz = 0.0;
    for (std::size_t k = 0; k < roi_point_indices[r].size(); ++k) {
      std::size_t src_offset = roi_point_indices[r][k] * point_step;
      std::size_t dst_offset = k * point_step;
      std::memcpy(&sub_cloud.data[dst_offset], &original_cloud.data[src_offset], point_step);

      // Read x,y,z for centroid (assume float at offset 0,4,8)
      float x, y, z;
      std::memcpy(&x, &original_cloud.data[src_offset + 0], sizeof(float));
      std::memcpy(&y, &original_cloud.data[src_offset + 4], sizeof(float));
      std::memcpy(&z, &original_cloud.data[src_offset + 8], sizeof(float));
      cx += x;
      cy += y;
      cz += z;
    }

    // Compute centroid and set pose
    double n = static_cast<double>(roi_point_indices[r].size());
    sub_obj.object.kinematics.pose_with_covariance.pose.position.x = cx / n;
    sub_obj.object.kinematics.pose_with_covariance.pose.position.y = cy / n;
    sub_obj.object.kinematics.pose_with_covariance.pose.position.z = cz / n;
    sub_obj.object.kinematics.pose_with_covariance.pose.orientation.w = 1.0;

    results.emplace_back(std::move(sub_obj), roi_matches[r]);
  }

  return results;
}

bool RoiClusterFusionNode::out_of_scope(const DetectedObjectWithFeature & obj)
{
  auto cluster = obj.feature.cluster;
  bool is_out = false;
  auto valid_point = [](float p, float min_num, float max_num) -> bool {
    return (p > min_num) && (p < max_num);
  };

  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cluster, "x"), iter_y(cluster, "y"),
       iter_z(cluster, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (!valid_point(*iter_x, filter_scope_min_x_, filter_scope_max_x_)) {
      is_out = true;
      break;
    }

    if (!valid_point(*iter_y, filter_scope_min_y_, filter_scope_max_y_)) {
      is_out = true;
      break;
    }

    if (!valid_point(*iter_z, filter_scope_min_z_, filter_scope_max_z_)) {
      is_out = true;
      break;
    }
  }

  return is_out;
}

bool RoiClusterFusionNode::is_far_enough(
  const DetectedObjectWithFeature & obj, const double distance_threshold)
{
  const auto & position = obj.object.kinematics.pose_with_covariance.pose.position;
  return position.x * position.x + position.y * position.y >
         distance_threshold * distance_threshold;
}

double RoiClusterFusionNode::cal_iou_by_mode(
  const sensor_msgs::msg::RegionOfInterest & roi_1,
  const sensor_msgs::msg::RegionOfInterest & roi_2, const std::string iou_mode)
{
  switch (IOU_MODE_MAP.at(iou_mode)) {
    case 0 /* use iou mode */:
      return calcIoU(roi_1, roi_2);

    case 1 /* use iou_x mode */:
      return calcIoUX(roi_1, roi_2);

    case 2 /* use iou_y mode */:
      return calcIoUY(roi_1, roi_2);
    default:
      return 0.0;
  }
}

void RoiClusterFusionNode::postprocess(
  const ClusterMsgType & processing_msg, ClusterMsgType & output_msg)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  output_msg = processing_msg;

  if (remove_unknown_) {
    // filter by object classification and existence probability
    output_msg.feature_objects.clear();
    for (const auto & feature_object : processing_msg.feature_objects) {
      if (
        feature_object.object.classification.front().label !=
          autoware_perception_msgs::msg::ObjectClassification::UNKNOWN ||
        feature_object.object.existence_probability >= min_roi_existence_prob_) {
        output_msg.feature_objects.push_back(feature_object);
      }
    }
  }
}

}  // namespace autoware::image_projection_based_fusion

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::image_projection_based_fusion::RoiClusterFusionNode)
