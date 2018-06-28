/*
 * Copyright (c) 2016-2017, Riccardo Monica
 *   RIMLab, Department of Engineering and Architecture
 *   University of Parma, Italy
 *   http://www.rimlab.ce.unipr.it/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "rviz_cloud_annotation_class.h"
#include "point_neighborhood_search.h"

// STL
#include <cstring>

// Boost
#include <boost/date_time/posix_time/posix_time.hpp>

// Eigen
#include <Eigen/Dense>

// ROS
#include <eigen_conversions/eigen_msg.h>

#define CLOUD_MARKER_NAME            "cloud"
#define CONTROL_POINT_MARKER_PREFIX  "control_points_"
#define LABEL_POINT_MARKER_PREFIX    "label_points_"

RVizCloudAnnotation::RVizCloudAnnotation(ros::NodeHandle & nh): m_nh(nh)
{
  std::string param_string;
  std::string param_string2;
  double param_double;
  int param_int;

  m_nh.param<std::string>(PARAM_NAME_UPDATE_TOPIC,param_string,PARAM_DEFAULT_UPDATE_TOPIC);
  m_interactive_marker_server = InteractiveMarkerServerPtr(new InteractiveMarkerServer(param_string));

  m_nh.param<std::string>(PARAM_NAME_CLOUD_FILENAME,param_string,PARAM_DEFAULT_CLOUD_FILENAME);
  m_nh.param<std::string>(PARAM_NAME_NORMAL_SOURCE,param_string2,PARAM_DEFAULT_NORMAL_SOURCE);
  m_cloud = PointXYZRGBNormalCloud::Ptr(new PointXYZRGBNormalCloud);
  try
  {
    LoadCloud(param_string,param_string2,*m_cloud);
  }
  catch (const std::string & msg)
  {
    ROS_FATAL("rviz_cloud_annotation: %s",msg.c_str());
    std::exit(1);
  }
  m_kdtree = KdTree::Ptr(new KdTree);
  m_kdtree->setInputCloud(m_cloud);

  {
    PointNeighborhood::Conf conf;

    m_nh.param<int>(PARAM_NAME_NEIGH_SEARCH_TYPE,param_int,PARAM_DEFAULT_NEIGH_SEARCH_TYPE);
    m_nh.param<std::string>(PARAM_NAME_NEIGH_SEARCH_PARAMS,param_string,PARAM_DEFAULT_NEIGH_SEARCH_PARAMS);
    if (param_int == PARAM_DEFAULT_NEIGH_SEARCH_TYPE && param_string == PARAM_DEFAULT_NEIGH_SEARCH_PARAMS)
    {
      ROS_INFO("rviz_cloud_annotation: parameter %s is at default value, using %s instead.",
               PARAM_NAME_NEIGH_SEARCH_PARAMS,PARAM_NAME_NEIGH_SEARCH_DISTANCE);
      m_nh.param<double>(PARAM_NAME_NEIGH_SEARCH_DISTANCE,param_double,PARAM_DEFAULT_NEIGH_SEARCH_DISTANCE);
      param_string = boost::lexical_cast<std::string>(param_double);
    }

    try
    {
      conf.searcher = PointNeighborhoodSearch::CreateFromString(param_int,param_string);
    }
    catch (const PointNeighborhoodSearch::ParserException & ex)
    {
      ROS_ERROR("rviz_cloud_annotation: could not configure point neighborhood search from ROS param: %s",ex.message.c_str());
      conf.searcher = PointNeighborhoodSearch::CreateFromString(PARAM_DEFAULT_NEIGH_SEARCH_TYPE,
        boost::lexical_cast<std::string>(PARAM_DEFAULT_NEIGH_SEARCH_DISTANCE));
    }

    m_nh.param<double>(PARAM_NAME_COLOR_IMPORTANCE,param_double,PARAM_DEFAULT_COLOR_IMPORTANCE);
    conf.color_importance = param_double;

    m_nh.param<double>(PARAM_NAME_NORMAL_IMPORTANCE,param_double,PARAM_DEFAULT_NORMAL_IMPORTANCE);
    conf.normal_importance = param_double;

    m_nh.param<double>(PARAM_NAME_POSITION_IMPORTANCE,param_double,PARAM_DEFAULT_POSITION_IMPORTANCE);
    conf.position_importance = param_double;

    m_nh.param<double>(PARAM_NAME_MAX_DISTANCE,param_double,PARAM_DEFAULT_MAX_DISTANCE);
    conf.max_distance = param_double;

    m_nh.param<int>(PARAM_NAME_WEIGHT_STEPS,param_int,PARAM_DEFAULT_WEIGHT_STEPS);
    m_control_point_max_weight = std::max<uint32>(1,param_int);

    ROS_INFO("rviz_cloud_annotation: building point neighborhood...");
    m_point_neighborhood = PointNeighborhood::ConstPtr(new PointNeighborhood(m_cloud,conf));
    ROS_INFO("rviz_cloud_annotation: done.");
  }

  RVizCloudAnnotationPoints::Ptr default_annotation = RVizCloudAnnotationPoints::Ptr(new RVizCloudAnnotationPoints(
    m_cloud->size(),m_control_point_max_weight,m_point_neighborhood));
  m_annotation = default_annotation;
  m_undo_redo.SetAnnotation(default_annotation);

  m_nh.param<std::string>(PARAM_NAME_FRAME_ID,m_frame_id,PARAM_DEFAULT_FRAME_ID);

  m_nh.param<double>(PARAM_NAME_POINT_SIZE,param_double,PARAM_DEFAULT_POINT_SIZE);
  m_point_size = param_double;

  m_nh.param<double>(PARAM_NAME_LABEL_SIZE,param_double,PARAM_DEFAULT_LABEL_SIZE);
  m_label_size = param_double;

  m_nh.param<double>(PARAM_NAME_CONTROL_LABEL_SIZE,param_double,PARAM_DEFAULT_CONTROL_LABEL_SIZE);
  m_control_label_size = param_double;

  m_nh.param<bool>(PARAM_NAME_SHOW_POINTS_BACK_LABELS,m_show_points_back_labels,PARAM_DEFAULT_SHOW_POINTS_BACK_LABELS);

  m_nh.param<std::string>(PARAM_NAME_CONTROL_POINT_VISUAL,param_string,PARAM_DEFAULT_CONTROL_POINT_VISUAL);
  if (param_string == PARAM_VALUE_CONTROL_POINT_VISUAL_LINE)
    m_control_points_visual = CONTROL_POINT_VISUAL_LINE;
  else if (param_string == PARAM_VALUE_CONTROL_POINT_VISUAL_SPHERE)
    m_control_points_visual = CONTROL_POINT_VISUAL_SPHERE;
  else if (param_string == PARAM_VALUE_CONTROL_POINT_VISUAL_THREE_SPHERES)
    m_control_points_visual = CONTROL_POINT_VISUAL_THREE_SPHERES;
  else
  {
    ROS_ERROR("rviz_cloud_annotation: invalid value for parameter %s: %s",PARAM_NAME_CONTROL_POINT_VISUAL,param_string.c_str());
    m_control_points_visual = CONTROL_POINT_VISUAL_SPHERE;
  }

  m_nh.param<double>(PARAM_NAME_CP_WEIGHT_SCALE_FRACTION,param_double,PARAM_DEFAULT_CP_WEIGHT_SCALE_FRACTION);
  m_cp_weight_scale_fraction = std::min<float>(1.0,std::max(0.0,param_double));

  m_nh.param<bool>(PARAM_NAME_ZERO_WEIGHT_CP_SHOW,m_show_zero_weight_control_points,PARAM_DEFAULT_ZERO_WEIGHT_CP_SHOW);

  m_nh.param<std::string>(PARAM_NAME_RECT_SELECTION_TOPIC,param_string,PARAM_DEFAULT_RECT_SELECTION_TOPIC);
  m_rect_selection_sub = m_nh.subscribe(param_string,1,&RVizCloudAnnotation::onRectangleSelectionViewport,this);

  m_nh.param<std::string>(PARAM_NAME_SAVE_TOPIC,param_string,PARAM_DEFAULT_SAVE_TOPIC);
  m_save_sub = m_nh.subscribe(param_string,1,&RVizCloudAnnotation::onSave,this);

  m_nh.param<std::string>(PARAM_NAME_RESTORE_TOPIC,param_string,PARAM_DEFAULT_RESTORE_TOPIC);
  m_restore_sub = m_nh.subscribe(param_string,1,&RVizCloudAnnotation::onRestore,this);

  m_nh.param<std::string>(PARAM_NAME_CLEAR_TOPIC,param_string,PARAM_DEFAULT_CLEAR_TOPIC);
  m_clear_sub = m_nh.subscribe(param_string,1,&RVizCloudAnnotation::onClear,this);

  m_nh.param<std::string>(PARAM_NAME_SET_EDIT_MODE_TOPIC,param_string,PARAM_DEFAULT_SET_EDIT_MODE_TOPIC);
  m_set_edit_mode_sub = m_nh.subscribe(param_string,1,&RVizCloudAnnotation::onSetEditMode,this);

  m_nh.param<std::string>(PARAM_NAME_TOGGLE_NONE_TOPIC,param_string,PARAM_DEFAULT_TOGGLE_NONE_TOPIC);
  m_toggle_none_sub = m_nh.subscribe(param_string,1,&RVizCloudAnnotation::onToggleNoneMode,this);

  m_nh.param<std::string>(PARAM_NAME_SET_CURRENT_LABEL_TOPIC,param_string,PARAM_DEFAULT_SET_CURRENT_LABEL_TOPIC);
  m_set_current_label_sub = m_nh.subscribe(param_string,1,&RVizCloudAnnotation::onSetCurrentLabel,this);

  m_nh.param<std::string>(PARAM_NAME_SET_EDIT_MODE_TOPIC2,param_string,PARAM_DEFAULT_SET_EDIT_MODE_TOPIC2);
  m_set_edit_mode_pub = m_nh.advertise<std_msgs::UInt32>(param_string,1);

  m_nh.param<std::string>(PARAM_NAME_CURRENT_LABEL_TOPIC,param_string,PARAM_DEFAULT_CURRENT_LABEL_TOPIC);
  m_set_current_label_pub = m_nh.advertise<std_msgs::UInt32>(param_string,1);

  m_nh.param<std::string>(PARAM_NAME_POINT_COUNT_UPDATE_TOPIC,param_string,PARAM_DEFAULT_POINT_COUNT_UPDATE_TOPIC);
  m_point_count_update_pub = m_nh.advertise<std_msgs::UInt64MultiArray>(param_string,1,true);

  m_nh.param<std::string>(PARAM_NAME_ANNOTATED_CLOUD,m_ann_cloud_filename_out,PARAM_DEFAULT_ANNOTATED_CLOUD);
  m_nh.param<std::string>(PARAM_NAME_ANN_FILENAME_IN,m_annotation_filename_in,PARAM_DEFAULT_ANN_FILENAME_IN);
  m_nh.param<std::string>(PARAM_NAME_ANN_FILENAME_OUT,m_annotation_filename_out,PARAM_DEFAULT_ANN_FILENAME_OUT);
  m_nh.param<std::string>(PARAM_NAME_LABEL_NAMES_FILENAME,m_label_names_filename_out,PARAM_DEFAULT_LABEL_NAMES_FILENAME);

  m_nh.param<std::string>(PARAM_NAME_SET_NAME_TOPIC,param_string,PARAM_DEFAULT_SET_NAME_TOPIC);
  m_set_name_sub = m_nh.subscribe(param_string,1,&RVizCloudAnnotation::onSetName,this);

  m_nh.param<std::string>(PARAM_NAME_SET_NAME_TOPIC2,param_string,PARAM_DEFAULT_SET_NAME_TOPIC2);
  m_set_name_pub = m_nh.advertise<std_msgs::String>(param_string,1,true);

  m_nh.param<std::string>(PARAM_NAME_VIEW_CONTROL_POINTS_TOPIC,param_string,PARAM_DEFAULT_VIEW_CONTROL_POINTS_TOPIC);
  m_view_control_points_sub = m_nh.subscribe(param_string,1,&RVizCloudAnnotation::onViewControlPoints,this);

  m_nh.param<std::string>(PARAM_NAME_VIEW_CLOUD_TOPIC,param_string,PARAM_DEFAULT_VIEW_CLOUD_TOPIC);
  m_view_cloud_sub = m_nh.subscribe(param_string,1,&RVizCloudAnnotation::onViewCloud,this);

  m_nh.param<std::string>(PARAM_NAME_VIEW_LABEL_TOPIC,param_string,PARAM_DEFAULT_VIEW_LABEL_TOPIC);
  m_view_labels_sub = m_nh.subscribe(param_string,1,&RVizCloudAnnotation::onViewLabels,this);

  m_nh.param<std::string>(PARAM_NAME_UNDO_REDO_STATE_TOPIC,param_string,PARAM_DEFAULT_UNDO_REDO_STATE_TOPIC);
  m_undo_redo_state_pub = m_nh.advertise<rviz_cloud_annotation::UndoRedoState>(param_string,1,true);

  m_nh.param<std::string>(PARAM_NAME_UNDO_TOPIC,param_string,PARAM_DEFAULT_UNDO_TOPIC);
  m_undo_sub = m_nh.subscribe(param_string,1,&RVizCloudAnnotation::onUndo,this);

  m_nh.param<std::string>(PARAM_NAME_REDO_TOPIC,param_string,PARAM_DEFAULT_REDO_TOPIC);
  m_redo_sub = m_nh.subscribe(param_string,1,&RVizCloudAnnotation::onRedo,this);

  m_nh.param<std::string>(PARAM_NAME_POINT_SIZE_CHANGE_TOPIC,param_string,PARAM_DEFAULT_POINT_SIZE_CHANGE_TOPIC);
  m_point_size_change_sub = m_nh.subscribe(param_string,1,&RVizCloudAnnotation::onPointSizeChange,this);

  m_nh.param<float>(PARAM_NAME_POINT_SIZE_CHANGE_MULT,m_point_size_change_multiplier,PARAM_DEFAULT_POINT_SIZE_CHANGE_MULT);

  m_nh.param<std::string>(PARAM_NAME_CONTROL_POINT_WEIGHT_TOPIC,param_string,PARAM_DEFAULT_CONTROL_POINT_WEIGHT_TOPIC);
  m_control_points_weight_sub = m_nh.subscribe(param_string,1,&RVizCloudAnnotation::onControlPointWeightChange,this);

  m_nh.param<std::string>(PARAM_NAME_CONTROL_POINT_MAX_WEIGHT_TOPIC,param_string,PARAM_DEFAULT_CONTROL_POINT_MAX_WEIGHT_TOPIC);
  m_control_point_weight_max_weight_pub = nh.advertise<std_msgs::UInt32>(param_string,1,true);

  m_nh.param<std::string>(PARAM_NAME_GOTO_FIRST_UNUSED_TOPIC,param_string,PARAM_DEFAULT_GOTO_FIRST_UNUSED_TOPIC);
  m_goto_first_unused_sub = nh.subscribe(param_string,1,&RVizCloudAnnotation::onGotoFirstUnused,this);

  m_nh.param<std::string>(PARAM_NAME_GOTO_LAST_UNUSED_TOPIC,param_string,PARAM_DEFAULT_GOTO_LAST_UNUSED_TOPIC);
  m_goto_last_unused_sub = nh.subscribe(param_string,1,&RVizCloudAnnotation::onGotoLastUnused,this);

  m_nh.param<std::string>(PARAM_NAME_GOTO_FIRST_TOPIC,param_string,PARAM_DEFAULT_GOTO_FIRST_TOPIC);
  m_goto_first_sub = nh.subscribe(param_string,1,&RVizCloudAnnotation::onGotoFirst,this);

  m_nh.param<std::string>(PARAM_NAME_GOTO_NEXT_UNUSED_TOPIC,param_string,PARAM_DEFAULT_GOTO_NEXT_UNUSED_TOPIC);
  m_goto_next_unused_sub = nh.subscribe(param_string,1,&RVizCloudAnnotation::onGotoNextUnused,this);

  m_nh.param<double>(PARAM_NAME_AUTOSAVE_TIME,param_double,PARAM_DEFAULT_AUTOSAVE_TIME);
  if (param_double >= 1.0)
  {
    m_autosave_timer = m_nh.createTimer(ros::Duration(param_double),&RVizCloudAnnotation::onAutosave,this,false,false);
    ROS_INFO("rviz_cloud_annotation: autosave every %f seconds.",float(param_double));
  }
  m_nh.param<bool>(PARAM_NAME_AUTOSAVE_TIMESTAMP,m_autosave_append_timestamp,PARAM_DEFAULT_AUTOSAVE_TIMESTAMP);

  m_current_label = 1;
  m_edit_mode = EDIT_MODE_NONE;
  m_prev_edit_mode = EDIT_MODE_NONE;

  m_point_size_multiplier = 1.0;

  m_control_point_weight_step = m_control_point_max_weight;

  m_view_cloud = m_view_labels = m_view_control_points = true;

  SendCloudMarker(true);
  SendControlPointMaxWeight();
  Restore(m_annotation_filename_in);

  m_autosave_timer.start();
}

RVizCloudAnnotation::InteractiveMarker RVizCloudAnnotation::ControlPointsToMarker(const PointXYZRGBNormalCloud & cloud,
                                        const ControlPointDataVector & control_points,
                                        const uint64 label,const bool interactive)
{
  const uint64 control_size = control_points.size();

  InteractiveMarker marker;
  marker.header.frame_id = m_frame_id;
  marker.name = std::string(CONTROL_POINT_MARKER_PREFIX) + boost::lexical_cast<std::string>(label);
  marker.description = "";

  const pcl::RGB color = pcl::GlasbeyLUT::at((label - 1) % 256);

  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;

  const bool use_spheres = m_control_points_visual == CONTROL_POINT_VISUAL_SPHERE ||
                           m_control_points_visual == CONTROL_POINT_VISUAL_THREE_SPHERES;

  const float control_label_size = use_spheres ? m_control_label_size / 2.0 : m_control_label_size;
  const uint64 cp_marker_count = m_control_points_visual == CONTROL_POINT_VISUAL_SPHERE ? 1 :
                                 m_control_points_visual == CONTROL_POINT_VISUAL_THREE_SPHERES ? 3 :
                                 m_control_points_visual == CONTROL_POINT_VISUAL_LINE ? 2 :
                                 2;

  Marker cloud_marker;
  cloud_marker.type = use_spheres ? int(Marker::SPHERE_LIST) : int(Marker::LINE_LIST);
  cloud_marker.scale.x = control_label_size;
  cloud_marker.scale.y = use_spheres ? control_label_size : 0.0;
  cloud_marker.scale.z = use_spheres ? control_label_size : 0.0;
  cloud_marker.color.r = float(color.r) / 255.0;
  cloud_marker.color.g = float(color.g) / 255.0;
  cloud_marker.color.b = float(color.b) / 255.0;
  cloud_marker.color.a = 1.0;

  cloud_marker.points.resize(control_size * cp_marker_count);
  uint64 out_marker_count = 0;
  for(uint64 i = 0; i < control_size; i++)
  {
    const PointXYZRGBNormal & pt = cloud[control_points[i].point_id];

    const uint32 weight_step_id = control_points[i].weight_step_id;
    if (weight_step_id == 0 && !m_show_zero_weight_control_points)
      continue;

    const float weight_rel = float(weight_step_id) / float(m_control_point_max_weight);
    const float weight_scale = (1.0 - m_cp_weight_scale_fraction) + weight_rel * m_cp_weight_scale_fraction;

    cloud_marker.points[out_marker_count].x = pt.x;
    cloud_marker.points[out_marker_count].y = pt.y;
    cloud_marker.points[out_marker_count].z = pt.z;
    out_marker_count++;

    if (cp_marker_count > 1)
    {
      cloud_marker.points[out_marker_count].x = pt.x + pt.normal_x * control_label_size * weight_scale;
      cloud_marker.points[out_marker_count].y = pt.y + pt.normal_y * control_label_size * weight_scale;
      cloud_marker.points[out_marker_count].z = pt.z + pt.normal_z * control_label_size * weight_scale;
      out_marker_count++;
    }

    if (cp_marker_count > 2)
    {
      cloud_marker.points[out_marker_count].x = pt.x + pt.normal_x * control_label_size * weight_scale / 2.0;
      cloud_marker.points[out_marker_count].y = pt.y + pt.normal_y * control_label_size * weight_scale / 2.0;
      cloud_marker.points[out_marker_count].z = pt.z + pt.normal_z * control_label_size * weight_scale / 2.0;
      out_marker_count++;
    }
  }
  cloud_marker.points.resize(out_marker_count);

  visualization_msgs::InteractiveMarkerControl points_control;
  points_control.always_visible = true;
  points_control.interaction_mode = interactive ?
    int32(visualization_msgs::InteractiveMarkerControl::BUTTON) :
    int32(visualization_msgs::InteractiveMarkerControl::NONE);
  if (m_view_control_points)
    points_control.markers.push_back(cloud_marker);
  marker.controls.push_back(points_control);

  return marker;
}

RVizCloudAnnotation::InteractiveMarker RVizCloudAnnotation::LabelsToMarker(
  const PointXYZRGBNormalCloud & cloud,
  const Uint64Vector & labels,
  const uint64 label,
  const bool interactive)
{
  const uint64 labels_size = labels.size();

  InteractiveMarker marker;
  marker.header.frame_id = m_frame_id;
  marker.name = std::string(LABEL_POINT_MARKER_PREFIX) + boost::lexical_cast<std::string>(label);
  marker.description = "";

  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;

  pcl::RGB color;
  if (label == 0)
    color.r = color.g = color.b = 0;
  else
    color = pcl::GlasbeyLUT::at((label - 1) % 256);

  const float label_size = m_point_size_multiplier * (m_view_cloud ? m_label_size : m_point_size);
  const float normal_mult = m_view_cloud ? (m_label_size / 2.0) : 0.0;

  Marker cloud_marker;
  cloud_marker.type = Marker::POINTS;
  cloud_marker.scale.x = label_size;
  cloud_marker.scale.y = label_size;
  cloud_marker.scale.z = label_size;
  cloud_marker.color.r = color.r / 255.0;
  cloud_marker.color.g = color.g / 255.0;
  cloud_marker.color.b = color.b / 255.0;
  cloud_marker.color.a = 1.0;

  cloud_marker.points.resize(labels_size);
  for(uint64 i = 0; i < labels_size; i++)
  {
    const PointXYZRGBNormal & pt = cloud[labels[i]];

    cloud_marker.points[i].x = pt.x + NANToZero(pt.normal_x) * normal_mult;
    cloud_marker.points[i].y = pt.y + NANToZero(pt.normal_y) * normal_mult;
    cloud_marker.points[i].z = pt.z + NANToZero(pt.normal_z) * normal_mult;
  }

  if (m_view_cloud && m_show_points_back_labels)
  {
    cloud_marker.points.resize(labels_size * 2);

    for(uint64 i = 0; i < labels_size; i++)
    {
      const PointXYZRGBNormal & pt = cloud[labels[i]];

      cloud_marker.points[labels_size + i].x = pt.x - NANToZero(pt.normal_x) * normal_mult; // point on the back
      cloud_marker.points[labels_size + i].y = pt.y - NANToZero(pt.normal_y) * normal_mult;
      cloud_marker.points[labels_size + i].z = pt.z - NANToZero(pt.normal_z) * normal_mult;
    }
  }

  visualization_msgs::InteractiveMarkerControl points_control;
  points_control.always_visible = true;
  points_control.interaction_mode = interactive ?
    int32(visualization_msgs::InteractiveMarkerControl::BUTTON) :
    int32(visualization_msgs::InteractiveMarkerControl::NONE);
  if (m_view_labels && (!m_view_cloud || label != 0))
    points_control.markers.push_back(cloud_marker);
  marker.controls.push_back(points_control);

  return marker;
}

RVizCloudAnnotation::InteractiveMarker RVizCloudAnnotation::CloudToMarker(const PointXYZRGBNormalCloud & cloud,const bool interactive)
{
  const uint64 cloud_size = cloud.size();

  InteractiveMarker marker;
  marker.header.frame_id = m_frame_id;
  marker.name = CLOUD_MARKER_NAME;
  marker.description = "";

  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;

  Marker cloud_marker;
  cloud_marker.type = Marker::POINTS;
  cloud_marker.scale.x = m_point_size_multiplier * m_point_size;
  cloud_marker.scale.y = m_point_size_multiplier * m_point_size;
  cloud_marker.scale.z = m_point_size_multiplier * m_point_size;
  cloud_marker.color.r = 1.0;
  cloud_marker.color.g = 1.0;
  cloud_marker.color.b = 1.0;
  cloud_marker.color.a = 1.0;

  cloud_marker.points.resize(cloud_size);
  cloud_marker.colors.resize(cloud_size);
  for(uint64 i = 0; i < cloud_size; i++)
  {
    const PointXYZRGBNormal & pt = cloud[i];

    cloud_marker.points[i].x = pt.x;
    cloud_marker.points[i].y = pt.y;
    cloud_marker.points[i].z = pt.z;

    cloud_marker.colors[i].r = pt.r / 255.0;
    cloud_marker.colors[i].g = pt.g / 255.0;
    cloud_marker.colors[i].b = pt.b / 255.0;
    cloud_marker.colors[i].a = 1.0;
  }

  visualization_msgs::InteractiveMarkerControl points_control;
  points_control.always_visible = true;
  points_control.interaction_mode = interactive ?
    int32(visualization_msgs::InteractiveMarkerControl::BUTTON) :
    int32(visualization_msgs::InteractiveMarkerControl::NONE);
  if (m_view_cloud)
    points_control.markers.push_back(cloud_marker);
  marker.controls.push_back(points_control);

  return marker;
}

void RVizCloudAnnotation::Restore(const std::string & filename)
{
  std::ifstream ifile(filename.c_str());
  if (!ifile)
  {
    ROS_ERROR("rviz_cloud_annotation: could not open file: %s",filename.c_str());
    return;
  }

  ROS_INFO("rviz_cloud_annotation: loading file: %s",filename.c_str());

  RVizCloudAnnotationPoints::Ptr maybe_new_annotation;
  try
  {
    maybe_new_annotation = RVizCloudAnnotationPoints::Deserialize(ifile,m_control_point_max_weight,m_point_neighborhood);
  }
  catch (const RVizCloudAnnotationPoints::IOE & e)
  {
    ROS_ERROR("rviz_cloud_annotation: could not load file %s, reason: %s.",filename.c_str(),e.description.c_str());
    return;
  }

  if (maybe_new_annotation->GetCloudSize() != m_annotation->GetCloudSize())
  {
    const uint new_size = maybe_new_annotation->GetCloudSize();
    const uint old_size = m_annotation->GetCloudSize();
    ROS_ERROR("rviz_cloud_annotation: file was created for a cloud of size %u, but it is %u. Load operation aborted.",
              new_size,old_size);
    return;
  }

  ClearControlPointsMarker(RangeUint64(1,m_annotation->GetNextLabel()),false);
  m_annotation = maybe_new_annotation;
  m_undo_redo.SetAnnotation(maybe_new_annotation);
  SendControlPointsMarker(RangeUint64(1,m_annotation->GetNextLabel()),true);
  SendName();
  SendPointCounts(RangeUint64(1,m_annotation->GetNextLabel()));
  SendUndoRedoState();

  ROS_INFO("rviz_cloud_annotation: file loaded.");
}

void RVizCloudAnnotation::LoadCloud(const std::string & filename,const std::string & normal_source,PointXYZRGBNormalCloud & cloud)
{
  cloud.clear();

  pcl::PCLPointCloud2 cloud2;

  if (pcl::io::loadPCDFile(filename,cloud2))
    throw std::string(std::string("could not load cloud: ") + filename);

  PointXYZRGBCloud xyz_rgb_cloud;
  PointNormalCloud normal_cloud;

  pcl::fromPCLPointCloud2(cloud2,xyz_rgb_cloud);

  if (normal_source == PARAM_VALUE_NORMAL_SOURCE_CLOUD)
    pcl::fromPCLPointCloud2(cloud2,normal_cloud);
  else if (normal_source.substr(0,std::string(PARAM_VALUE_NORMAL_SOURCE_OTHER_CLOUD).size()) == PARAM_VALUE_NORMAL_SOURCE_OTHER_CLOUD)
  {
    const std::string filename_n = normal_source.substr(std::string(PARAM_VALUE_NORMAL_SOURCE_OTHER_CLOUD).size());
    if (pcl::io::loadPCDFile(filename_n,normal_cloud))
      throw std::string(std::string("could not load normal source cloud: ") + filename_n);
  }
  else
    throw std::string(std::string("invalid normal source: ") + normal_source);

  if (normal_cloud.size() != xyz_rgb_cloud.size())
    throw std::string("cloud and normal cloud have different sizes");

  pcl::copyPointCloud(xyz_rgb_cloud,cloud);

  for (uint64 i = 0; i < normal_cloud.size(); i++)
  {
    cloud[i].normal_x = normal_cloud[i].normal_x;
    cloud[i].normal_y = normal_cloud[i].normal_y;
    cloud[i].normal_z = normal_cloud[i].normal_z;
    cloud[i].curvature = normal_cloud[i].curvature;
  }
}

void RVizCloudAnnotation::Save(const bool is_autosave)
{
  if (is_autosave)
    ROS_INFO("rviz_cloud_annotation: auto-saving.");

  std::string filename = m_annotation_filename_out;
  if (is_autosave && m_autosave_append_timestamp)
    filename = AppendTimestampBeforeExtension(filename);

  std::ofstream ofile(filename.c_str());
  if (!ofile)
  {
    ROS_ERROR("rviz_cloud_annotation: could not create file: %s",filename.c_str());
    return;
  }

  ROS_INFO("rviz_cloud_annotation: saving file: %s",filename.c_str());

  try
  {
    m_annotation->Serialize(ofile);
  }
  catch (const RVizCloudAnnotationPoints::IOE & e)
  {
    ROS_ERROR("rviz_cloud_annotation: could not save file %s, reason: %s.",filename.c_str(),e.description.c_str());
  }
  ROS_INFO("rviz_cloud_annotation: done.");

  std::string cloud_filename = m_ann_cloud_filename_out;
  if (is_autosave && m_autosave_append_timestamp)
    cloud_filename = AppendTimestampBeforeExtension(cloud_filename);

  ROS_INFO("rviz_cloud_annotation: saving cloud: %s",cloud_filename.c_str());
  {
    PointXYZRGBLCloud cloud_out;
    pcl::copyPointCloud(*m_cloud,cloud_out);
    m_annotation->LabelCloudWithColor(cloud_out);
    if (pcl::io::savePCDFileBinary(cloud_filename,cloud_out))
      ROS_ERROR("rviz_cloud_annotation: could not save labeled cloud.");
  }
  ROS_INFO("rviz_cloud_annotation: done.");

  std::string names_filename = m_label_names_filename_out;
  if (is_autosave && m_autosave_append_timestamp)
    names_filename = AppendTimestampBeforeExtension(names_filename);

  ROS_INFO("rviz_cloud_annotation: saving names: %s",names_filename.c_str());
  {
    std::ofstream ofile(names_filename.c_str());
    for (uint64 i = 1; i < m_annotation->GetNextLabel(); i++)
      ofile << i << ": " << m_annotation->GetNameForLabel(i) << "\n";
    if (!ofile)
      ROS_ERROR("rviz_cloud_annotation: could not write file.");
  }
  ROS_INFO("rviz_cloud_annotation: done.");
}

void RVizCloudAnnotation::SetEditMode(const uint64 new_edit_mode)
{
  if (m_edit_mode == new_edit_mode)
    return; // nothing to do

  const char * info_string;

  bool send_cloud = false;
  switch (new_edit_mode)
  {
    case EDIT_MODE_NONE:
      if (m_edit_mode != EDIT_MODE_NONE)
        send_cloud = true;
      info_string = "NONE";
      break;
    case EDIT_MODE_CONTROL_POINT:
      if (m_edit_mode == EDIT_MODE_NONE)
        send_cloud = true;
      info_string = "CONTROL_POINT";
      break;
    case EDIT_MODE_ERASER:
      if (m_edit_mode == EDIT_MODE_NONE)
        send_cloud = true;
      info_string = "ERASER";
      break;
    case EDIT_MODE_COLOR_PICKER:
      if (m_edit_mode == EDIT_MODE_NONE)
        send_cloud = true;
      info_string = "COLOR_PICKER";
      break;
    default:
      ROS_ERROR("rviz_cloud_annotation: unsupported edit mode %u received.",(unsigned int)(new_edit_mode));
      return; // invalid
  }

  ROS_INFO("rviz_cloud_annotation: edit mode is now: %s",info_string);

  if ((m_edit_mode == EDIT_MODE_NONE) || (new_edit_mode == EDIT_MODE_NONE))
    m_prev_edit_mode = m_edit_mode; // there must be at least one EDIT_MODE_NONE between current and prev
  m_edit_mode = new_edit_mode;

  std_msgs::UInt32 msg;
  msg.data = m_edit_mode;
  m_set_edit_mode_pub.publish(msg);

  if (send_cloud)
  {
    SendCloudMarker(false);
    SendControlPointsMarker(RangeUint64(1,m_annotation->GetNextLabel()),true);
  }
}

std::string RVizCloudAnnotation::GetClickType(const std::string & marker_name,uint64 & label_out) const
{
  label_out = 0;
  if (marker_name == CLOUD_MARKER_NAME)
    return CLOUD_MARKER_NAME;

  std::string result;
  std::string num_str;
  if (marker_name.substr(0,std::strlen(CONTROL_POINT_MARKER_PREFIX)) == CONTROL_POINT_MARKER_PREFIX)
  {
    result = CONTROL_POINT_MARKER_PREFIX;
    num_str = marker_name.substr(std::strlen(CONTROL_POINT_MARKER_PREFIX));
  }
  else if (marker_name.substr(0,std::strlen(LABEL_POINT_MARKER_PREFIX)) == LABEL_POINT_MARKER_PREFIX)
  {
    result = LABEL_POINT_MARKER_PREFIX;
    num_str = marker_name.substr(std::strlen(LABEL_POINT_MARKER_PREFIX));
  }
  else
  {
    ROS_ERROR("rviz_cloud_annotation: click: unsupported marker name in message: %s",marker_name.c_str());
    return "";
  }

  try
  {
    label_out = boost::lexical_cast<uint64>(num_str);
  }
  catch (const boost::bad_lexical_cast &)
  {
    ROS_ERROR("rviz_cloud_annotation: click: could not convert %s to number.",num_str.c_str());
    return "";
  }

  return result;
}

RVizCloudAnnotation::uint64 RVizCloudAnnotation::GetClickedPointId(const InteractiveMarkerFeedback & click_feedback,bool & ok)
{
  ok = false;
  const std::string marker_name = click_feedback.marker_name;

  uint64 label;
  std::string click_type = GetClickType(marker_name,label);

  if (click_type == CLOUD_MARKER_NAME || click_type == LABEL_POINT_MARKER_PREFIX)
  {
    ROS_INFO("rviz_cloud_annotation: clicked on cloud point.");

    PointXYZRGBNormal click_pt;
    click_pt.x = click_feedback.mouse_point.x;
    click_pt.y = click_feedback.mouse_point.y;
    click_pt.z = click_feedback.mouse_point.z;

    ROS_INFO("rviz_cloud_annotation: click at: %f %f %f",float(click_pt.x),float(click_pt.y),float(click_pt.z));

    std::vector<int> idxs(1);
    std::vector<float> dsts(1);

    if (m_kdtree->nearestKSearch(click_pt,1,idxs,dsts) <= 0)
    {
      ROS_WARN("rviz_cloud_annotation: point was clicked, but no nearest cloud point found.");
      return 0;
    }

    const uint64 idx = idxs[0];
    const float dst = std::sqrt(dsts[0]);

    ROS_INFO("rviz_cloud_annotation: clicked on point: %u (accuracy: %f)",(unsigned int)(idx),float(dst));

    ok = true;
    return idx;
  }

  if (click_type == CONTROL_POINT_MARKER_PREFIX)
  {
    if (label == 0 || label >= m_annotation->GetNextLabel())
    {
      ROS_ERROR("rviz_cloud_annotation: click: invalid control point label %u",(unsigned int)(label));
      return 0;
    }

    const ControlPointDataVector control_points = m_annotation->GetControlPointList(label);
    if (control_points.empty())
    {
      ROS_ERROR("rviz_cloud_annotation: click: control point label %u is empty!",(unsigned int)(label));
      return 0;
    }

    const Eigen::Vector3f click_pt(click_feedback.mouse_point.x,click_feedback.mouse_point.y,click_feedback.mouse_point.z);

    uint64 nearest_idx;
    float nearest_sqr_dist;
    for (uint64 i = 0; i < control_points.size(); i++)
    {
      const PointXYZRGBNormal & pt = (*m_cloud)[control_points[i].point_id];
      const Eigen::Vector3f ept(pt.x,pt.y,pt.z);
      const Eigen::Vector3f en(pt.normal_x,pt.normal_y,pt.normal_z);
      const Eigen::Vector3f shift_pt = ept + en * m_control_label_size / 2.0;
      if (i == 0 || (shift_pt - click_pt).squaredNorm() < nearest_sqr_dist)
      {
        nearest_idx = i;
        nearest_sqr_dist = (shift_pt - click_pt).squaredNorm();
      }
    }

    const uint64 idx = control_points[nearest_idx].point_id;

    ROS_INFO("rviz_cloud_annotation: clicked on control point: %u (accuracy: %f)",
             (unsigned int)(idx),float(std::sqrt(nearest_sqr_dist)));

    ok = true;
    return idx;
  }

  return 0;
}

void RVizCloudAnnotation::SendUndoRedoState()
{
  rviz_cloud_annotation::UndoRedoState msg;
  msg.redo_enabled = m_undo_redo.IsRedoEnabled();
  msg.undo_enabled = m_undo_redo.IsUndoEnabled();
  msg.redo_description = m_undo_redo.GetRedoDescription();
  msg.undo_description = m_undo_redo.GetUndoDescription();
  m_undo_redo_state_pub.publish(msg);
}

void RVizCloudAnnotation::SendName()
{
  std::string name = m_annotation->GetNameForLabel(m_current_label);
  std_msgs::String msg;
  msg.data = name;
  m_set_name_pub.publish(msg);
}

void RVizCloudAnnotation::SendPointCounts(const Uint64Vector & labels)
{
  const uint64 labels_size = labels.size();
  std_msgs::UInt64MultiArray msg;
  for (uint64 i = 0; i < labels_size; i++)
  {
    const uint64 label = labels[i];
    const uint64 count = m_annotation->GetLabelPointCount(label);
    msg.data.push_back(label);
    msg.data.push_back(count);
  }
  m_point_count_update_pub.publish(msg);
}

void RVizCloudAnnotation::onUndo(const std_msgs::Empty &)
{
  if (!m_undo_redo.IsUndoEnabled())
    return;

  ROS_INFO("rviz_cloud_annotation: Undo.");
  const Uint64Vector changed = m_undo_redo.Undo();
  SendControlPointsMarker(changed,true);
  SendPointCounts(changed);
  SendName();
  SendUndoRedoState();
}

void RVizCloudAnnotation::onRedo(const std_msgs::Empty &)
{
  if (!m_undo_redo.IsRedoEnabled())
    return;

  ROS_INFO("rviz_cloud_annotation: Redo.");
  const Uint64Vector changed = m_undo_redo.Redo();
  SendControlPointsMarker(changed,true);
  SendPointCounts(changed);
  SendName();
  SendUndoRedoState();
}

void RVizCloudAnnotation::onClear(const std_msgs::UInt32 & label_msg)
{
  const uint64 old_max_label = m_annotation->GetNextLabel();

  const uint64 clear_label = label_msg.data;
  if (clear_label >= old_max_label)
    return;

  if (clear_label != 0)
  {
    const Uint64Vector changed = m_undo_redo.ClearLabel(clear_label);
    SendControlPointsMarker(changed,true);
    SendPointCounts(changed);
    SendName();
    SendUndoRedoState();
    return;
  }

  const Uint64Vector changed = m_undo_redo.Clear();
  SendPointCounts(changed);
  SendControlPointsMarker(changed,true);
  SendName();
  SendUndoRedoState();
}

void RVizCloudAnnotation::SendControlPointMaxWeight()
{
  std_msgs::UInt32 msg;
  msg.data = m_control_point_max_weight;
  m_control_point_weight_max_weight_pub.publish(msg);
}

void RVizCloudAnnotation::onControlPointWeightChange(const std_msgs::UInt32 & msg)
{
  const uint32 new_weight = msg.data;
  if (new_weight > m_control_point_max_weight)
  {
    ROS_ERROR("rviz_cloud_annotation: could not set weight to %u, maximum is %u",(unsigned int)(new_weight),
      (unsigned int)(m_control_point_weight_step));
    return;
  }

  m_control_point_weight_step = new_weight;
  ROS_INFO("rviz_cloud_annotation: control point weight is now: %u",(unsigned int)(m_control_point_weight_step));
}

void RVizCloudAnnotation::onPointSizeChange(const std_msgs::Int32 & msg)
{
  switch (msg.data)
  {
    case POINT_SIZE_CHANGE_BIGGER:
      if (m_point_size_multiplier > 1e5)
        return;
      m_point_size_multiplier *= (1.0 + m_point_size_change_multiplier);
      break;
    case POINT_SIZE_CHANGE_SMALLER:
      if (m_point_size_multiplier < 1e-5)
        return;
      m_point_size_multiplier /= (1.0 + m_point_size_change_multiplier);
      break;
    case POINT_SIZE_CHANGE_RESET:
      if (m_point_size_multiplier == 1.0)
        return;
      m_point_size_multiplier = 1.0;
      break;
    default:
      return;
  }

  ROS_INFO("rviz_cloud_annotation: point size multiplier is now: %f",float(m_point_size_multiplier));
  SendCloudMarker(false);
  SendControlPointsMarker(RangeUint64(1,m_annotation->GetNextLabel()),true);
}

void RVizCloudAnnotation::onGotoFirstUnused(const std_msgs::Empty &)
{
  ROS_INFO("rviz_cloud_annotation: go to first unused label.");
  uint64 i;
  for (i = 1; (m_annotation->GetLabelPointCount(i) != 0) && (i < UINT64_MAX); i++) {}
  SetCurrentLabel(i);
}

void RVizCloudAnnotation::onGotoLastUnused(const std_msgs::Empty &)
{
  ROS_INFO("rviz_cloud_annotation: go to last unused label.");
  uint64 i = m_annotation->GetMaxLabel() + 1;
  for (; (m_annotation->GetLabelPointCount(i) == 0) && (i > 0); i--) {}
  SetCurrentLabel(i + 1);
}

void RVizCloudAnnotation::onGotoFirst(const std_msgs::Empty &)
{
  ROS_INFO("rviz_cloud_annotation: go to first label.");
  SetCurrentLabel(1);
}

void RVizCloudAnnotation::onGotoNextUnused(const std_msgs::Empty &)
{
  ROS_INFO("rviz_cloud_annotation: go to next unused label.");
  uint64 i = m_current_label + 1;
  for (; (m_annotation->GetLabelPointCount(i) != 0) && (i < UINT64_MAX); i++) {}
  SetCurrentLabel(i);
}

void RVizCloudAnnotation::ClearControlPointsMarker(const Uint64Vector & indices,const bool apply)
{
  const uint64 changed_size = indices.size();
  const ControlPointDataVector control_points_empty;
  const Uint64Vector labels_empty;
  for (uint64 i = 0; i < changed_size; i++)
  {
    const uint64 label = indices[i];
    m_interactive_marker_server->insert(
      ControlPointsToMarker(*m_cloud,control_points_empty,label,(m_edit_mode != EDIT_MODE_NONE)),
      boost::bind(&RVizCloudAnnotation::onClickOnCloud,this,_1));
    m_interactive_marker_server->insert(
      LabelsToMarker(*m_cloud,labels_empty,label,(m_edit_mode != EDIT_MODE_NONE)),
      boost::bind(&RVizCloudAnnotation::onClickOnCloud,this,_1));
  }

  m_interactive_marker_server->insert(
    LabelsToMarker(*m_cloud,labels_empty,0,(m_edit_mode != EDIT_MODE_NONE)),
    boost::bind(&RVizCloudAnnotation::onClickOnCloud,this,_1));

  if (apply)
    m_interactive_marker_server->applyChanges();
}

void RVizCloudAnnotation::SendControlPointsMarker(const Uint64Vector & changed_labels,const bool apply)
{
  const uint64 changed_size = changed_labels.size();
  for (uint64 i = 0; i < changed_size; i++)
  {
    const uint64 label = changed_labels[i];
    const bool isabove = label >= m_annotation->GetNextLabel();

    const ControlPointDataVector control_points = isabove ? ControlPointDataVector() : m_annotation->GetControlPointList(label);
    m_interactive_marker_server->insert(
      ControlPointsToMarker(*m_cloud,control_points,label,(m_edit_mode != EDIT_MODE_NONE)),
      boost::bind(&RVizCloudAnnotation::onClickOnCloud,this,_1));

    const Uint64Vector label_points = isabove ? Uint64Vector() : m_annotation->GetLabelPointList(label);
    m_interactive_marker_server->insert(
      LabelsToMarker(*m_cloud,label_points,label,(m_edit_mode != EDIT_MODE_NONE)),
      boost::bind(&RVizCloudAnnotation::onClickOnCloud,this,_1));
  }

  const Uint64Vector label_points = m_annotation->GetLabelPointList(0);
  m_interactive_marker_server->insert(
    LabelsToMarker(*m_cloud,label_points,0,(m_edit_mode != EDIT_MODE_NONE)),
    boost::bind(&RVizCloudAnnotation::onClickOnCloud,this,_1));

  if (apply)
    m_interactive_marker_server->applyChanges();
}

void RVizCloudAnnotation::onClickOnCloud(const InteractiveMarkerFeedbackConstPtr & feedback_ptr)
{
  if (m_edit_mode == EDIT_MODE_NONE)
  {
    ROS_WARN("rviz_cloud_annotation: received stray click while not in edit mode.");
    return;
  }

  const InteractiveMarkerFeedback & feedback = *feedback_ptr;
  uint8 type = feedback.event_type;

  if (type != InteractiveMarkerFeedback::BUTTON_CLICK)
    return; // not a click

  ROS_INFO("rviz_cloud_annotation: click event (marker: %s).",feedback_ptr->marker_name.c_str());

  if (!feedback.mouse_point_valid)
    return; // invalid point

  bool ok;
  const uint64 idx = GetClickedPointId(feedback,ok);

  if (!ok)
    return;

  if (m_edit_mode == EDIT_MODE_CONTROL_POINT)
  {
    ROS_INFO("rviz_cloud_annotation: setting label %u to point %u",(unsigned int)(m_current_label),(unsigned int)(idx));
    const Uint64Vector changed_labels = m_undo_redo.SetControlPoint(idx,m_control_point_weight_step,m_current_label);
    SendControlPointsMarker(changed_labels,true);
    SendPointCounts(changed_labels);
    SendUndoRedoState();
  }
  else if (m_edit_mode == EDIT_MODE_ERASER)
  {
    ROS_INFO("rviz_cloud_annotation: eraser: erasing label from point %u",(unsigned int)(idx));
    const Uint64Vector changed_labels = m_undo_redo.SetControlPoint(idx,0,0);
    SendControlPointsMarker(changed_labels,true);
    SendPointCounts(changed_labels);
    SendUndoRedoState();
  }
  else if (m_edit_mode == EDIT_MODE_COLOR_PICKER)
  {
    const uint64 label = m_annotation->GetLabelForPoint(idx);
    if (label ==  0)
      ROS_WARN("rviz_cloud_annotation: color picker: point %u has no label yet.",uint(idx));
    else
    {
      SetCurrentLabel(label);
      SetEditMode(EDIT_MODE_CONTROL_POINT);
    }
  }
}

void RVizCloudAnnotation::SendCloudMarker(const bool apply)
{
  m_interactive_marker_server->insert(
    CloudToMarker(*m_cloud,(m_edit_mode != EDIT_MODE_NONE)),
    boost::bind(&RVizCloudAnnotation::onClickOnCloud,this,_1));

  if (apply)
    m_interactive_marker_server->applyChanges();
}

void RVizCloudAnnotation::SetCurrentLabel(const uint64 label)
{
  if (m_current_label == label)
    return;

  m_current_label = label;
  ROS_INFO("rviz_cloud_annotation: label is now: %u",(unsigned int)(m_current_label));
  SendName();
  SendUndoRedoState();

  std_msgs::UInt32 msg;
  msg.data = label;
  m_set_current_label_pub.publish(msg);
}

void RVizCloudAnnotation::onRectangleSelectionViewport(const rviz_cloud_annotation::RectangleSelectionViewport & msg)
{
  ROS_INFO("rviz_cloud_annotation: rectangle selection event received.");
  const bool is_deep_selection = msg.is_deep_selection;

  Eigen::Matrix4f projection_matrix;
  for (uint32 y = 0; y < 4; y++)
    for (uint32 x = 0; x < 4; x++)
      projection_matrix(y,x) = msg.projection_matrix[x + y * 4];

  const uint32 start_x = msg.start_x;
  const uint32 start_y = msg.start_y;
  const uint32 end_x = msg.end_x;
  const uint32 end_y = msg.end_y;
  const uint32 viewport_height = msg.viewport_height;
  const uint32 viewport_width = msg.viewport_width;
  const uint32 width = end_x - start_x;
  const uint32 height = end_y - start_y;

  const float focal_length = msg.focal_length;
  const float point_size = m_point_size_multiplier * m_point_size;

  Eigen::Affine3f camera_pose;
  {
    Eigen::Affine3d camera_pose_d;
    tf::poseMsgToEigen(msg.camera_pose,camera_pose_d);
    camera_pose = camera_pose_d.cast<float>();
  }
  const Eigen::Affine3f camera_pose_inv = camera_pose.inverse();

  Eigen::Affine3f scale_matrix = Eigen::Affine3f::Identity();
  scale_matrix(0,0) = viewport_width / 2.0;
  scale_matrix(1,1) = viewport_height / -2.0; // y axis must be inverted
  scale_matrix(1,3) = viewport_height; // y is now negative, so move back to positive

  Eigen::Affine3f translation_matrix = Eigen::Affine3f::Identity();
  translation_matrix.translation().x() = 1.0;
  translation_matrix.translation().y() = 1.0;

  const Eigen::Matrix4f prod_matrix = (scale_matrix *
                                       translation_matrix *
                                       projection_matrix *
                                       camera_pose_inv).matrix();

  Int32PolyTriangleVector tri_cond;
  if (!msg.polyline_x.empty())
  {
    if (msg.polyline_x.size() < 3 || msg.polyline_x.size() != msg.polyline_y.size())
    {
      ROS_WARN("rviz_cloud_annotation: rectangle selection received invalid polyline: ignored.");
      return;
    }

    for (uint64 i = 2; i < msg.polyline_x.size(); i++)
    {
      Int32PolyTriangle tri(msg.polyline_x[0],msg.polyline_y[0],
                        msg.polyline_x[i - 1],msg.polyline_y[i - 1],
                        msg.polyline_x[i],msg.polyline_y[i]);
      tri_cond.push_back(tri);
    }
  }

  const Uint64Vector ids = RectangleSelectionToIds(prod_matrix,camera_pose_inv,*m_cloud,
                                                   start_x,start_y,width,height,
                                                   tri_cond,
                                                   point_size,focal_length,
                                                   is_deep_selection);
  ROS_INFO("rviz_cloud_annotation: rectangle selection selected %d points.",int(ids.size()));
  VectorSelection(ids);
}

RVizCloudAnnotation::int32 RVizCloudAnnotation::Int32PolyTriangle::Contains(const int32 px,const int32 py) const
{
  bool clockwise = false;

  for (uint64 i = 0; i < 3; i++)
  {
    const uint64 i1 = (i + 1) % 3;
    const uint64 i2 = (i + 2) % 3;
    const Eigen::Vector2i normal = Eigen::Vector2i(-y[i1] + y[i],x[i1] - x[i]);
    if (normal == Eigen::Vector2i::Zero())
      return 1; // degenerate

    const int32 dot1 = Eigen::Vector2i(x[i2] - x[i],y[i2] - y[i]).dot(normal);
    if (i == 0)
      clockwise = (dot1 < 0);

    const int32 dotp1 = Eigen::Vector2i(px - x[i],py - y[i]).dot(normal);
    if (!dot1)
      return 1; // degenerate
    if (!dotp1)
    {
      if (i != 1 && (clockwise == (i == 0)))
        return 1; // point on first edge is outside
    }

    if ((dotp1 > 0) != (dot1 > 0))
      return 1; // outside the triangle
  }

  return -1;
}

RVizCloudAnnotation::Uint64Vector RVizCloudAnnotation::RectangleSelectionToIds(const Eigen::Matrix4f prod_matrix,
                                                                               const Eigen::Affine3f camera_pose_inv,
                                                                               const PointXYZRGBNormalCloud & cloud,
                                                                               const uint32 start_x,
                                                                               const uint32 start_y,
                                                                               const uint32 width,
                                                                               const uint32 height,
                                                                               const Int32PolyTriangleVector tri_cond,
                                                                               const float point_size,
                                                                               const float focal_length,
                                                                               const bool is_deep_selection)
{
  Uint64Vector virtual_id_image(width * height,0);
  FloatVector virtual_depth_image(width * height,0.0);

  BoolVector virtual_image_mask(width * height,tri_cond.empty()); // if empty, all is true
  if (!tri_cond.empty())
  {
    for (uint32 y = 0; y < height; y++)
      for (uint32 x = 0; x < width; x++)
      {
        uint32 found = 0;
        for (uint64 i = 0; i < tri_cond.size(); i++)
        {
          const Int32PolyTriangle & tri = tri_cond[i];
          const int32 contains = tri.Contains(x + start_x,y + start_y);
          if (contains <= 0)
            found += 1; // inside
        }

        virtual_image_mask[y * width + x] = (found % 2);
      }
  }

  const uint64 cloud_size = cloud.size();
  BoolVector selected_points(cloud_size,false);
  for (uint64 i = 0; i < cloud_size; i++)
  {
    const PointXYZRGBNormal & ppt = cloud[i];
    const Eigen::Vector3f ept(ppt.x,ppt.y,ppt.z);
    const Eigen::Vector4f thpt = prod_matrix * ept.homogeneous();
    if (thpt.w() < 1e-5)
      continue;
    const Eigen::Vector3f tpt = thpt.head<3>() / thpt.w();
    const Eigen::Vector2i itpt = tpt.head<2>().cast<int>() - Eigen::Vector2i(start_x,start_y);
    const float depth = -(camera_pose_inv * ept).z();
    if (depth < 0.0)
      continue; // behind the observer

    if (is_deep_selection)
    {
      if (itpt.x() < 0 || itpt.y() < 0 || itpt.x() >= int(width) || itpt.y() >= int(height))
        continue;
      if (!virtual_image_mask[itpt.x() + itpt.y() * width])
        continue;
      selected_points[i] = true;
      continue;
    }

    // if not deep, then we must compute point size for occlusions
    const float size_px = (point_size / 4.0) * focal_length / depth;
    const int32 window_px = std::max<int32>(0,size_px);
    if (itpt.x() < -window_px || itpt.y() < -window_px ||
        itpt.x() >= int(width) + window_px || itpt.y() >= int(height) + window_px)
      continue;

    if (window_px == 0)
    {
      if (itpt.x() < 0 || itpt.y() < 0 || itpt.x() >= int(width) || itpt.y() >= int(height))
        continue;
      const uint64 di = itpt.x() + itpt.y() * width;
      if (!virtual_image_mask[di])
        continue;

      if (virtual_id_image[di] == 0 || virtual_depth_image[di] > depth)
      {
        virtual_id_image[di] = i + 1;
        virtual_depth_image[di] = depth;
      }
      continue;
    }

    for (int32 dy = -window_px; dy <= window_px; dy++)
      for (int32 dx = -window_px; dx <= window_px; dx++)
      {
        const Eigen::Vector2i ditpt = itpt + Eigen::Vector2i(dx,dy);
        if (ditpt.x() < 0 || ditpt.y() < 0 || ditpt.x() >= int(width) || ditpt.y() >= int(height))
          continue;
        const uint64 di = ditpt.x() + ditpt.y() * width;
        if (!virtual_image_mask[di])
          continue;

        if (virtual_id_image[di] == 0 || virtual_depth_image[di] > depth)
        {
          virtual_id_image[di] = i + 1;
          virtual_depth_image[di] = depth;
        }
      }
  }

  if (!is_deep_selection)
  {
    for (uint64 i = 0; i < virtual_id_image.size(); i++)
      if (virtual_id_image[i])
        selected_points[virtual_id_image[i] - 1] = true;
  }

  Uint64Vector ids;
  for (uint64 i = 0; i < cloud_size; i++)
    if (selected_points[i])
      ids.push_back(i);
  return ids;
}

void RVizCloudAnnotation::VectorSelection(const Uint64Vector & ids)
{
  if (m_edit_mode == EDIT_MODE_CONTROL_POINT)
  {
    const Uint64Vector changed_labels = m_undo_redo.SetControlPointVector(ids,0,m_current_label);
    SendControlPointsMarker(changed_labels,true);
    SendPointCounts(changed_labels);
    SendUndoRedoState();
    ROS_INFO("rviz_cloud_annotation: selection set %d points.",int(ids.size()));
  }
  else if (m_edit_mode == EDIT_MODE_ERASER)
  {
    const Uint64Vector changed_labels = m_undo_redo.SetControlPointVector(ids,0,0);
    SendControlPointsMarker(changed_labels,true);
    SendPointCounts(changed_labels);
    SendUndoRedoState();
    ROS_INFO("rviz_cloud_annotation: selection cleared %d points.",int(ids.size()));
  }
  else
  {
    ROS_WARN("rviz_cloud_annotation: invalid action %d for selection.",int(m_edit_mode));
  }
}

std::string RVizCloudAnnotation::AppendTimestampBeforeExtension(const std::string & filename)
{
  std::string datetime = boost::posix_time::to_simple_string(boost::posix_time::second_clock::local_time());
  std::replace(datetime.begin(),datetime.end(),' ','_');
  std::replace(datetime.begin(),datetime.end(),':','-');

  const std::size_t last_dot = filename.rfind('.');
  const std::size_t last_slash = filename.rfind("/");
  if (last_dot == std::string::npos)
    return filename + datetime; // no dot
  if (last_slash != std::string::npos && last_slash > last_dot)
    return filename + datetime; // the dot is in a directory

  return filename.substr(0,last_dot) + datetime + filename.substr(last_dot);
}
