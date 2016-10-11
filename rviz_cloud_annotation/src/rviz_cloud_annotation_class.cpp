/*
 * Copyright (c) 2016, Riccardo Monica
 */

#include "rviz_cloud_annotation_class.h"
#include "point_neighborhood_search.h"

#include <cstring>

#define CLOUD_MARKER_NAME            "cloud"
#define CONTROL_POINT_MARKER_PREFIX  "control_points_"
#define LABEL_POINT_MARKER_PREFIX    "label_points_"

RVizCloudAnnotation::RVizCloudAnnotation(ros::NodeHandle & nh): m_nh(nh)
{
  std::string param_string;
  double param_double;
  int param_int;

  m_nh.param<std::string>(PARAM_NAME_UPDATE_TOPIC,param_string,PARAM_DEFAULT_UPDATE_TOPIC);
  m_interactive_marker_server = InteractiveMarkerServerPtr(new InteractiveMarkerServer(param_string));

  m_nh.param<std::string>(PARAM_NAME_CLOUD_FILENAME,param_string,PARAM_DEFAULT_CLOUD_FILENAME);
  m_cloud = PointXYZRGBNormalCloud::Ptr(new PointXYZRGBNormalCloud);
  LoadCloud(param_string,*m_cloud);
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

    ROS_INFO("rviz_cloud_annotation: building point neighborhood...");
    m_point_neighborhood = PointNeighborhood::ConstPtr(new PointNeighborhood(m_cloud,conf));
    ROS_INFO("rviz_cloud_annotation: done.");
  }

  RVizCloudAnnotationPoints::Ptr default_annotation = RVizCloudAnnotationPoints::Ptr(new RVizCloudAnnotationPoints(
    m_cloud->size(),m_point_neighborhood));
  m_annotation = default_annotation;
  m_undo_redo.SetAnnotation(default_annotation);

  m_nh.param<std::string>(PARAM_NAME_FRAME_ID,m_frame_id,PARAM_DEFAULT_FRAME_ID);

  m_nh.param<double>(PARAM_NAME_POINT_SIZE,param_double,PARAM_DEFAULT_POINT_SIZE);
  m_point_size = param_double;

  m_nh.param<double>(PARAM_NAME_LABEL_SIZE,param_double,PARAM_DEFAULT_LABEL_SIZE);
  m_label_size = param_double;

  m_nh.param<double>(PARAM_NAME_CONTROL_LABEL_SIZE,param_double,PARAM_DEFAULT_CONTROL_LABEL_SIZE);
  m_control_label_size = param_double;

  m_nh.param<std::string>(PARAM_NAME_SAVE_TOPIC,param_string,PARAM_DEFAULT_SAVE_TOPIC);
  m_save_sub = m_nh.subscribe(param_string,1,&RVizCloudAnnotation::onSave,this);

  m_nh.param<std::string>(PARAM_NAME_RESTORE_TOPIC,param_string,PARAM_DEFAULT_RESTORE_TOPIC);
  m_restore_sub = m_nh.subscribe(param_string,1,&RVizCloudAnnotation::onRestore,this);

  m_nh.param<std::string>(PARAM_NAME_CLEAR_TOPIC,param_string,PARAM_DEFAULT_CLEAR_TOPIC);
  m_clear_sub = m_nh.subscribe(param_string,1,&RVizCloudAnnotation::onClear,this);

  m_nh.param<std::string>(PARAM_NAME_SET_EDIT_MODE_TOPIC,param_string,PARAM_DEFAULT_SET_EDIT_MODE_TOPIC);
  m_set_edit_mode_sub = m_nh.subscribe(param_string,1,&RVizCloudAnnotation::onSetEditMode,this);

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

  m_current_label = 1;
  m_edit_mode = EDIT_MODE_NONE;

  m_point_size_multiplier = 1.0;

  m_control_point_weight = 1.0;

  m_view_cloud = m_view_labels = m_view_control_points = true;

  SendCloudMarker(true);
  Restore(m_annotation_filename_in);
}

RVizCloudAnnotation::InteractiveMarker RVizCloudAnnotation::ControlPointsToMarker(const PointXYZRGBNormalCloud & cloud,
                                        const Uint64Vector & control_points,
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

  Marker cloud_marker;
  cloud_marker.type = Marker::LINE_LIST;
  cloud_marker.scale.x = m_point_size_multiplier * m_control_label_size / 2.0;
  cloud_marker.scale.y = 0.0;
  cloud_marker.scale.z = 0.0;
  cloud_marker.color.r = float(color.r) / 255.0;
  cloud_marker.color.g = float(color.g) / 255.0;
  cloud_marker.color.b = float(color.b) / 255.0;
  cloud_marker.color.a = 1.0;

  cloud_marker.points.resize(control_size * 2);
  for(uint64 i = 0; i < control_size; i++)
  {
    const PointXYZRGBNormal & pt = cloud[control_points[i]];

    cloud_marker.points[i * 2].x = pt.x;
    cloud_marker.points[i * 2].y = pt.y;
    cloud_marker.points[i * 2].z = pt.z;

    cloud_marker.points[i * 2 + 1].x = pt.x + pt.normal_x * m_point_size_multiplier * m_control_label_size;
    cloud_marker.points[i * 2 + 1].y = pt.y + pt.normal_y * m_point_size_multiplier * m_control_label_size;
    cloud_marker.points[i * 2 + 1].z = pt.z + pt.normal_z * m_point_size_multiplier * m_control_label_size;
  }

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

    cloud_marker.points[i].x = pt.x + pt.normal_x * normal_mult;
    cloud_marker.points[i].y = pt.y + pt.normal_y * normal_mult;
    cloud_marker.points[i].z = pt.z + pt.normal_z * normal_mult;
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
    maybe_new_annotation = RVizCloudAnnotationPoints::Deserialize(ifile,m_point_neighborhood);
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

void RVizCloudAnnotation::onSave(const std_msgs::String & filename_msg)
{
  std::string filename = filename_msg.data.empty() ? m_annotation_filename_out : filename_msg.data;
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

  ROS_INFO("rviz_cloud_annotation: saving cloud: %s",m_ann_cloud_filename_out.c_str());
  {
    PointXYZRGBLCloud cloud_out;
    pcl::copyPointCloud(*m_cloud,cloud_out);
    m_annotation->LabelCloudWithColor(cloud_out);
    if (pcl::io::savePCDFileBinary(m_ann_cloud_filename_out,cloud_out))
      ROS_ERROR("rviz_cloud_annotation: could not save labeled cloud.");
  }
  ROS_INFO("rviz_cloud_annotation: done.");

  ROS_INFO("rviz_cloud_annotation: saving names: %s",m_label_names_filename_out.c_str());
  {
    std::ofstream ofile(m_label_names_filename_out.c_str());
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

    const Uint64Vector & control_points = m_annotation->GetControlPointList(label);
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
      const PointXYZRGBNormal & pt = (*m_cloud)[control_points[i]];
      const Eigen::Vector3f ept(pt.x,pt.y,pt.z);
      const Eigen::Vector3f en(pt.normal_x,pt.normal_y,pt.normal_z);
      const Eigen::Vector3f shift_pt = ept + en * m_control_label_size / 2.0;
      if (i == 0 || (shift_pt - click_pt).squaredNorm() < nearest_sqr_dist)
      {
        nearest_idx = i;
        nearest_sqr_dist = (shift_pt - click_pt).squaredNorm();
      }
    }

    const uint64 idx = control_points[nearest_idx];

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

void RVizCloudAnnotation::onUndo(const std_msgs::Empty &)
{
  if (!m_undo_redo.IsUndoEnabled())
    return;
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

void RVizCloudAnnotation::onControlPointWeightChange(const std_msgs::Float32 & msg)
{
  m_control_point_weight = std::min<float>(std::max<float>(1.0 - msg.data,0.0),1.0);
  ROS_INFO("rviz_cloud_annotation: control point weight is now: %f",float(1.0 - m_control_point_weight));
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

void RVizCloudAnnotation::ClearControlPointsMarker(const Uint64Vector & indices,const bool apply)
{
  const uint64 changed_size = indices.size();
  const Uint64Vector control_points_empty;
  for (uint64 i = 0; i < changed_size; i++)
  {
    const uint64 label = indices[i];
    m_interactive_marker_server->insert(
      ControlPointsToMarker(*m_cloud,control_points_empty,label,(m_edit_mode != EDIT_MODE_NONE)),
      boost::bind(&RVizCloudAnnotation::onClickOnCloud,this,_1));
    m_interactive_marker_server->insert(
      LabelsToMarker(*m_cloud,control_points_empty,label,(m_edit_mode != EDIT_MODE_NONE)),
      boost::bind(&RVizCloudAnnotation::onClickOnCloud,this,_1));
  }

  m_interactive_marker_server->insert(
    LabelsToMarker(*m_cloud,control_points_empty,0,(m_edit_mode != EDIT_MODE_NONE)),
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

    const Uint64Vector control_points = isabove ? Uint64Vector() : m_annotation->GetControlPointList(label);
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
    const Uint64Vector changed_labels = m_undo_redo.SetControlPoint(idx,m_current_label);
    SendControlPointsMarker(changed_labels,true);
    SendPointCounts(changed_labels);
    SendUndoRedoState();
  }
  else if (m_edit_mode == EDIT_MODE_ERASER)
  {
    ROS_INFO("rviz_cloud_annotation: eraser: erasing label from point %u",(unsigned int)(idx));
    const Uint64Vector changed_labels = m_undo_redo.SetControlPoint(idx,0);
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

