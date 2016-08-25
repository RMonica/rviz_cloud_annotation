#include "rviz_cloud_annotation_class.h"

RVizCloudAnnotation::InteractiveMarker RVizCloudAnnotation::ControlPointsToMarker(const PointXYZRGBNormalCloud & cloud,
                                        const Uint64Vector & control_points,
                                        const uint64 label,const bool interactive)
{
  const uint64 control_size = control_points.size();

  InteractiveMarker marker;
  marker.header.frame_id = m_frame_id;
  marker.name = std::string("control_points_") + boost::lexical_cast<std::string>(label);
  marker.description = "";

  const pcl::RGB color = pcl::GlasbeyLUT::at((label - 1) % 256);

  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;

  Marker cloud_marker;
  cloud_marker.type = Marker::LINE_LIST;
  cloud_marker.scale.x = m_control_label_size / 2.0;
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

    cloud_marker.points[i * 2 + 1].x = pt.x + pt.normal_x * m_control_label_size;
    cloud_marker.points[i * 2 + 1].y = pt.y + pt.normal_y * m_control_label_size;
    cloud_marker.points[i * 2 + 1].z = pt.z + pt.normal_z * m_control_label_size;
  }

  visualization_msgs::InteractiveMarkerControl points_control;
  points_control.always_visible = true;
  points_control.interaction_mode = interactive ?
    int32(visualization_msgs::InteractiveMarkerControl::BUTTON) :
    int32(visualization_msgs::InteractiveMarkerControl::NONE);
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
  marker.name = std::string("label_points_") + boost::lexical_cast<std::string>(label);
  marker.description = "";

  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;

  const pcl::RGB color = pcl::GlasbeyLUT::at((label - 1) % 256);

  Marker cloud_marker;
  cloud_marker.type = Marker::POINTS;
  cloud_marker.scale.x = m_label_size;
  cloud_marker.scale.y = m_label_size;
  cloud_marker.scale.z = m_label_size;
  cloud_marker.color.r = color.r / 255.0;
  cloud_marker.color.g = color.g / 255.0;
  cloud_marker.color.b = color.b / 255.0;
  cloud_marker.color.a = 1.0;

  cloud_marker.points.resize(labels_size);
  for(uint64 i = 0; i < labels_size; i++)
  {
    const PointXYZRGBNormal & pt = cloud[labels[i]];

    cloud_marker.points[i].x = pt.x + pt.normal_x * m_label_size / 2.0;
    cloud_marker.points[i].y = pt.y + pt.normal_y * m_label_size / 2.0;
    cloud_marker.points[i].z = pt.z + pt.normal_z * m_label_size / 2.0;
  }

  visualization_msgs::InteractiveMarkerControl points_control;
  points_control.always_visible = true;
  points_control.interaction_mode = interactive ?
    int32(visualization_msgs::InteractiveMarkerControl::BUTTON) :
    int32(visualization_msgs::InteractiveMarkerControl::NONE);
  points_control.markers.push_back(cloud_marker);
  marker.controls.push_back(points_control);

  return marker;
}

RVizCloudAnnotation::InteractiveMarker RVizCloudAnnotation::CloudToMarker(const PointXYZRGBNormalCloud & cloud,const bool interactive)
{
  const uint64 cloud_size = cloud.size();

  InteractiveMarker marker;
  marker.header.frame_id = m_frame_id;
  marker.name = "cloud";
  marker.description = "";

  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;

  Marker cloud_marker;
  cloud_marker.type = Marker::POINTS;
  cloud_marker.scale.x = m_point_size;
  cloud_marker.scale.y = m_point_size;
  cloud_marker.scale.z = m_point_size;
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

  m_annotation->ExpandControlPointsUntil(m_current_label);

  ClearControlPointsMarker(RangeUint64(1,m_annotation->GetMaxLabel()),false);
  m_annotation = maybe_new_annotation;
  SendControlPointsMarker(RangeUint64(1,m_annotation->GetMaxLabel()),true);
  SendName();

  ROS_INFO("rviz_cloud_annotation: file loaded.");
}
