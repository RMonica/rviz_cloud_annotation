/*
 * Copyright (c) 2016, Riccardo Monica
 */

#ifndef RVIZ_CLOUD_ANNOTATION_CLASS_H
#define RVIZ_CLOUD_ANNOTATION_CLASS_H

#include "rviz_cloud_annotation.h"
#include "rviz_cloud_annotation_points.h"
#include "point_neighborhood.h"

// STL
#include <stdint.h>
#include <cmath>
#include <string>
#include <fstream>

// ROS
#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt64MultiArray.h>
#include <std_msgs/Bool.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/colors.h>

// boost
#include <boost/lexical_cast.hpp>

class RVizCloudAnnotation
{
  public:
  typedef visualization_msgs::InteractiveMarker InteractiveMarker;
  typedef visualization_msgs::Marker Marker;
  typedef interactive_markers::InteractiveMarkerServer InteractiveMarkerServer;
  typedef boost::shared_ptr<interactive_markers::InteractiveMarkerServer> InteractiveMarkerServerPtr;
  typedef visualization_msgs::InteractiveMarkerFeedback InteractiveMarkerFeedback;
  typedef visualization_msgs::InteractiveMarkerFeedbackPtr InteractiveMarkerFeedbackPtr;
  typedef visualization_msgs::InteractiveMarkerFeedbackConstPtr InteractiveMarkerFeedbackConstPtr;
  typedef pcl::PointXYZRGBNormal PointXYZRGBNormal;
  typedef pcl::PointCloud<PointXYZRGBNormal> PointXYZRGBNormalCloud;
  typedef pcl::PointXYZRGBL PointXYZRGBL;
  typedef pcl::PointCloud<PointXYZRGBL> PointXYZRGBLCloud;
  typedef pcl::KdTreeFLANN<PointXYZRGBNormal> KdTree;

  typedef uint64_t uint64;
  typedef uint32_t uint32;
  typedef int32_t int32;
  typedef uint8_t uint8;
  typedef unsigned int uint;
  typedef std::vector<uint32> Uint32Vector;
  typedef std::vector<uint64> Uint64Vector;
  typedef std::vector<Uint64Vector> Uint64VectorVector;
  typedef std::vector<float> FloatVector;

  RVizCloudAnnotation(ros::NodeHandle & nh);

  void LoadCloud(const std::string & filename,PointXYZRGBNormalCloud & cloud)
  {
    cloud.clear();

    pcl::PCLPointCloud2 cloud2;

    if (pcl::io::loadPCDFile(filename,cloud2))
    {
      ROS_ERROR("rviz_cloud_annotation: could not load cloud: %s",filename.c_str());
      return;
    }

    pcl::fromPCLPointCloud2(cloud2,cloud);
  }

  void onSave(const std_msgs::String & filename_msg);

  void onRestore(const std_msgs::String & filename_msg)
  {
    std::string filename = filename_msg.data.empty() ? m_annotation_filename_out : filename_msg.data;
    Restore(filename);
  }

  void Restore(const std::string & filename);

  void onClear(const std_msgs::UInt32 & label_msg)
  {
    const uint64 old_max_label = m_annotation->GetNextLabel();

    const uint64 clear_label = label_msg.data;
    if (clear_label >= old_max_label)
      return;

    if (clear_label != 0)
    {
      const Uint64Vector changed = m_annotation->ClearLabel(clear_label);
      SendControlPointsMarker(changed,true);
      SendPointCounts(changed);
      return;
    }

    const Uint64Vector changed = m_annotation->Clear();
    SendPointCounts(changed);
    SendControlPointsMarker(changed,true);
  }

  void onClickOnCloud(const InteractiveMarkerFeedbackConstPtr & feedback_ptr);
  std::string GetClickType(const std::string & marker_name,uint64 & label_out) const;
  uint64 GetClickedPointId(const InteractiveMarkerFeedback & click_feedback,bool & ok);

  void SetCurrentLabel(const uint64 label)
  {
    if (m_current_label == label)
      return;

    m_current_label = label;
    ROS_INFO("rviz_cloud_annotation: label is now: %u",(unsigned int)(m_current_label));
    SendName();

    std_msgs::UInt32 msg;
    msg.data = label;
    m_set_current_label_pub.publish(msg);
  }

  void SetEditMode(const uint64 new_edit_mode);

  void onViewLabels(const std_msgs::Bool & msg)
  {
    m_view_labels = msg.data;

    SendCloudMarker(false);
    SendControlPointsMarker(RangeUint64(1,m_annotation->GetNextLabel()),true);
  }

  void onViewControlPoints(const std_msgs::Bool & msg)
  {
    m_view_control_points = msg.data;

    SendCloudMarker(false);
    SendControlPointsMarker(RangeUint64(1,m_annotation->GetNextLabel()),true);
  }

  void onViewCloud(const std_msgs::Bool & msg)
  {
    m_view_cloud = msg.data;

    SendCloudMarker(false);
    SendControlPointsMarker(RangeUint64(1,m_annotation->GetNextLabel()),true);
  }

  void onSetCurrentLabel(const std_msgs::UInt32 & msg)
  {
    SetCurrentLabel(msg.data);
  }

  void onSetEditMode(const std_msgs::UInt32 & msg)
  {
    SetEditMode(msg.data);
  }

  void onSetName(const std_msgs::String & msg)
  {
    m_annotation->SetNameForLabel(m_current_label,msg.data);
    ROS_INFO("rviz_cloud_annotation: label %u is now named \"%s\".",uint(m_current_label),msg.data.c_str());
    SendName();
  }

  void SendName()
  {
    std::string name = m_annotation->GetNameForLabel(m_current_label);
    std_msgs::String msg;
    msg.data = name;
    m_set_name_pub.publish(msg);
  }

  void SendPointCounts(const Uint64Vector & labels)
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

  Uint64Vector RangeUint64(const uint64 start,const uint64 end) const
  {
    Uint64Vector result(end - start);
    for (uint64 i = start; i < end; i++)
      result[i - start] = i;
    return result;
  }

  void SendCloudMarker(const bool apply)
  {
    m_interactive_marker_server->insert(
      CloudToMarker(*m_cloud,(m_edit_mode != EDIT_MODE_NONE)),
      boost::bind(&RVizCloudAnnotation::onClickOnCloud,this,_1));

    if (apply)
      m_interactive_marker_server->applyChanges();
  }

  void ClearControlPointsMarker(const Uint64Vector & indices,const bool apply)
  {
    const uint64 changed_size = indices.size();
    for (uint64 i = 0; i < changed_size; i++)
    {
      const uint64 label = indices[i];
      const Uint64Vector control_points_empty;
      m_interactive_marker_server->insert(
        ControlPointsToMarker(*m_cloud,control_points_empty,label,(m_edit_mode != EDIT_MODE_NONE)),
        boost::bind(&RVizCloudAnnotation::onClickOnCloud,this,_1));
      m_interactive_marker_server->insert(
        LabelsToMarker(*m_cloud,control_points_empty,label,(m_edit_mode != EDIT_MODE_NONE)),
        boost::bind(&RVizCloudAnnotation::onClickOnCloud,this,_1));
    }

    if (apply)
      m_interactive_marker_server->applyChanges();
  }

  void SendControlPointsMarker(const Uint64Vector & changed_labels,const bool apply)
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

    if (apply)
      m_interactive_marker_server->applyChanges();
  }

  InteractiveMarker ControlPointsToMarker(const PointXYZRGBNormalCloud & cloud,
                                          const Uint64Vector & control_points,
                                          const uint64 label,const bool interactive);
  InteractiveMarker LabelsToMarker(const PointXYZRGBNormalCloud & cloud,
                                   const Uint64Vector & labels,
                                   const uint64 label,const bool interactive);

  InteractiveMarker CloudToMarker(const PointXYZRGBNormalCloud & cloud,const bool interactive);

  private:
  ros::NodeHandle & m_nh;
  InteractiveMarkerServerPtr m_interactive_marker_server;
  PointXYZRGBNormalCloud::Ptr m_cloud;

  RVizCloudAnnotationPoints::Ptr m_annotation;

  KdTree::Ptr m_kdtree;

  ros::Subscriber m_set_edit_mode_sub;
  ros::Subscriber m_set_current_label_sub;

  ros::Publisher m_set_edit_mode_pub;
  ros::Publisher m_set_current_label_pub;

  ros::Subscriber m_set_name_sub;
  ros::Publisher m_set_name_pub;

  ros::Subscriber m_save_sub;
  ros::Subscriber m_restore_sub;
  ros::Subscriber m_clear_sub;

  ros::Subscriber m_view_control_points_sub;
  ros::Subscriber m_view_cloud_sub;
  ros::Subscriber m_view_labels_sub;
  bool m_view_cloud;
  bool m_view_labels;
  bool m_view_control_points;

  ros::Publisher m_point_count_update_pub;

  std::string m_frame_id;
  float m_point_size;
  float m_label_size;
  float m_control_label_size;

  uint64 m_current_label;
  uint64 m_edit_mode;

  PointNeighborhood::ConstPtr m_point_neighborhood;

  std::string m_annotation_filename_in;
  std::string m_annotation_filename_out;
  std::string m_ann_cloud_filename_out;
  std::string m_label_names_filename_out;
};

#endif // RVIZ_CLOUD_ANNOTATION_CLASS_H
