/*
 * Copyright (c) 2016, Riccardo Monica
 */

#ifndef RVIZ_CLOUD_ANNOTATION_CLASS_H
#define RVIZ_CLOUD_ANNOTATION_CLASS_H

#include "rviz_cloud_annotation.h"
#include "rviz_cloud_annotation_points.h"
#include "point_neighborhood.h"
#include "rviz_cloud_annotation_undo.h"
#include <rviz_cloud_annotation/UndoRedoState.h>

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
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

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
  typedef pcl::Normal PointNormal;
  typedef pcl::PointCloud<PointNormal> PointNormalCloud;
  typedef pcl::PointXYZRGB PointXYZRGB;
  typedef pcl::PointCloud<PointXYZRGB> PointXYZRGBCloud;
  typedef pcl::PointXYZRGBL PointXYZRGBL;
  typedef pcl::PointCloud<PointXYZRGBL> PointXYZRGBLCloud;
  typedef pcl::KdTreeFLANN<PointXYZRGBNormal> KdTree;
  typedef RVizCloudAnnotationPoints::CPData ControlPointData;
  typedef RVizCloudAnnotationPoints::CPDataVector ControlPointDataVector;

  typedef uint64_t uint64;
  typedef uint32_t uint32;
  typedef int32_t int32;
  typedef uint8_t uint8;
  typedef unsigned int uint;
  typedef std::vector<uint32> Uint32Vector;
  typedef std::vector<uint64> Uint64Vector;
  typedef std::vector<Uint64Vector> Uint64VectorVector;
  typedef std::vector<float> FloatVector;

  enum ControlPointVisual
  {
    CONTROL_POINT_VISUAL_SPHERE,
    CONTROL_POINT_VISUAL_THREE_SPHERES,
    CONTROL_POINT_VISUAL_LINE,
  };

  RVizCloudAnnotation(ros::NodeHandle & nh);

  void LoadCloud(const std::string & filename,const std::string & normal_source,PointXYZRGBNormalCloud & cloud);

  void onSave(const std_msgs::String & filename_msg) { Save(); }
  void onAutosave(const ros::TimerEvent & event) { Save(true); }
  void Save(const bool is_autosave = false);

  std::string AppendTimestampBeforeExtension(const std::string & filename);

  void onRestore(const std_msgs::String & filename_msg)
  {
    std::string filename = filename_msg.data.empty() ? m_annotation_filename_out : filename_msg.data;
    Restore(filename);
  }

  void Restore(const std::string & filename);

  void onClear(const std_msgs::UInt32 & label_msg);

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
    SendUndoRedoState();

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
    m_undo_redo.SetNameForLabel(m_current_label,msg.data);
    ROS_INFO("rviz_cloud_annotation: label %u is now named \"%s\".",uint(m_current_label),msg.data.c_str());
    SendName();
    SendUndoRedoState();
  }

  void onUndo(const std_msgs::Empty &);
  void onRedo(const std_msgs::Empty &);

  void onPointSizeChange(const std_msgs::Int32 & msg);

  void onControlPointWeightChange(const std_msgs::UInt32 & msg);

  void onGotoFirstUnused(const std_msgs::Empty &);
  void onGotoLastUnused(const std_msgs::Empty &);
  void onGotoFirst(const std_msgs::Empty &);
  void onGotoNextUnused(const std_msgs::Empty &);

  void SendName()
  {
    std::string name = m_annotation->GetNameForLabel(m_current_label);
    std_msgs::String msg;
    msg.data = name;
    m_set_name_pub.publish(msg);
  }

  void SendUndoRedoState();

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

  void SendCloudMarker(const bool apply);

  void ClearControlPointsMarker(const Uint64Vector & indices,const bool apply);

  void SendControlPointsMarker(const Uint64Vector & changed_labels,const bool apply);

  void SendControlPointMaxWeight();

  InteractiveMarker ControlPointsToMarker(const PointXYZRGBNormalCloud & cloud,
                                          const ControlPointDataVector & control_points,
                                          const uint64 label, const bool interactive);
  InteractiveMarker LabelsToMarker(const PointXYZRGBNormalCloud & cloud,
                                   const Uint64Vector & labels,
                                   const uint64 label,const bool interactive);

  InteractiveMarker CloudToMarker(const PointXYZRGBNormalCloud & cloud,const bool interactive);

  private:
  ros::NodeHandle & m_nh;
  InteractiveMarkerServerPtr m_interactive_marker_server;
  PointXYZRGBNormalCloud::Ptr m_cloud;

  RVizCloudAnnotationPoints::ConstPtr m_annotation;
  RVizCloudAnnotationUndo m_undo_redo;

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

  ros::Subscriber m_undo_sub;
  ros::Subscriber m_redo_sub;
  ros::Publisher m_undo_redo_state_pub;

  ros::Subscriber m_point_size_change_sub;

  ros::Subscriber m_goto_first_unused_sub;
  ros::Subscriber m_goto_last_unused_sub;
  ros::Subscriber m_goto_first_sub;
  ros::Subscriber m_goto_next_unused_sub;

  ros::Subscriber m_view_control_points_sub;
  ros::Subscriber m_view_cloud_sub;
  ros::Subscriber m_view_labels_sub;
  bool m_view_cloud;
  bool m_view_labels;
  bool m_view_control_points;

  ros::Publisher m_point_count_update_pub;

  ros::Subscriber m_control_points_weight_sub;
  ros::Publisher m_control_point_weight_max_weight_pub;
  uint32 m_control_point_weight_step;
  uint32 m_control_point_max_weight;

  ros::Timer m_autosave_timer;
  bool m_autosave_append_timestamp;

  std::string m_frame_id;
  float m_point_size;
  float m_label_size;
  float m_control_label_size;

  float m_point_size_multiplier;
  float m_point_size_change_multiplier;

  bool m_show_points_back_labels;
  float m_cp_weight_scale_fraction;
  ControlPointVisual m_control_points_visual;

  uint64 m_current_label;
  uint64 m_edit_mode;

  PointNeighborhood::ConstPtr m_point_neighborhood;

  std::string m_annotation_filename_in;
  std::string m_annotation_filename_out;
  std::string m_ann_cloud_filename_out;
  std::string m_label_names_filename_out;
};

#endif // RVIZ_CLOUD_ANNOTATION_CLASS_H
