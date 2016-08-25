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

  RVizCloudAnnotation(ros::NodeHandle & nh): m_nh(nh)
  {
    std::string param_string;
    double param_double;

    m_nh.param<std::string>(PARAM_NAME_UPDATE_TOPIC,param_string,PARAM_DEFAULT_UPDATE_TOPIC);
    m_interactive_marker_server = InteractiveMarkerServerPtr(new InteractiveMarkerServer(param_string));

    m_nh.param<std::string>(PARAM_NAME_CLOUD_FILENAME,param_string,PARAM_DEFAULT_CLOUD_FILENAME);
    m_cloud = PointXYZRGBNormalCloud::Ptr(new PointXYZRGBNormalCloud);
    LoadCloud(param_string,*m_cloud);
    m_kdtree = KdTree::Ptr(new KdTree);
    m_kdtree->setInputCloud(m_cloud);

    {
      PointNeighborhood::Conf conf;

      m_nh.param<double>(PARAM_NAME_NEIGH_SEARCH_DISTANCE,param_double,PARAM_DEFAULT_NEIGH_SEARCH_DISTANCE);
      conf.search_distance = param_double;

      m_nh.param<double>(PARAM_NAME_COLOR_IMPORTANCE,param_double,PARAM_DEFAULT_COLOR_IMPORTANCE);
      conf.color_importance = param_double;

      m_nh.param<double>(PARAM_NAME_NORMAL_IMPORTANCE,param_double,PARAM_DEFAULT_NORMAL_IMPORTANCE);
      conf.normal_importance = param_double;

      m_nh.param<double>(PARAM_NAME_POSITION_IMPORTANCE,param_double,PARAM_DEFAULT_POSITION_IMPORTANCE);
      conf.position_importance = param_double;

      m_nh.param<double>(PARAM_NAME_MAX_DISTANCE,param_double,PARAM_DEFAULT_MAX_DISTANCE);
      conf.max_distance = param_double;

      ROS_INFO("Preparing point neighborhood...");
      m_point_neighborhood = PointNeighborhood::ConstPtr(new PointNeighborhood(m_cloud,conf));
      ROS_INFO("done.");
    }

    m_annotation = RVizCloudAnnotationPoints::Ptr(new RVizCloudAnnotationPoints(
      m_cloud->size(),m_point_neighborhood));

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

    m_nh.param<std::string>(PARAM_NAME_ANNOTATED_CLOUD,m_ann_cloud_filename_out,PARAM_DEFAULT_ANNOTATED_CLOUD);
    m_nh.param<std::string>(PARAM_NAME_ANN_FILENAME_IN,m_annotation_filename_in,PARAM_DEFAULT_ANN_FILENAME_IN);
    m_nh.param<std::string>(PARAM_NAME_ANN_FILENAME_OUT,m_annotation_filename_out,PARAM_DEFAULT_ANN_FILENAME_OUT);

    m_nh.param<std::string>(PARAM_NAME_SET_NAME_TOPIC,param_string,PARAM_DEFAULT_SET_NAME_TOPIC);
    m_set_name_sub = m_nh.subscribe(param_string,1,&RVizCloudAnnotation::onSetName,this);

    m_nh.param<std::string>(PARAM_NAME_SET_NAME_TOPIC2,param_string,PARAM_DEFAULT_SET_NAME_TOPIC2);
    m_set_name_pub = m_nh.advertise<std_msgs::String>(param_string,1,true);

    m_current_label = 1;
    m_edit_mode = EDIT_MODE_NONE;

    SendCloudMarker(true);
    Restore(m_annotation_filename_in);
  }

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

  void onSave(const std_msgs::String & filename_msg)
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
      return;
    }

    ROS_INFO("rviz_cloud_annotation: file saved.");

    ROS_INFO("rviz_cloud_annotation: saving cloud %s",m_ann_cloud_filename_out.c_str());
    {
      PointXYZRGBLCloud cloud_out;
      pcl::copyPointCloud(*m_cloud,cloud_out);
      m_annotation->LabelCloudWithColor(cloud_out);
      if (pcl::io::savePCDFileBinary(m_ann_cloud_filename_out,cloud_out))
      {
        ROS_ERROR("rviz_cloud_annotation: could not save labeled cloud.");
        return;
      }
    }
    ROS_INFO("rviz_cloud_annotation: done.");
  }

  void onRestore(const std_msgs::String & filename_msg)
  {
    std::string filename = filename_msg.data.empty() ? m_annotation_filename_out : filename_msg.data;
    Restore(filename);
  }

  void Restore(const std::string & filename);

  void onClear(const std_msgs::UInt32 & /*label_msg*/)
  {
    ClearControlPointsMarker(RangeUint64(1,m_annotation->GetMaxLabel()),false);
    m_annotation->Clear();
    SendControlPointsMarker(RangeUint64(1,m_annotation->GetMaxLabel()),true);
  }

  void onClickOnCloud(const InteractiveMarkerFeedbackConstPtr & feedback_ptr)
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

    ROS_INFO("rviz_cloud_annotation: click.");

    if (!feedback.mouse_point_valid)
      return; // invalid point

    PointXYZRGBNormal click_pt;
    click_pt.x = feedback.mouse_point.x;
    click_pt.y = feedback.mouse_point.y;
    click_pt.z = feedback.mouse_point.z;

    ROS_INFO("rviz_cloud_annotation: click at: %f %f %f",float(click_pt.x),float(click_pt.y),float(click_pt.z));

    std::vector<int> idxs(1);
    std::vector<float> dsts(1);

    if (m_kdtree->nearestKSearch(click_pt,1,idxs,dsts) <= 0)
    {
      ROS_WARN("rviz_cloud_annotation: point was clicked, but no nearest cloud point found.");
      return;
    }

    const uint64 idx = idxs[0];
    const float dst = std::sqrt(dsts[0]);

    ROS_INFO("rviz_cloud_annotation: clicked on point: %u (accuracy: %f)",(unsigned int)(idx),float(dst));

    if (m_edit_mode == EDIT_MODE_CONTROL_POINT)
    {
      ROS_INFO("rviz_cloud_annotation: setting label %u to point %u",(unsigned int)(m_current_label),(unsigned int)(idx));
      const Uint64Vector changed_labels = m_annotation->SetControlPoint(idx,m_current_label);
      SendControlPointsMarker(changed_labels,true);
    }
    else if (m_edit_mode == EDIT_MODE_ERASER)
    {
      ROS_INFO("rviz_cloud_annotation: eraser: erasing label from point %u",(unsigned int)(idx));
      const Uint64Vector changed_labels = m_annotation->SetControlPoint(idx,0);
      SendControlPointsMarker(changed_labels,true);
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

  void SetCurrentLabel(const uint64 label)
  {
    if (m_current_label == label)
      return;

    m_current_label = label;
    m_annotation->ExpandControlPointsUntil(m_current_label);
    ROS_INFO("rviz_cloud_annotation: label is now: %u",(unsigned int)(m_current_label));
    SendName();

    std_msgs::UInt32 msg;
    msg.data = label;
    m_set_current_label_pub.publish(msg);
  }

  void SetEditMode(const uint64 new_edit_mode)
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
      SendControlPointsMarker(RangeUint64(1,m_annotation->GetMaxLabel()),true);
    }
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
    ROS_INFO("rviz_cloud_annotation: label %u has now name %s.",uint(m_current_label),msg.data.c_str());
    SendName();
  }

  void SendName()
  {
    std::string name = m_annotation->GetNameForLabel(m_current_label);
    std_msgs::String msg;
    msg.data = name;
    m_set_name_pub.publish(msg);
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

  void SendControlPointsMarker(const Uint64Vector & changed_control_points,const bool apply)
  {
    const uint64 changed_size = changed_control_points.size();
    for (uint64 i = 0; i < changed_size; i++)
    {
      const uint64 label = changed_control_points[i];
      const Uint64Vector & control_points = m_annotation->GetControlPointList(label);
      m_interactive_marker_server->insert(
        ControlPointsToMarker(*m_cloud,control_points,label,(m_edit_mode != EDIT_MODE_NONE)),
        boost::bind(&RVizCloudAnnotation::onClickOnCloud,this,_1));

      const Uint64Vector label_points = m_annotation->GetLabelPointList(label);
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
};

#endif // RVIZ_CLOUD_ANNOTATION_CLASS_H
