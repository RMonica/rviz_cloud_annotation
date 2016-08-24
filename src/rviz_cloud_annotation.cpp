#include "rviz_cloud_annotation.h"
#include "rviz_cloud_annotation_points.h"

// STL
#include <stdint.h>
#include <cmath>
#include <string>

// ROS
#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <std_msgs/UInt32.h>

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
  typedef pcl::PointXYZL PointXYZL;
  typedef pcl::PointCloud<PointXYZL> PointXYZLCloud;
  typedef pcl::PointXYZI PointXYZI;
  typedef pcl::PointCloud<PointXYZI> PointXYZICloud;
  typedef pcl::KdTreeFLANN<PointXYZRGBNormal> KdTree;

  typedef uint64_t uint64;
  typedef uint32_t uint32;
  typedef int32_t int32;
  typedef uint8_t uint8;
  typedef std::vector<uint32> Uint32Vector;
  typedef std::vector<uint64> Uint64Vector;
  typedef std::vector<Uint64Vector> Uint64VectorVector;
  typedef std::vector<float> FloatVector;

  enum EditMode
  {
    EDIT_MODE_NONE,
    EDIT_MODE_LABELS,
    EDIT_MODE_COLOR_PICKER,
  };

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

    m_annotation = RVizCloudAnnotationPoints::Ptr(new RVizCloudAnnotationPoints(m_cloud->size()));

    m_nh.param<std::string>(PARAM_NAME_FRAME_ID,m_frame_id,PARAM_DEFAULT_FRAME_ID);

    m_nh.param<double>(PARAM_NAME_POINT_SIZE,param_double,PARAM_DEFAULT_POINT_SIZE);
    m_point_size = param_double;

    m_nh.param<double>(PARAM_NAME_LABEL_SIZE,param_double,PARAM_DEFAULT_LABEL_SIZE);
    m_label_size = param_double;

    m_nh.param<double>(PARAM_NAME_CONTROL_LABEL_SIZE,param_double,PARAM_DEFAULT_CONTROL_LABEL_SIZE);
    m_control_label_size = param_double;

    m_nh.param<std::string>(PARAM_NAME_SET_EDIT_MODE_TOPIC,param_string,PARAM_DEFAULT_SET_EDIT_MODE_TOPIC);
    m_set_edit_mode_sub = m_nh.subscribe(param_string,1,&RVizCloudAnnotation::onSetEditMode,this);

    m_nh.param<std::string>(PARAM_NAME_SET_CURRENT_LABEL_TOPIC,param_string,PARAM_DEFAULT_SET_CURRENT_LABEL_TOPIC);
    m_set_current_label_sub = m_nh.subscribe(param_string,1,&RVizCloudAnnotation::onSetCurrentLabel,this);

    m_current_label = 1;
    m_edit_mode = EDIT_MODE_NONE;

    SendCloudMarker(true);
    //m_interactive_marker_server->insert(CloudToCubeMarker(*m_cloud,false),&processFeedback);
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

  void onClickOnCloud(const InteractiveMarkerFeedbackConstPtr & feedback_ptr)
  {
    if (m_edit_mode == EDIT_MODE_NONE)
    {
      ROS_WARN("rviz_cloud_annotation: received spurious click while not in edit mode.");
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
    ROS_INFO("rviz_cloud_annotation: setting label %u to point %u",(unsigned int)(m_current_label),(unsigned int)(idx));

    const uint64 prev_label = m_annotation->SetControlPoint(idx,m_current_label);
    if (prev_label != m_current_label)
    {
      Uint64Vector affected_markers;
      if (prev_label != 0)
        affected_markers.push_back(prev_label);
      if (m_current_label != 0)
        affected_markers.push_back(m_current_label);
      SendControlPointsMarker(affected_markers,true);
    }
  }

  void onSetCurrentLabel(const std_msgs::UInt32 & msg)
  {
    m_current_label = msg.data;
    ROS_INFO("rviz_cloud_annotation: label is now: %u",(unsigned int)(m_current_label));
  }

  void onSetEditMode(const std_msgs::UInt32 & msg)
  {
    const uint64 i = msg.data;
    bool send_cloud = false;
    switch (i)
    {
      case 0:
        if (m_edit_mode != EDIT_MODE_NONE)
          send_cloud = true;
        m_edit_mode = EDIT_MODE_NONE;
        break;
      case 1:
        if (m_edit_mode == EDIT_MODE_NONE)
          send_cloud = true;
        m_edit_mode = EDIT_MODE_LABELS;
        break;
      case 2:
        if (m_edit_mode == EDIT_MODE_NONE)
          send_cloud = true;
        m_edit_mode = EDIT_MODE_COLOR_PICKER;
        break;
      default:
        ROS_ERROR("rviz_cloud_annotation: unsupported edit mode %u received.",(unsigned int)(i));
        break;
    }

    if (send_cloud)
    {
      SendCloudMarker(false);
      SendControlPointsMarker(RangeUint64(1,m_annotation->GetControlPoints().size() + 1),true);
    }
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

  void SendControlPointsMarker(const Uint64Vector & changed_control_points,const bool apply)
  {
    const uint64 changed_size = changed_control_points.size();
    for (uint64 i = 0; i < changed_size; i++)
    {
      const uint64 label = changed_control_points[i];
      const Uint64Vector & control_points = m_annotation->GetControlPoints()[label - 1];
      m_interactive_marker_server->insert(
        ControlPointsToMarker(*m_cloud,control_points,label,(m_edit_mode != EDIT_MODE_NONE)),
        boost::bind(&RVizCloudAnnotation::onClickOnCloud,this,_1));
    }

    if (apply)
      m_interactive_marker_server->applyChanges();
  }

  InteractiveMarker ControlPointsToMarker(const PointXYZRGBNormalCloud & cloud,
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

  InteractiveMarker CloudToMarker(const PointXYZRGBNormalCloud & cloud,const bool interactive)
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

  private:
  ros::NodeHandle & m_nh;
  InteractiveMarkerServerPtr m_interactive_marker_server;
  PointXYZRGBNormalCloud::Ptr m_cloud;

  RVizCloudAnnotationPoints::Ptr m_annotation;

  KdTree::Ptr m_kdtree;

  ros::Subscriber m_set_edit_mode_sub;
  ros::Subscriber m_set_current_label_sub;

  std::string m_frame_id;
  float m_point_size;
  float m_label_size;
  float m_control_label_size;

  uint64 m_current_label;
  EditMode m_edit_mode;
};

int main(int argc, char** argv)
{
  ros::init(argc,argv,"rviz_cloud_annotation");
  ros::NodeHandle nh("~");

  RVizCloudAnnotation rvca(nh);

  ros::spin();
}

