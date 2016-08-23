#include "rviz_cloud_annotation.h"

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

namespace vm = visualization_msgs;

void processFeedback( const vm::InteractiveMarkerFeedbackConstPtr &feedback )
{
  uint8_t type = feedback->event_type;

  if( type == vm::InteractiveMarkerFeedback::BUTTON_CLICK ||
      type == vm::InteractiveMarkerFeedback::MOUSE_DOWN ||
      type == vm::InteractiveMarkerFeedback::MOUSE_UP )
  {
    const char* type_str = (type == vm::InteractiveMarkerFeedback::BUTTON_CLICK ? "button click" :
                            (type == vm::InteractiveMarkerFeedback::MOUSE_DOWN ? "mouse down" : "mouse up"));

    if( feedback->mouse_point_valid )
    {
      ROS_INFO( "%s at %f, %f, %f in frame %s",
                type_str,
                feedback->mouse_point.x, feedback->mouse_point.y, feedback->mouse_point.z,
                feedback->header.frame_id.c_str() );
    }
    else
    {
      ROS_INFO( "%s", type_str );
    }
  }
  else if( type == vm::InteractiveMarkerFeedback::POSE_UPDATE )
  {
    ROS_INFO_STREAM( feedback->marker_name << " is now at "
                     << feedback->pose.position.x << ", " << feedback->pose.position.y
                     << ", " << feedback->pose.position.z );
  }
  else
    ROS_INFO("unknown type: %u",(unsigned int)(type));
}

class RVizCloudAnnotation
{
  public:
  typedef visualization_msgs::InteractiveMarker InteractiveMarker;
  typedef visualization_msgs::Marker Marker;
  typedef interactive_markers::InteractiveMarkerServer InteractiveMarkerServer;
  typedef boost::shared_ptr<interactive_markers::InteractiveMarkerServer> InteractiveMarkerServerPtr;
  typedef pcl::PointXYZRGBNormal PointXYZRGBNormal;
  typedef pcl::PointCloud<PointXYZRGBNormal> PointXYZRGBNormalCloud;
  typedef pcl::PointXYZL PointXYZL;
  typedef pcl::PointCloud<PointXYZL> PointXYZLCloud;
  typedef pcl::PointXYZI PointXYZI;
  typedef pcl::PointCloud<PointXYZI> PointXYZICloud;

  typedef uint64_t uint64;
  typedef uint32_t uint32;
  typedef int32_t int32;
  typedef std::vector<uint32> Uint32Vector;
  typedef std::vector<float> FloatVector;

  RVizCloudAnnotation(ros::NodeHandle & nh): m_nh(nh)
  {
    std::string param_string;
    double param_double;

    m_nh.param<std::string>(PARAM_NAME_UPDATE_TOPIC,param_string,PARAM_DEFAULT_UPDATE_TOPIC);
    m_interactive_marker_server = InteractiveMarkerServerPtr(new InteractiveMarkerServer(param_string));

    m_nh.param<std::string>(PARAM_NAME_CLOUD_FILENAME,param_string,PARAM_DEFAULT_CLOUD_FILENAME);
    m_cloud = PointXYZRGBNormalCloud::Ptr(new PointXYZRGBNormalCloud);
    LoadCloud(param_string,*m_cloud,m_labels,m_intensities);

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

    m_interactive_marker_server->insert(CloudToMarker(*m_cloud,false),&processFeedback);
    m_interactive_marker_server->insert(CloudToCubeMarker(*m_cloud,false),&processFeedback);
    m_interactive_marker_server->applyChanges();

    m_current_label = 1;
  }

  void LoadCloud(const std::string & filename,PointXYZRGBNormalCloud & cloud,Uint32Vector & labels,FloatVector & intensities)
  {
    labels.clear();
    cloud.clear();
    intensities.clear();

    pcl::PCLPointCloud2 cloud2;

    if (pcl::io::loadPCDFile(filename,cloud2))
    {
      ROS_ERROR("rviz_cloud_annotation: could not load cloud: %s",filename.c_str());
      return;
    }

    pcl::fromPCLPointCloud2(cloud2,cloud);
    const uint64 cloud_size = cloud.size();
    labels.resize(cloud_size,0);
    intensities.resize(cloud_size,0.0);

    bool has_label = false;
    for (uint64 i = 0; i < cloud2.fields.size(); i++)
      if (cloud2.fields[i].name == "label")
      {
        has_label = true;
        break;
      }

    bool has_intensity = false;
    for (uint64 i = 0; i < cloud2.fields.size(); i++)
      if (cloud2.fields[i].name == "intensity")
      {
        has_intensity = true;
        break;
      }

    if (has_label)
    {
      ROS_INFO("rviz_cloud_annotation: previous labels were found in cloud.");
      PointXYZLCloud labels_cloud;
      pcl::fromPCLPointCloud2(cloud2,labels_cloud);

      for (uint64 i = 0; i < cloud_size; i++)
      {
        labels[i] = labels_cloud[i].label;
        if (labels[i] != 0)
          intensities[i] = 1.0;
      }
    }

    if (has_label && has_intensity)
    {
      ROS_INFO("rviz_cloud_annotation: previous intensities were found in cloud.");
      PointXYZICloud intensity_cloud;
      pcl::fromPCLPointCloud2(cloud2,intensity_cloud);

      for (uint64 i = 0; i < cloud_size; i++)
        if (labels[i] != 0)
          intensities[i] = intensity_cloud[i].intensity;
    }
  }

  void onSetCurrentLabel(const std_msgs::UInt32 & msg)
  {
    m_current_label = msg.data;
    ROS_INFO("rviz_cloud_annotation: label is now: %u",(unsigned int)(m_current_label));
  }

  void onSetEditMode(const std_msgs::UInt32 & msg)
  {
    const bool edit = (msg.data ? true : false);

    m_interactive_marker_server->insert(CloudToMarker(*m_cloud,edit),&processFeedback);
    m_interactive_marker_server->insert(CloudToCubeMarker(*m_cloud,edit),&processFeedback);
    m_interactive_marker_server->applyChanges();
  }

  InteractiveMarker CloudToCubeMarker(const PointXYZRGBNormalCloud & cloud,const bool interactive)
  {
    const uint64 cloud_size = cloud.size();

    InteractiveMarker marker;
    marker.header.frame_id = m_frame_id;
    marker.name = "cubes";
    marker.description = "";

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;

    Marker cloud_marker;
    cloud_marker.type = Marker::LINE_LIST;
    cloud_marker.scale.x = m_point_size / 4.0;
    cloud_marker.scale.y = m_point_size / 4.0;
    cloud_marker.scale.z = m_point_size / 4.0;
    cloud_marker.color.r = 0.0;
    cloud_marker.color.g = 0.0;
    cloud_marker.color.b = 0.0;
    cloud_marker.color.a = 1.0;

    cloud_marker.points.resize(cloud_size * 2);
    for(uint64 i = 0; i < cloud_size; i++)
    {
      const PointXYZRGBNormal & pt = cloud[i];

      cloud_marker.points[i * 2].x = pt.x;
      cloud_marker.points[i * 2].y = pt.y;
      cloud_marker.points[i * 2].z = pt.z;

      cloud_marker.points[i * 2 + 1].x = pt.x + pt.normal_x * 0.01;
      cloud_marker.points[i * 2 + 1].y = pt.y + pt.normal_y * 0.01;
      cloud_marker.points[i * 2 + 1].z = pt.z + pt.normal_z * 0.01;
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
  Uint32Vector m_labels;
  FloatVector m_intensities;

  ros::Subscriber m_set_edit_mode_sub;
  ros::Subscriber m_set_current_label_sub;

  std::string m_frame_id;
  float m_point_size;
  float m_label_size;
  float m_control_label_size;

  uint64 m_current_label;
};

int main(int argc, char** argv)
{
  ros::init(argc,argv,"rviz_cloud_annotation");
  ros::NodeHandle nh("~");

  RVizCloudAnnotation rvca(nh);

  ros::spin();
}

