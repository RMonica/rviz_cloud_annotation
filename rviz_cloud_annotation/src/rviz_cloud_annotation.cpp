/*
 * Copyright (c) 2016, Riccardo Monica
 */

#include "rviz_cloud_annotation_class.h"

int main(int argc, char** argv)
{
  ros::init(argc,argv,"rviz_cloud_annotation");
  ros::NodeHandle nh("~");

  RVizCloudAnnotation rvca(nh);

  ros::spin();

  return 0;
}

