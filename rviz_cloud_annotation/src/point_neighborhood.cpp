/*
 * Copyright (c) 2016, Riccardo Monica
 */

#include "point_neighborhood.h"

#include <Eigen/Dense>
#include <Eigen/StdVector>

PointNeighborhood::PointNeighborhood(PointXYZRGBNormalCloud::ConstPtr cloudptr,const Conf & conf)
{
  const PointXYZRGBNormalCloud & cloud = *cloudptr;
  const uint64 cloud_size = cloud.size();

  m_conf = conf;

  m_index.resize(cloud_size);
  Uint64Vector temporary_indices_vector(cloud_size);

  KdTree::Ptr kdtree(new KdTree);
  kdtree->setInputCloud(cloudptr);

  uint64 counter = 0;

  std::vector<int> idxs;
  std::vector<float> dsts;
  for (uint64 i = 0; i < cloud_size; i++)
  {
    const PointXYZRGBNormal & pt = cloud[i];
    kdtree->radiusSearch(pt,conf.search_distance,idxs,dsts);
    const uint64 size = idxs.size();

    m_index[i].size = 0;

    if (size == 0)
      continue;

    const uint64 size_1 = size - 1;

    m_index[i].size = size_1;
    m_neighbors.resize(counter + size_1);
    m_total_dists.resize(counter + size_1);
    m_position_dists.resize(counter + size_1);
    uint64 inc = 0;
    for (uint64 h = 0; h < size; h++)
      if (uint64(idxs[h]) != i) // do not add self
      {
        m_neighbors[counter + inc] = idxs[h];
        m_total_dists[counter + inc] = TotalDistance(cloud[i],cloud[idxs[h]],conf);
        m_position_dists[counter + inc] = std::sqrt(dsts[h]);
        inc++;
      }

    temporary_indices_vector[i] = counter;
    counter += size_1;
  }

  for (uint64 i = 0; i < cloud_size; i++)
  {
    m_index[i].neighbors = &(m_neighbors[temporary_indices_vector[i]]);
    m_index[i].total_dists = &(m_total_dists[temporary_indices_vector[i]]);
    m_index[i].position_dists = &(m_position_dists[temporary_indices_vector[i]]);
  }
}

float PointNeighborhood::TotalDistance(const PointXYZRGBNormal & a,const PointXYZRGBNormal & b,const Conf & conf) const
{
  const Eigen::Vector3f epta(a.x,a.y,a.z);
  const Eigen::Vector3f eptb(b.x,b.y,b.z);
  const Eigen::Vector3f eca(a.r,a.g,a.b);
  const Eigen::Vector3f ecb(b.r,b.g,b.b);
  const Eigen::Vector3f ena(a.normal_x,a.normal_y,a.normal_z);
  const Eigen::Vector3f enb(b.normal_x,b.normal_y,b.normal_z);

  float spatial_dist = (epta - eptb).norm() / conf.max_distance;
  float color_dist = (eca - ecb).norm() / 255.0f;
  float cos_angle_normal = 1.0f - ena.dot(enb);
  return spatial_dist * conf.position_importance + cos_angle_normal * conf.normal_importance + color_dist * conf.color_importance;
}

