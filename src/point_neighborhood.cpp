#include "point_neighborhood.h"

#include <Eigen/Dense>
#include <Eigen/StdVector>

PointNeighborhood::PointNeighborhood(PointXYZRGBNormalCloud::ConstPtr cloudptr,const Conf & conf)
{
  const PointXYZRGBNormalCloud & cloud = *cloudptr;
  const uint64 cloud_size = cloud.size();

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
  return 1.0;
}

