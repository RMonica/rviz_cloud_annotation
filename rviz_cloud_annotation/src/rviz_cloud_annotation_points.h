/*
 * Copyright (c) 2016, Riccardo Monica
 */

#ifndef RVIZ_CLOUD_ANNOTATION_POINTS_H
#define RVIZ_CLOUD_ANNOTATION_POINTS_H

// STL
#include <stdint.h>
#include <vector>
#include <string>
#include <istream>
#include <ostream>
#include <queue>
#include <set>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/common/colors.h>
#include <pcl/point_types.h>

// boost
#include <boost/shared_ptr.hpp>

#include "point_neighborhood.h"

class RVizCloudAnnotationPoints
{
  public:
  typedef uint64_t uint64;
  typedef uint32_t uint32;
  typedef uint8_t uint8;
  typedef std::vector<uint64> Uint64Vector;
  typedef std::vector<Uint64Vector> Uint64VectorVector;
  typedef std::vector<uint32> Uint32Vector;
  typedef std::vector<uint8> Uint8Vector;
  typedef std::vector<float> FloatVector;
  typedef std::queue<uint64> Uint64Queue;
  typedef std::vector<bool> BoolVector;
  typedef std::vector<std::string> StringVector;
  typedef std::set<uint64> Uint64Set;

  typedef boost::shared_ptr<const RVizCloudAnnotationPoints> ConstPtr;
  typedef boost::shared_ptr<RVizCloudAnnotationPoints> Ptr;

  explicit RVizCloudAnnotationPoints(const uint64 cloud_size,const PointNeighborhood::ConstPtr neighborhood);

  struct IOE // IO exception
  {
    IOE(const std::string & d): description(d) {}
    std::string description;
  };

  static RVizCloudAnnotationPoints::Ptr Deserialize(std::istream & ifile,
    PointNeighborhood::ConstPtr neighborhood);
  void Serialize(std::ostream & ofile) const;

  // returns the list of affected labels
  Uint64Vector SetControlPoint(const uint64 point_id,const uint64 label);
  Uint64Vector SetControlPointList(const Uint64Vector & point_ids,const uint64 label);
  Uint64Vector SetControlPointList(const Uint64Vector & point_ids,const Uint64Vector & labels);

  Uint64Vector Clear();
  Uint64Vector ClearLabel(const uint64 label); // clear label, returns list of affected labels
  Uint64Vector SetNameForLabel(const uint64 label,const std::string & name);

  Uint64Vector GetControlPointList(const uint64 label) const;
  Uint64Vector GetLabelPointList(const uint64 label) const;

  // 0 if none
  uint64 GetLabelForPoint(const uint64 idx) const {return m_labels_assoc[idx]; }
  uint64 GetControlPointForPoint(const uint64 idx) const {return m_control_points_assoc[idx]; }

  std::string GetNameForLabel(const uint64 label) const
  {
    if (label > GetMaxLabel())
      return "";
    return m_control_point_names[label - 1];
  }

  uint64 GetNextLabel() const {return m_control_points.size() + 1; }
  uint64 GetMaxLabel() const {return m_control_points.size(); }
  uint64 GetCloudSize() const {return m_cloud_size; }

  uint64 GetLabelPointCount(const uint64 label) const
  {
    if (label > GetMaxLabel())
      return 0;
    return m_control_points[label - 1].size();
  }

  template <class PointT>
    void LabelCloud(pcl::PointCloud<PointT> & cloud) const;
  template <class PointT>
    void LabelCloudWithColor(pcl::PointCloud<PointT> & cloud) const;

  private:
  // returns a list of labels affected by the label update
  Uint64Vector UpdateLabels(const Uint64Vector & point_ids,const uint64 prev_label,const uint64 next_label);

  void ExpandControlPointsUntil(const uint64 label);

  void RegenerateControlPointsAssoc();
  void RegenerateLabelAssoc(BoolVector & touched);
  void UpdateLabelAssocAdded(const Uint64Vector & added_indices,const uint32 added_label,BoolVector & touched);
  void UpdateLabelAssocDeleted(const Uint64Vector & removed_indices,const uint32 removed_label,BoolVector & touched);
  void UpdateLabelAssocChanged(const Uint64Vector & changed_indices,const uint32 added_label,
                               const uint32 removed_label,BoolVector & touched);
  static void RunRegionGrowing(const uint64 cloud_size,
                               const Uint64VectorVector & control_points,
                               const PointNeighborhood & point_neighborhood,
                               Uint32Vector & labels_assoc,
                               FloatVector & last_generated_tot_dists,
                               BoolVector & touched);
  static void UpdateRegionGrowing(const uint64 cloud_size,
                                  const PointNeighborhood & point_neighborhood,
                                  const Uint64Vector & seeds,
                                  Uint32Vector & labels_assoc,
                                  FloatVector & last_generated_tot_dists,
                                  BoolVector & touched);

  Uint32Vector m_control_points_assoc;
  Uint32Vector m_labels_assoc;
  // control points for each label
  Uint64VectorVector m_control_points;
  uint64 m_cloud_size;

  FloatVector m_last_generated_tot_dists;

  StringVector m_control_point_names;

  PointNeighborhood::ConstPtr m_point_neighborhood;
};

template <class PointT>
  void RVizCloudAnnotationPoints::LabelCloud(pcl::PointCloud<PointT> & cloud) const
{
  for (uint64 i = 0; i < m_cloud_size; i++)
    cloud[i].label = m_labels_assoc[i];
}

template <class PointT>
  void RVizCloudAnnotationPoints::LabelCloudWithColor(pcl::PointCloud<PointT> & cloud) const
{
  LabelCloud(cloud);
  for (uint64 i = 0; i < m_cloud_size; i++)
  {
    const uint32 label = cloud[i].label;
    if (!label)
    {
      cloud[i].r = 0;
      cloud[i].g = 0;
      cloud[i].b = 0;
      continue;
    }
    const pcl::RGB color = pcl::GlasbeyLUT::at((label - 1) % 256);
    cloud[i].r = color.r;
    cloud[i].g = color.g;
    cloud[i].b = color.b;
  }
}

#endif // RVIZ_CLOUD_ANNOTATION_POINTS_H
