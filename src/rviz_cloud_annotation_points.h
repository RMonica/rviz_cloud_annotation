#ifndef RVIZ_CLOUD_ANNOTATION_POINTS_H
#define RVIZ_CLOUD_ANNOTATION_POINTS_H

// STL
#include <stdint.h>
#include <vector>
#include <string>
#include <istream>
#include <ostream>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/common/colors.h>
#include <pcl/point_types.h>

// boost
#include <boost/shared_ptr.hpp>

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

  typedef boost::shared_ptr<const RVizCloudAnnotationPoints> ConstPtr;
  typedef boost::shared_ptr<RVizCloudAnnotationPoints> Ptr;

  explicit RVizCloudAnnotationPoints(const uint64 cloud_size);

  struct IOE // IO exception
  {
    IOE(const std::string & d): description(d) {}
    std::string description;
  };

  static RVizCloudAnnotationPoints::Ptr Deserialize(std::istream & ifile);
  void Serialize(std::ostream & ofile) const;

  // returns the old label
  Uint64Vector SetControlPoint(const uint64 point_id,const uint64 label);

  // returns a list of control points affected by the label update
  Uint64Vector UpdateLabels(const uint64 point_id,const uint64 prev_label,const uint64 next_label);

  //const Uint64VectorVector & GetControlPoints() const {return m_control_points; }
  const Uint64Vector & GetControlPointList(const uint64 label) const {return m_control_points[label - 1]; }
  const Uint64Vector GetLabelPointList(const uint64 label) const;

  uint64 GetMaxLabel() const {return m_control_points.size() + 1; }
  uint64 GetCloudSize() const {return m_cloud_size; }

  template <class PointT>
    void LabelCloud(pcl::PointCloud<PointT> & cloud) const;
  template <class PointT>
    void LabelCloudWithColor(pcl::PointCloud<PointT> & cloud) const;

  void Clear();

  private:
  Uint32Vector m_control_points_assoc;
  Uint32Vector m_labels_assoc;
  // control points for each label
  Uint64VectorVector m_control_points;
  uint64 m_cloud_size;
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
