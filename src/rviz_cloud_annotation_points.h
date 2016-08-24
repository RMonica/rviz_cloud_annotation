#ifndef RVIZ_CLOUD_ANNOTATION_POINTS_H
#define RVIZ_CLOUD_ANNOTATION_POINTS_H

// STL
#include <stdint.h>
#include <vector>
#include <string>
#include <istream>
#include <ostream>

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

  static RVizCloudAnnotationPoints Deserialize(std::istream & ifile);
  void Serialize(std::ostream & ofile) const;

  // returns the old label
  uint64 SetControlPoint(const uint64 point_id,const uint64 label);

  const Uint64VectorVector & GetControlPoints() const {return m_control_points; }

  private:
  Uint32Vector m_control_points_assoc;
  Uint32Vector m_labels_assoc;
  // control points for each label
  Uint64VectorVector m_control_points;
  uint64 m_cloud_size;
};

#endif // RVIZ_CLOUD_ANNOTATION_POINTS_H
