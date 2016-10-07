#ifndef POINT_NEIGHBORHOOD_SEARCH_H
#define POINT_NEIGHBORHOOD_SEARCH_H

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

// STL
#include <stdint.h>
#include <vector>
#include <istream>
#include <ostream>

namespace PointNeighborhoodSearch
{
  typedef uint64_t uint64;
  typedef uint32_t uint32;

  typedef pcl::PointXYZRGBNormal PointXYZRGBNormal;
  typedef pcl::PointCloud<PointXYZRGBNormal> PointXYZRGBNormalCloud;
  typedef pcl::KdTreeFLANN<PointXYZRGBNormal> KdTree;

  typedef std::vector<int> IntVector;
  typedef std::vector<float> FloatVector;

  class Searcher
  {
    public:
    typedef boost::shared_ptr<Searcher> Ptr;
    typedef boost::shared_ptr<const Searcher> ConstPtr;

    virtual void Search(const KdTree & kdtree,const PointXYZRGBNormal & center,
      IntVector & indices,FloatVector & distances) const = 0;
    virtual void Serialize(std::ostream & ofile) const = 0;
    virtual std::string ToString() const = 0;
    virtual uint64 GetId() const = 0;

    virtual bool ApproxEquals(const Searcher & other) const = 0;

    // if true, the searcher can guarantee that the matches are biunivocal
    virtual bool BiunivocalityGuaranteed() const = 0;
  };

  struct ParserException
  {
    ParserException(const std::string m): message(m) {}
    std::string message;
  };

  // from ROS parameter
  Searcher::ConstPtr CreateFromString(const uint64 id,const std::string & param);
  // from binary stream
  Searcher::ConstPtr CreateFromIstream(std::istream & ifile);

  class FixedDistanceSearcher;
  class KNearestNeighborsSearcher;
}

#endif // POINT_NEIGHBORHOOD_SEARCH_H
