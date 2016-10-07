#include "point_neighborhood_search.h"

#include <cmath>

#include "rviz_cloud_annotation.h"

class PointNeighborhoodSearch::FixedDistanceSearcher: public PointNeighborhoodSearch::Searcher
{
  public:
  FixedDistanceSearcher(const float search_distance): m_search_distance(search_distance) {}

  void Search(const KdTree & kdtree,
              const PointXYZRGBNormal & center,
              IntVector & indices,
              FloatVector & distances) const
  {
    kdtree.radiusSearch(center,m_search_distance,indices,distances);
  }

  void Serialize(std::ostream & ofile) const
  {
    const uint64 id = PARAM_VALUE_NEIGH_SEARCH_FIXED_DISTANCE;
    ofile.write((const char *)&id,sizeof(id));
    ofile.write((const char *)&m_search_distance,sizeof(m_search_distance));
  }

  std::string ToString() const
  {
    return boost::lexical_cast<std::string>(m_search_distance);
  }

  bool ApproxEquals(const Searcher & other) const
  {
    const FixedDistanceSearcher * const fds = dynamic_cast<const FixedDistanceSearcher *>(&other);
    return fds && (std::abs(fds->m_search_distance - m_search_distance) < 1e-5);
  }

  uint64 GetId() const {return PARAM_VALUE_NEIGH_SEARCH_FIXED_DISTANCE; }

  bool BiunivocalityGuaranteed() const {return true; }

  private:
  float m_search_distance;
};

class PointNeighborhoodSearch::KNearestNeighborsSearcher: public PointNeighborhoodSearch::Searcher
{
  public:
  KNearestNeighborsSearcher(const uint32 knn): m_knn(knn) {}

  void Search(const KdTree & kdtree,
              const PointXYZRGBNormal & center,
              IntVector & indices,
              FloatVector & distances) const
  {
    indices.clear();
    distances.clear();
    indices.reserve(m_knn);
    distances.reserve(m_knn);

    if (m_knn == 0)
      return; // nothing to do

    const uint32 n = kdtree.nearestKSearch(center,m_knn,indices,distances);
    indices.resize(n);
    distances.resize(n);
  }

  void Serialize(std::ostream & ofile) const
  {
    const uint64 id = PARAM_VALUE_NEIGH_SEARCH_KNN;
    ofile.write((const char *)&id,sizeof(id));
    ofile.write((const char *)&m_knn,sizeof(m_knn));
  }

  std::string ToString() const
  {
    return boost::lexical_cast<std::string>(m_knn);
  }

  bool ApproxEquals(const Searcher & other) const
  {
    const KNearestNeighborsSearcher * const fds = dynamic_cast<const KNearestNeighborsSearcher *>(&other);
    return fds && (fds->m_knn == m_knn);
  }

  uint64 GetId() const {return PARAM_VALUE_NEIGH_SEARCH_KNN; }

  bool BiunivocalityGuaranteed() const {return false; }

  private:
  uint32 m_knn;
};

PointNeighborhoodSearch::Searcher::ConstPtr PointNeighborhoodSearch::CreateFromString(const uint64 id,const std::string & param)
{
  switch (id)
  {
    case PARAM_VALUE_NEIGH_SEARCH_FIXED_DISTANCE:
    {
      try
      {
        const float fixed_distance = boost::lexical_cast<float>(param);
        return Searcher::Ptr(new FixedDistanceSearcher(fixed_distance));
      }
      catch (const boost::bad_lexical_cast &)
      {
        throw ParserException(std::string("Parameter for fixed distance search must be a float, it is \"") +
                              param + std::string("\" instead."));
      }
    }
    case PARAM_VALUE_NEIGH_SEARCH_KNN:
    {
      try
      {
        const uint32 knn = boost::lexical_cast<uint32>(param);
        return Searcher::Ptr(new KNearestNeighborsSearcher(knn));
      }
      catch (const boost::bad_lexical_cast &)
      {
        throw ParserException(std::string("Parameter for knn search must be an integer, it is \"") +
                              param + std::string("\" instead."));
      }
    }
    default:
      throw ParserException(std::string("Unknown search type: ") +
                            boost::lexical_cast<std::string>(id) + ".");
  }
}

PointNeighborhoodSearch::Searcher::ConstPtr PointNeighborhoodSearch::CreateFromIstream(std::istream & ifile)
{
  uint64 id;
  ifile.read((char *)&id,sizeof(id));
  if (!ifile)
    throw ParserException("Unexpected EOF while reading neighborhood searcher id.");

  switch (id)
  {
    case PARAM_VALUE_NEIGH_SEARCH_FIXED_DISTANCE:
    {
      float distance;
      ifile.read((char *)&distance,sizeof(distance));
      if (!ifile)
        throw ParserException("Unexpected EOF while reading fixed distance search parameter.");

      return Searcher::Ptr(new FixedDistanceSearcher(distance));
    }
    case PARAM_VALUE_NEIGH_SEARCH_KNN:
    {
      uint32 knn;
      ifile.read((char *)&knn,sizeof(knn));
      if (!ifile)
        throw ParserException("Unexpected EOF while reading knn search parameter.");

      return Searcher::Ptr(new KNearestNeighborsSearcher(knn));
    }
    default:
      throw ParserException(std::string("Unknown search type: ") +
                            boost::lexical_cast<std::string>(id) + std::string("."));
  }
}


