#include "rviz_cloud_annotation_points.h"

#include <boost/lexical_cast.hpp>

RVizCloudAnnotationPoints::RVizCloudAnnotationPoints(const uint64 cloud_size)
{
  m_control_points_assoc.resize(cloud_size,0);
  m_labels_assoc.resize(cloud_size,0);
  m_cloud_size = cloud_size;
}

void RVizCloudAnnotationPoints::Clear()
{
  m_control_points_assoc.clear();
  m_labels_assoc.clear();
  m_control_points.clear();

  m_control_points_assoc.resize(m_cloud_size,0);
  m_labels_assoc.resize(m_cloud_size,0);
}

RVizCloudAnnotationPoints::Uint64Vector RVizCloudAnnotationPoints::SetControlPoint(const uint64 point_id,const uint64 label)
{
  uint64 prev_label = m_control_points_assoc[point_id];

  if (prev_label == label)
    return Uint64Vector(); // nothing to do

  if (prev_label != 0)
  {
    Uint64Vector & control_point_indices = m_control_points[prev_label - 1];
    for (Uint64Vector::iterator iter = control_point_indices.begin(); iter != control_point_indices.end(); ++iter)
      if (*iter == point_id)
      {
        control_point_indices.erase(iter);
        break;
      }
  }

  m_control_points_assoc[point_id] = label;

  if (label != 0)
  {
    if (m_control_points.size() < label)
      m_control_points.resize(label);

    Uint64Vector & control_point_indices = m_control_points[label - 1];
    control_point_indices.push_back(point_id);
  }

  return UpdateLabels(point_id,prev_label,label);
}

RVizCloudAnnotationPoints::Uint64Vector RVizCloudAnnotationPoints::UpdateLabels(
  const uint64 point_id,const uint64 prev_label,const uint64 next_label)
{
  m_labels_assoc[point_id] = next_label;
  Uint64Vector result;
  if (prev_label != 0)
    result.push_back(prev_label);
  if (next_label != 0)
    result.push_back(next_label);
  return result;
}

#define MAGIC_STRING "ANNOTATION"
#define MAGIC_VERSION (1)

RVizCloudAnnotationPoints::Ptr RVizCloudAnnotationPoints::Deserialize(std::istream & ifile)
{
  if (!ifile)
    throw IOE("Invalid file stream.");

  const std::string magic_string = MAGIC_STRING;
  Uint8Vector maybe_magic_string(magic_string.size() + 1);
  ifile.read((char *)(maybe_magic_string.data()),magic_string.size() + 1);
  if (!ifile)
    throw IOE("Unexpected EOF while reading magic string.");
  if (std::memcmp(magic_string.c_str(),maybe_magic_string.data(),magic_string.size() + 1) != 0)
    throw IOE("Invalid magic string.");

  uint64 version;
  ifile.read((char *)&version,sizeof(version));
  if (!ifile)
    throw IOE("Unexpected EOF while reading version.");
  if (version != MAGIC_VERSION)
    throw IOE(std::string("Invalid version: ") + boost::lexical_cast<std::string>(version));

  uint64 cloud_size;
  ifile.read((char *)&cloud_size,sizeof(cloud_size));
  if (!ifile)
    throw IOE("Unexpected EOF while reading cloud size.");

  RVizCloudAnnotationPoints::Ptr resultptr(new RVizCloudAnnotationPoints(cloud_size));
  RVizCloudAnnotationPoints & result = *resultptr;

  uint64 control_points_size;
  ifile.read((char *)&control_points_size,sizeof(control_points_size));
  if (!ifile)
    throw IOE("Unexpected EOF while reading control point size.");
  result.m_control_points.resize(control_points_size);

  for (uint64 i = 0; i < control_points_size; i++)
  {
    uint64 control_point_size;
    ifile.read((char *)&control_point_size,sizeof(control_point_size));
    if (!ifile)
      throw IOE("Unexpected EOF while reading size of control point " + boost::lexical_cast<std::string>(i) + ".");
    result.m_control_points[i].resize(control_point_size);

    for (uint64 h = 0; h < control_point_size; h++)
    {
      uint64 point_index;
      ifile.read((char *)&point_index,sizeof(point_index));
      if (!ifile)
        throw IOE("Unexpected EOF while reading content of control point " + boost::lexical_cast<std::string>(i) + ".");
      result.m_control_points[i][h] = point_index;
    }
  }

  for (uint64 i = 0; i < cloud_size; i++)
  {
    uint32 centroid_index;
    ifile.read((char *)&centroid_index,sizeof(centroid_index));
    if (!ifile)
      throw IOE("Unexpected EOF while reading control point association.");
    result.m_control_points_assoc[i] = centroid_index;
  }

  for (uint64 i = 0; i < cloud_size; i++)
  {
    uint32 label_index;
    ifile.read((char *)&label_index,sizeof(label_index));
    if (!ifile)
      throw IOE("Unexpected EOF while reading label association.");
    result.m_labels_assoc[i] = label_index;
  }

  return resultptr;
}

void RVizCloudAnnotationPoints::Serialize(std::ostream & ofile) const
{
  if (!ofile)
    throw IOE("Invalid file stream.");

  const std::string magic_string = MAGIC_STRING;
  ofile.write(magic_string.c_str(),magic_string.size() + 1);
  const uint64 version = MAGIC_VERSION;
  ofile.write((char *)&version,sizeof(version));
  const uint64 cloud_size = m_cloud_size;
  ofile.write((char *)&cloud_size,sizeof(cloud_size));
  const uint64 control_points_size = m_control_points.size();
  ofile.write((char *)&control_points_size,sizeof(control_points_size));

  for (uint64 i = 0; i < control_points_size; i++)
  {
    const uint64 control_point_size = m_control_points[i].size();
    ofile.write((char *)&control_point_size,sizeof(control_point_size));
    for (uint64 h = 0; h < control_point_size; h++)
    {
      const uint64 point_index = m_control_points[i][h];
      ofile.write((char *)&point_index,sizeof(point_index));
    }
  }

  for (uint64 i = 0; i < cloud_size; i++)
  {
    const uint32 centroid_index = m_control_points_assoc[i];
    ofile.write((char *)&centroid_index,sizeof(centroid_index));
  }

  for (uint64 i = 0; i < cloud_size; i++)
  {
    const uint32 label_index = m_labels_assoc[i];
    ofile.write((char *)&label_index,sizeof(label_index));
  }

  if (!ofile)
    throw IOE("Write error.");
}
