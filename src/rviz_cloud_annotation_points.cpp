#include "rviz_cloud_annotation_points.h"

#include <boost/lexical_cast.hpp>

#include "rviz_cloud_annotation.h"

#include <pcl/console/time.h>
#define DEBUG_SELF_TEST false

RVizCloudAnnotationPoints::RVizCloudAnnotationPoints(const uint64 cloud_size,
                                                     const PointNeighborhood::ConstPtr neighborhood)
{
  m_cloud_size = cloud_size;

  m_control_points_assoc.resize(m_cloud_size,0);
  m_labels_assoc.resize(m_cloud_size,0);
  m_last_generated_tot_dists.resize(m_cloud_size,0.0);

  m_point_neighborhood = neighborhood;
}

RVizCloudAnnotationPoints::Uint64Vector RVizCloudAnnotationPoints::Clear()
{
  Uint64Vector result;
  result.resize(m_control_points.size());
  for (uint64 i = 0; i < result.size(); i++)
    result[i] = i + 1;

  m_control_points_assoc.clear();
  m_labels_assoc.clear();
  m_last_generated_tot_dists.clear();
  m_control_points.clear();
  m_control_point_names.clear();

  m_control_points_assoc.resize(m_cloud_size,0);
  m_labels_assoc.resize(m_cloud_size,0);
  m_last_generated_tot_dists.resize(m_cloud_size,0.0);

  return result;
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
    ExpandControlPointsUntil(label);

    Uint64Vector & control_point_indices = m_control_points[label - 1];
    control_point_indices.push_back(point_id);
  }

  Uint64Vector point_ids;
  point_ids.push_back(point_id);
  return UpdateLabels(point_ids,prev_label,label);
}

RVizCloudAnnotationPoints::Uint64Vector RVizCloudAnnotationPoints::UpdateLabels(
  const Uint64Vector & point_ids,const uint64 prev_label,const uint64 next_label)
{
  BoolVector touched;

  pcl::console::TicToc tictoc;
  if (DEBUG_SELF_TEST)
    tictoc.tic();

  if (prev_label == 0 && next_label != 0)
    UpdateLabelAssocAdded(point_ids,next_label,touched);
  else if (prev_label != 0 && next_label == 0)
    UpdateLabelAssocDeleted(point_ids,prev_label,touched);
  else if (prev_label != 0 && next_label != 0)
    UpdateLabelAssocChanged(point_ids,next_label,prev_label,touched);
  else
    RegenerateLabelAssoc(touched);

  if (DEBUG_SELF_TEST)
  {
    tictoc.toc_print();

    Uint32Vector prev_lbl = m_labels_assoc;
    tictoc.tic();
    RegenerateLabelAssoc(touched);
    tictoc.toc_print();
    if (prev_lbl != m_labels_assoc)
      std::cout << "SELF TEST ERROR!" << std::endl;
    m_labels_assoc = prev_lbl;
  }

  if (prev_label != 0)
    touched[prev_label - 1] = true;
  if (next_label != 0)
    touched[next_label - 1] = true;

  Uint64Vector result;
  for (uint64 i = 0; i < touched.size(); i++)
    if (touched[i])
      result.push_back(i + 1);
  return result;
}

RVizCloudAnnotationPoints::Uint64Vector RVizCloudAnnotationPoints::GetLabelPointList(const uint64 label) const
{
  Uint64Vector result;

  for (uint64 i = 0; i < m_cloud_size; i++)
    if (m_labels_assoc[i] == label)
      result.push_back(i);

  return result;
}

void RVizCloudAnnotationPoints::RegenerateControlPointsAssoc()
{
  for (uint64 i = 0; i < m_cloud_size; i++)
    m_control_points_assoc[i] = 0;

  const uint64 control_points_size = m_control_points.size();
  for (uint64 i = 0; i < control_points_size; i++)
  {
    const uint64 size = m_control_points[i].size();
    for (uint64 h = 0; h < size; h++)
      m_control_points_assoc[m_control_points[i][h]] = i + 1;
  }
}

void RVizCloudAnnotationPoints::UpdateRegionGrowing(const uint64 cloud_size,
                                                    const PointNeighborhood & point_neighborhood,
                                                    const Uint64Vector & seeds,
                                                    Uint32Vector & labels_assoc,
                                                    FloatVector & last_generated_tot_dists,
                                                    BoolVector & touched)
{
  Uint64Queue queue;
  BoolVector in_queue(cloud_size,false);
  for (uint64 i = 0; i < seeds.size(); i++)
  {
    const uint64 first = seeds[i];
    queue.push(first);
    in_queue[first] = true;
  }

  while (!queue.empty())
  {
    const uint64 current = queue.front();
    queue.pop();
    in_queue[current] = false;

    const float current_tot_dist = last_generated_tot_dists[current];
    const uint32 current_label = labels_assoc[current];

    const float * neigh_dists;
    const float * neigh_tot_dists;
    const uint64 * neighs;
    const uint64 neighs_size = point_neighborhood.GetNeigborhoodAsPointer(current,neighs,neigh_tot_dists,neigh_dists);

    for (uint64 i = 0; i < neighs_size; i++)
    {
      const uint64 next = neighs[i];
      const float next_tot_dist = neigh_tot_dists[i] + current_tot_dist;
      const uint32 next_label = labels_assoc[next];

      if (next_tot_dist > 1.0)
        continue;

      if (next_label != 0 && last_generated_tot_dists[next] <= next_tot_dist)
        continue;

      if (next_label != 0)
        touched[next_label - 1] = true;
      touched[current_label - 1] = true;

      last_generated_tot_dists[next] = next_tot_dist;
      labels_assoc[next] = current_label;

      if (!in_queue[next])
      {
        in_queue[next] = true;
        queue.push(next);
      }
    }
  }
}

void RVizCloudAnnotationPoints::RunRegionGrowing(const uint64 cloud_size,
                                                 const Uint64VectorVector & control_points,
                                                 const PointNeighborhood & point_neighborhood,
                                                 Uint32Vector & labels_assoc,
                                                 FloatVector & last_generated_tot_dists,
                                                 BoolVector & touched)
{
  for (uint64 i = 0; i < cloud_size; i++)
  {
    labels_assoc[i] = 0;
    last_generated_tot_dists[i] = 0.0;
  }

  touched.clear();
  touched.resize(control_points.size(),false);

  Uint64Vector seeds;
  for (uint64 i = 0; i < control_points.size(); i++)
    for (uint64 h = 0; h < control_points[i].size(); h++)
    {
      const uint64 first = control_points[i][h];
      seeds.push_back(first);
      labels_assoc[first] = i + 1;
      last_generated_tot_dists[first] = 0.0;
    }

  UpdateRegionGrowing(cloud_size,point_neighborhood,seeds,labels_assoc,last_generated_tot_dists,touched);
}

void RVizCloudAnnotationPoints::RegenerateLabelAssoc(BoolVector & touched)
{
  return RunRegionGrowing(m_cloud_size,m_control_points,*m_point_neighborhood,m_labels_assoc,m_last_generated_tot_dists,touched);
}

void RVizCloudAnnotationPoints::UpdateLabelAssocAdded(const Uint64Vector & added_indices,
                                                      const uint32 added_label,BoolVector & touched)
{
  touched.clear();
  touched.resize(m_control_points.size(),false);
  touched[added_label - 1] = true; // we are adding it

  for (uint64 i = 0; i < added_indices.size(); i++)
  {
    m_labels_assoc[added_indices[i]] = added_label;
    m_last_generated_tot_dists[added_indices[i]] = 0.0;
  }

  UpdateRegionGrowing(m_cloud_size,*m_point_neighborhood,added_indices,m_labels_assoc,m_last_generated_tot_dists,touched);
}

void RVizCloudAnnotationPoints::UpdateLabelAssocDeleted(const Uint64Vector & removed_indices,
                                                        const uint32 removed_label,BoolVector & touched)
{
  touched.clear();
  touched.resize(m_control_points.size(),false);
  touched[removed_label - 1] = true; // we are removing it

  Uint64Vector seeds(m_control_points[removed_label - 1]); // all the control points of the removed label are seeds
  Uint64Set other_seeds;

  for (uint64 i = 0; i < m_cloud_size; i++)
    if (m_labels_assoc[i] == removed_label)
    {
      const float * neigh_dists;
      const float * neigh_tot_dists;
      const uint64 * neighs;
      const uint64 neighs_size = m_point_neighborhood->GetNeigborhoodAsPointer(i,neighs,neigh_tot_dists,neigh_dists);

      for (uint64 h = 0; h < neighs_size; h++)
      {
        const uint32 label = m_labels_assoc[neighs[h]];
        if (label != 0 && label != removed_label)
          other_seeds.insert(neighs[h]);
      }

      m_labels_assoc[i] = 0;
      m_last_generated_tot_dists[i] = 0.0;
    }

  for (uint64 i = 0; i < seeds.size(); i++)
  {
    m_labels_assoc[seeds[i]] = removed_label; // restore labels at control points
    m_last_generated_tot_dists[seeds[i]] = 0.0;
  }

  // do NOT set the m_last_generated_tot_dists at the other seeds to zero, since their label is not going to change
  seeds.insert(seeds.end(),other_seeds.begin(),other_seeds.end());

  UpdateRegionGrowing(m_cloud_size,*m_point_neighborhood,seeds,m_labels_assoc,m_last_generated_tot_dists,touched);
}

void RVizCloudAnnotationPoints::UpdateLabelAssocChanged(const Uint64Vector & changed_indices,const uint32 added_label,
                                                        const uint32 removed_label,BoolVector & touched)
{
  touched.clear();
  touched.resize(m_control_points.size(),false);
  touched[added_label - 1] = true;
  touched[removed_label - 1] = true;

  Uint64Vector seeds(m_control_points[removed_label - 1]); // all the control points of the removed label are seeds

  for (uint64 i = 0; i < m_cloud_size; i++)
    if (m_labels_assoc[i] == removed_label)
    {
      m_labels_assoc[i] = 0;
      m_last_generated_tot_dists[i] = 0.0;
    }

  for (uint64 i = 0; i < seeds.size(); i++)
  {
    m_labels_assoc[seeds[i]] = removed_label; // restore labels at control points
    m_last_generated_tot_dists[seeds[i]] = 0.0;
  }

  for (uint64 i = 0; i < changed_indices.size(); i++)
  {
    m_labels_assoc[changed_indices[i]] = added_label; // restore labels at changed points
    m_last_generated_tot_dists[changed_indices[i]] = 0.0;
  }

  seeds.insert(seeds.end(),changed_indices.begin(),changed_indices.end());

  UpdateRegionGrowing(m_cloud_size,*m_point_neighborhood,seeds,m_labels_assoc,m_last_generated_tot_dists,touched);
}

RVizCloudAnnotationPoints::Uint64Vector RVizCloudAnnotationPoints::ClearLabel(const uint64 label)
{
  if (label == 0)
    return Uint64Vector();
  if (label > m_control_points.size())
    return Uint64Vector();

  const Uint64Vector control_points = m_control_points[label - 1];
  if (control_points.empty())
    return Uint64Vector();

  m_control_points[label - 1].clear();

  for (uint64 i = 0; i < control_points.size(); i++)
    m_control_points_assoc[control_points[i]] = 0;

  return UpdateLabels(control_points,label,0);
}

void RVizCloudAnnotationPoints::SetNameForLabel(const uint64 label,const std::string & name)
{
  ExpandControlPointsUntil(label);
  m_control_point_names[label - 1] = name;
}

void RVizCloudAnnotationPoints::ExpandControlPointsUntil(const uint64 label)
{
  if (m_control_points.size() >= label)
    return;

  m_control_points.resize(label);
  m_control_point_names.resize(label);
}

#define MAGIC_STRING "ANNOTATION"
#define MAGIC_MIN_VERSION (1)
#define MAGIC_MAX_VERSION (2)

RVizCloudAnnotationPoints::Ptr RVizCloudAnnotationPoints::Deserialize(std::istream & ifile,
                                                                      PointNeighborhood::ConstPtr neighborhood)
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
  if (version < MAGIC_MIN_VERSION || version > MAGIC_MAX_VERSION)
    throw IOE(std::string("Invalid version: ") + boost::lexical_cast<std::string>(version));

  uint64 cloud_size;
  ifile.read((char *)&cloud_size,sizeof(cloud_size));
  if (!ifile)
    throw IOE("Unexpected EOF while reading cloud size.");

  {
    const PointNeighborhood::Conf & conf = neighborhood->GetConf();
    float position_importance;
    ifile.read((char *)&position_importance,sizeof(position_importance));
    float color_importance;
    ifile.read((char *)&color_importance,sizeof(color_importance));
    float normal_importance;
    ifile.read((char *)&normal_importance,sizeof(normal_importance));
    float max_distance;
    ifile.read((char *)&max_distance,sizeof(max_distance));
    float search_distance;
    ifile.read((char *)&search_distance,sizeof(search_distance));
    if (!ifile)
      throw IOE("Unexpected EOF while reading neighborhood configuration parameters.");

    if (position_importance != conf.position_importance ||
      color_importance != conf.color_importance ||
      normal_importance != conf.normal_importance ||
      max_distance != conf.max_distance ||
      search_distance != conf.search_distance)
    {
      const uint64 w = 30;
      std::ostringstream msg;
      msg << "Loaded neighborhood configuration parameters do not match: \n"
          << std::setw(w) << "Name" << std::setw(w) << "ROS param" << std::setw(w) << "File\n"
          << std::setw(w) << PARAM_NAME_POSITION_IMPORTANCE
            << std::setw(w) << conf.position_importance << std::setw(w) << position_importance << "\n"
          << std::setw(w) << PARAM_NAME_COLOR_IMPORTANCE
            << std::setw(w) << conf.color_importance << std::setw(w) << color_importance << "\n"
          << std::setw(w) << PARAM_NAME_NORMAL_IMPORTANCE
            << std::setw(w) << conf.normal_importance << std::setw(w) << normal_importance << "\n"
          << std::setw(w) << PARAM_NAME_MAX_DISTANCE
            << std::setw(w) << conf.max_distance << std::setw(w) << max_distance << "\n"
          << std::setw(w) << PARAM_NAME_NEIGH_SEARCH_DISTANCE
            << std::setw(w) << conf.search_distance << std::setw(w) << search_distance << "\n"
          ;
      throw IOE(msg.str());
    }

  }

  RVizCloudAnnotationPoints::Ptr resultptr(new RVizCloudAnnotationPoints(cloud_size,neighborhood));
  RVizCloudAnnotationPoints & result = *resultptr;

  uint64 control_points_size;
  ifile.read((char *)&control_points_size,sizeof(control_points_size));
  if (!ifile)
    throw IOE("Unexpected EOF while reading control point size.");
  result.ExpandControlPointsUntil(control_points_size);

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

  if (version >= 2)
  {
    for (uint64 i = 0; i < control_points_size; i++)
    {
      uint32 string_size;
      ifile.read((char *)&string_size,sizeof(string_size));
      if (!ifile)
        throw IOE("Unexpected EOF while reading text label size " + boost::lexical_cast<std::string>(i) + ".");
      Uint8Vector data(string_size + 1,0); // this is 0-terminated for sure
      ifile.read((char *)(data.data()),string_size);
      if (!ifile)
        throw IOE("Unexpected EOF while reading text label content " + boost::lexical_cast<std::string>(i) + ".");
      std::string str((const char *)(data.data()));
      result.m_control_point_names[i] = str;
    }
  }

  result.RegenerateControlPointsAssoc();

  BoolVector touched;
  result.RegenerateLabelAssoc(touched);

  return resultptr;
}

void RVizCloudAnnotationPoints::Serialize(std::ostream & ofile) const
{
  if (!ofile)
    throw IOE("Invalid file stream.");

  const std::string magic_string = MAGIC_STRING;
  ofile.write(magic_string.c_str(),magic_string.size() + 1);
  const uint64 version = MAGIC_MAX_VERSION;
  ofile.write((char *)&version,sizeof(version));
  const uint64 cloud_size = m_cloud_size;
  ofile.write((char *)&cloud_size,sizeof(cloud_size));

  {
    const PointNeighborhood::Conf & conf = m_point_neighborhood->GetConf();
    const float position_importance = conf.position_importance;
    ofile.write((char *)&position_importance,sizeof(position_importance));
    const float color_importance = conf.color_importance;
    ofile.write((char *)&color_importance,sizeof(color_importance));
    const float normal_importance = conf.normal_importance;
    ofile.write((char *)&normal_importance,sizeof(normal_importance));
    const float max_distance = conf.max_distance;
    ofile.write((char *)&max_distance,sizeof(max_distance));
    const float search_distance = conf.search_distance;
    ofile.write((char *)&search_distance,sizeof(search_distance));
  }

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

  for (uint64 i = 0; i < control_points_size; i++)
  {
    uint32 string_size = m_control_point_names[i].size();
    ofile.write((char *)&string_size,sizeof(string_size));
    ofile.write(m_control_point_names[i].c_str(),string_size);
  }

  if (!ofile)
    throw IOE("Write error.");
}
