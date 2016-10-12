/*
 * Copyright (c) 2016, Riccardo Monica
 */

#include "rviz_cloud_annotation_points.h"
#include "rviz_cloud_annotation.h"

#include <boost/lexical_cast.hpp>

#include <algorithm>

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

  m_control_points.clear();
  m_erased_control_points.clear();
  m_ext_label_names.clear();
  m_control_points_for_label.clear();

  m_control_points_assoc.assign(m_cloud_size,0);
  m_labels_assoc.assign(m_cloud_size,0);
  m_last_generated_tot_dists.assign(m_cloud_size,0.0);

  return result;
}

RVizCloudAnnotationPoints::Uint64Vector RVizCloudAnnotationPoints::SetControlPoint(const uint64 point_id,const uint64 label)
{
  ExpandControlPointsUntil(label);

  const uint64 prev_control_point_id = m_control_points_assoc[point_id];

  if (prev_control_point_id == 0)
  {
    if (label == 0)
      return Uint64Vector(); // nothing to do

    const uint64 new_id = InternalSetControlPoint(point_id,label);
    Uint64Vector indices;
    BoolVector touched;
    indices.push_back(new_id);
    UpdateLabelAssocAdded(indices,touched);

    return TouchedBoolVectorToExtLabel(touched);
  }
  else // if (prev_control_point_id != 0)
  {
    const uint32 prev_label = m_control_points[prev_control_point_id - 1].label_id;

    if (label == 0)
    {
      Uint64Vector indices;
      BoolVector touched;
      indices.push_back(prev_control_point_id);
      UpdateLabelAssocDeleted(indices,touched);
      touched[prev_control_point_id - 1] = true;
      const Uint64Vector result = TouchedBoolVectorToExtLabel(touched);

      InternalSetControlPoint(point_id,0);

      return result;
    }
    else // if (label != 0)
    {
      InternalSetControlPoint(point_id,label);
      Uint64Vector result;
      result.push_back(label);
      result.push_back(prev_label);
      return result;
    }
  }
}

RVizCloudAnnotationPoints::Uint64Vector RVizCloudAnnotationPoints::SetControlPointList(
  const Uint64Vector & point_ids,const uint64 label)
{
  const Uint64Vector all_labels(point_ids.size(),label);
  return SetControlPointList(point_ids,all_labels);
}

RVizCloudAnnotationPoints::Uint64Vector RVizCloudAnnotationPoints::SetControlPointList(
  const Uint64Vector & point_ids,const Uint64Vector & labels)
{
  Uint64Set touched;

  const uint64 size = point_ids.size();
  for (uint64 i = 0; i < size; i++)
  {
    const Uint64Vector local_touched = SetControlPoint(point_ids[i],labels[i]);
    touched.insert(local_touched.begin(),local_touched.end());
  }

  return Uint64Vector(touched.begin(),touched.end());
}

RVizCloudAnnotationPoints::Uint64Vector RVizCloudAnnotationPoints::GetControlPointList(const uint64 label) const
{
  if (label > GetMaxLabel())
    return Uint64Vector();

  const Uint64Vector & control_point_ids = m_control_points_for_label[label - 1];
  Uint64Vector result;
  result.reserve(control_point_ids.size());
  for (uint64 i = 0; i < control_point_ids.size(); i++)
    result.push_back(m_control_points[control_point_ids[i] - 1].point_id);

  return result;
}

RVizCloudAnnotationPoints::Uint64Vector RVizCloudAnnotationPoints::GetLabelPointList(const uint64 label) const
{
  Uint64Vector result;

  if (label > GetMaxLabel())
    return result;

  if (label == 0)
  {
    for (uint64 i = 0; i < m_cloud_size; i++)
      if (m_labels_assoc[i] == 0)
        result.push_back(i);
    return result;
  }

  const Uint64Vector & internal_label_list = m_control_points_for_label[label - 1];
  const Uint64Set internal_label_search(internal_label_list.begin(),internal_label_list.end());

  for (uint64 i = 0; i < m_cloud_size; i++)
    if (internal_label_search.find(m_labels_assoc[i]) != internal_label_search.end())
      result.push_back(i);

  return result;
}

void RVizCloudAnnotationPoints::UpdateRegionGrowing(const uint64 cloud_size,
                                                    const PointNeighborhood & point_neighborhood,
                                                    const Uint64Vector & seeds,
                                                    Uint64Vector & labels_assoc,
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
    const uint64 current_label = labels_assoc[current];

    const float * neigh_dists;
    const float * neigh_tot_dists;
    const uint64 * neighs;
    const uint64 neighs_size = point_neighborhood.GetNeigborhoodAsPointer(current,neighs,neigh_tot_dists,neigh_dists);

    for (uint64 i = 0; i < neighs_size; i++)
    {
      const uint64 next = neighs[i];
      const float next_tot_dist = neigh_tot_dists[i] + current_tot_dist;
      const uint64 next_label = labels_assoc[next];

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
                                                 const ControlPointVector & control_points,
                                                 const PointNeighborhood & point_neighborhood,
                                                 Uint64Vector & labels_assoc,
                                                 FloatVector & last_generated_tot_dists,
                                                 BoolVector & touched)
{
  labels_assoc.assign(cloud_size,0);
  last_generated_tot_dists.assign(cloud_size,0.0);
  touched.assign(control_points.size(),false);

  Uint64Vector seeds;
  for (uint64 i = 0; i < control_points.size(); i++)
  {
    const ControlPoint & first = control_points[i];
    if (first.Invalid())
      continue;

    seeds.push_back(first.point_id);
    labels_assoc[first.point_id] = i + 1;
    last_generated_tot_dists[first.point_id] = 0.0;
  }

  UpdateRegionGrowing(cloud_size,point_neighborhood,seeds,labels_assoc,last_generated_tot_dists,touched);
}

void RVizCloudAnnotationPoints::RegenerateLabelAssoc(BoolVector & touched)
{
  return RunRegionGrowing(m_cloud_size,m_control_points,*m_point_neighborhood,m_labels_assoc,m_last_generated_tot_dists,touched);
}

void RVizCloudAnnotationPoints::UpdateLabelAssocAdded(const Uint64Vector & added_indices,BoolVector & touched)
{
  touched.assign(m_control_points.size(),false);
  Uint64Vector seeds;

  for (uint64 i = 0; i < added_indices.size(); i++)
  {
    touched[added_indices[i] - 1] = true; // we are adding it
    const uint64 point_id = m_control_points[added_indices[i] - 1].point_id;
    seeds.push_back(point_id);
    m_labels_assoc[point_id] = added_indices[i];
    m_last_generated_tot_dists[point_id] = 0.0;
  }

  UpdateRegionGrowing(m_cloud_size,*m_point_neighborhood,seeds,m_labels_assoc,m_last_generated_tot_dists,touched);
}

void RVizCloudAnnotationPoints::UpdateLabelAssocDeleted(const Uint64Vector & removed_indices,BoolVector & touched)
{
  touched.clear();
  touched.resize(m_control_points.size(),false);
  for (uint64 i = 0; i < removed_indices.size(); i++)
    touched[removed_indices[i] - 1] = true; // we are removing it

  Uint64Set seeds_set;

  for (uint64 i = 0; i < m_cloud_size; i++)
    if (std::find(removed_indices.begin(),removed_indices.end(),m_labels_assoc[i]) != removed_indices.end())
    {
      const float * neigh_dists;
      const float * neigh_tot_dists;
      const uint64 * neighs;
      const uint64 neighs_size = m_point_neighborhood->GetNeigborhoodAsPointer(i,neighs,neigh_tot_dists,neigh_dists);

      for (uint64 h = 0; h < neighs_size; h++)
      {
        const uint32 label = m_labels_assoc[neighs[h]];
        if (label != 0 && std::find(removed_indices.begin(),removed_indices.end(),label) == removed_indices.end())
          seeds_set.insert(neighs[h]);
      }

      m_labels_assoc[i] = 0;
      m_last_generated_tot_dists[i] = 0.0;
    }

  const Uint64Vector seeds(seeds_set.begin(),seeds_set.end());

  UpdateRegionGrowing(m_cloud_size,*m_point_neighborhood,seeds,m_labels_assoc,m_last_generated_tot_dists,touched);
}

void RVizCloudAnnotationPoints::UpdateLabelAssocChanged(const Uint64Vector & changed_indices,BoolVector & touched)
{
  touched.assign(m_control_points.size(),false);
  for (uint64 i = 0; i < changed_indices.size(); i++)
    touched[changed_indices[i] - 1] = true;
}

RVizCloudAnnotationPoints::Uint64Vector RVizCloudAnnotationPoints::ClearLabel(const uint64 label)
{
  if (label == 0)
    return Uint64Vector();
  if (label > GetMaxLabel())
    return Uint64Vector();

  const Uint64Vector control_points_ids = m_control_points_for_label[label - 1];
  if (control_points_ids.empty())
    return Uint64Vector();

  BoolVector touched;
  UpdateLabelAssocDeleted(control_points_ids,touched);
  const Uint64Vector result = TouchedBoolVectorToExtLabel(touched);

  for (uint64 i = 0; i < control_points_ids.size(); i++)
    InternalSetControlPoint(m_control_points[control_points_ids[i] - 1].point_id,0);

  return result;
}

RVizCloudAnnotationPoints::Uint64Vector RVizCloudAnnotationPoints::SetNameForLabel(const uint64 label,const std::string & name)
{
  ExpandControlPointsUntil(label);
  m_ext_label_names[label - 1] = name;
  return Uint64Vector(1,label);
}

void RVizCloudAnnotationPoints::ExpandControlPointsUntil(const uint64 label)
{
  if (label <= GetMaxLabel())
    return;

  m_ext_label_names.resize(label);
  m_control_points_for_label.resize(label);
}

#define MAGIC_STRING "ANNOTATION"
#define MAGIC_MIN_VERSION (1)
#define MAGIC_MAX_VERSION (3)

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

    PointNeighborhoodSearch::Searcher::ConstPtr searcher;
    if (version <= 2)
    {
      float search_distance;
      ifile.read((char *)&search_distance,sizeof(search_distance));
      searcher = PointNeighborhoodSearch::CreateFromString(0,boost::lexical_cast<std::string>(search_distance));
    }
    else
    {
      try
      {
        searcher = PointNeighborhoodSearch::CreateFromIstream(ifile);
      }
      catch (const PointNeighborhoodSearch::ParserException & ex)
      {
        throw IOE(std::string("Invalid neighborhood configuration parameters: ") + ex.message);
      }
    }

    if (!ifile)
      throw IOE("Unexpected EOF while reading neighborhood configuration parameters.");

    if (position_importance != conf.position_importance ||
      color_importance != conf.color_importance ||
      normal_importance != conf.normal_importance ||
      max_distance != conf.max_distance ||
      !searcher->ApproxEquals(*conf.searcher))
    {
      const uint64 w = 30;
      std::ostringstream msg;
      msg << "Loaded neighborhood configuration parameters do not match: \n"
          << std::setw(w) << "Name" << std::setw(w) << "ROS param" << std::setw(w) << "File" << "\n"
          << std::setw(w) << PARAM_NAME_POSITION_IMPORTANCE
            << std::setw(w) << conf.position_importance << std::setw(w) << position_importance << "\n"
          << std::setw(w) << PARAM_NAME_COLOR_IMPORTANCE
            << std::setw(w) << conf.color_importance << std::setw(w) << color_importance << "\n"
          << std::setw(w) << PARAM_NAME_NORMAL_IMPORTANCE
            << std::setw(w) << conf.normal_importance << std::setw(w) << normal_importance << "\n"
          << std::setw(w) << PARAM_NAME_MAX_DISTANCE
            << std::setw(w) << conf.max_distance << std::setw(w) << max_distance << "\n"
          << std::setw(w) << PARAM_NAME_NEIGH_SEARCH_TYPE
            << std::setw(w) << conf.searcher->GetId() << std::setw(w) << searcher->GetId() << "\n"
          << std::setw(w) << PARAM_NAME_NEIGH_SEARCH_PARAMS
            << std::setw(w) << conf.searcher->ToString() << std::setw(w) << searcher->ToString() << "\n"
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

    for (uint64 h = 0; h < control_point_size; h++)
    {
      uint64 point_index;
      ifile.read((char *)&point_index,sizeof(point_index));
      if (!ifile)
        throw IOE("Unexpected EOF while reading content of control point " + boost::lexical_cast<std::string>(i) + ".");
      result.InternalSetControlPoint(point_index,i + 1);
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
      result.m_ext_label_names[i] = str;
    }
  }

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
    conf.searcher->Serialize(ofile);
  }

  const uint64 control_points_size = m_control_points_for_label.size();
  ofile.write((char *)&control_points_size,sizeof(control_points_size));

  for (uint64 i = 0; i < control_points_size; i++)
  {
    const uint64 control_point_size = m_control_points_for_label[i].size();
    ofile.write((char *)&control_point_size,sizeof(control_point_size));
    for (uint64 h = 0; h < control_point_size; h++)
    {
      const uint64 control_point_index = m_control_points_for_label[i][h];
      const uint64 point_index = m_control_points[control_point_index - 1].point_id;
      ofile.write((char *)&point_index,sizeof(point_index));
    }
  }

  for (uint64 i = 0; i < control_points_size; i++)
  {
    uint32 string_size = m_ext_label_names[i].size();
    ofile.write((char *)&string_size,sizeof(string_size));
    ofile.write(m_ext_label_names[i].c_str(),string_size);
  }

  if (!ofile)
    throw IOE("Write error.");
}

RVizCloudAnnotationPoints::uint64 RVizCloudAnnotationPoints::InternalSetControlPoint(const uint64 point_id,const uint32 label)
{
  ExpandControlPointsUntil(label);

  const uint64 prev_control_point_id = m_control_points_assoc[point_id];
  if (prev_control_point_id != 0)
  {
    const uint32 prev_label = m_control_points[prev_control_point_id - 1].label_id;

    if (label == 0)
    {
      // remove control point
      m_control_points_assoc[point_id] = 0;
      m_control_points[prev_control_point_id - 1].Invalidate();
      m_erased_control_points.push_back(prev_control_point_id);
      EraseFromVector(m_control_points_for_label[prev_label - 1],prev_control_point_id);

      while (!m_control_points.empty() && m_control_points.back().Invalid())
      {
        EraseFromVector(m_erased_control_points,m_control_points.size());
        m_control_points.pop_back();
      }

      return 0;
    }

    // change label of control point
    EraseFromVector(m_control_points_for_label[prev_label - 1],prev_control_point_id);
    m_control_points[prev_control_point_id - 1].label_id = label;
    m_control_points_for_label[label - 1].push_back(prev_control_point_id);
    return prev_control_point_id;
  }

  // ok, create new control point
  uint64 new_id;
  if (m_erased_control_points.empty())
  {
    m_control_points.push_back(ControlPoint(point_id,label));
    new_id = m_control_points.size();
  }
  else // if an erased control point is present, reuse it
  {
    new_id = m_erased_control_points.back();
    m_control_points[new_id - 1] = ControlPoint(point_id,label);
    m_erased_control_points.pop_back();
  }
  m_control_points_for_label[label - 1].push_back(new_id);
  m_control_points_assoc[point_id] = new_id;

  return new_id;
}

template <typename T>
  void RVizCloudAnnotationPoints::EraseFromVector(std::vector<T> & vector,const T value)
{
  typename std::vector<T>::iterator iter = std::find(vector.begin(),vector.end(),value);
  if (iter != vector.end())
    vector.erase(iter);
}

RVizCloudAnnotationPoints::Uint64Vector RVizCloudAnnotationPoints::TouchedBoolVectorToExtLabel(const BoolVector & touched) const
{
  BoolVector touched_labels(m_control_points_for_label.size(),false);
  Uint64Vector result;

  for (uint64 i = 0; i < touched.size(); i++)
    if (touched[i])
      touched_labels[m_control_points[i].label_id - 1] = true;

  for (uint64 i = 0; i < touched_labels.size(); i++)
    if (touched_labels[i])
      result.push_back(i + 1);

  return result;
}
