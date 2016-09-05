/*
 * Copyright (c) 2016, Riccardo Monica
 */

#include "rviz_cloud_annotation_undo.h"

#include <sstream>

typedef RVizCloudAnnotationUndo::SetControlPointAction SetControlPointAction;
typedef RVizCloudAnnotationUndo::Action Action;
typedef RVizCloudAnnotationUndo::SetNameForLabelAction SetNameForLabelAction;
typedef RVizCloudAnnotationUndo::ClearLabelAction ClearLabelAction;
typedef RVizCloudAnnotationUndo::RestoreLabelAction RestoreLabelAction;
typedef RVizCloudAnnotationUndo::ClearAction ClearAction;
typedef RVizCloudAnnotationUndo::RestoreAction RestoreAction;

#define QUEUE_SIZE_SANITY (10000)

RVizCloudAnnotationUndo::RVizCloudAnnotationUndo()
{
  m_undone_count = 0;
}

void RVizCloudAnnotationUndo::SetAnnotation(RVizCloudAnnotationPoints::Ptr annotation)
{
  m_annotation = annotation;
  Reset();
}

void RVizCloudAnnotationUndo::Reset()
{
  m_actions.clear();
  m_undone_count = 0;
}

RVizCloudAnnotationUndo::Uint64Vector RVizCloudAnnotationUndo::SetControlPoint(const uint64 idx,const uint32 next_label)
{
  const uint32 prev_label = m_annotation->GetControlPointForPoint(idx);
  if (prev_label == next_label)
    return Uint64Vector(); // nothing to do

  const Action::Ptr action(new SetControlPointAction(idx,prev_label,next_label));
  PushAction(action);
  return action->Execute(*m_annotation);
}

void RVizCloudAnnotationUndo::PushAction(Action::Ptr action)
{
  for ( ; m_undone_count > 0; m_undone_count--)
    m_actions.pop_front(); // bring m_undone_count to 0 by popping all actions

  m_actions.push_front(action);
  while (m_actions.size() > QUEUE_SIZE_SANITY)
    m_actions.pop_back();
}

RVizCloudAnnotationUndo::Uint64Vector RVizCloudAnnotationUndo::Undo()
{
  if (m_actions.size() == 0)
    return Uint64Vector(); // nothing to undo

  if (m_actions.size() == m_undone_count)
    return Uint64Vector(); // everything is undone

  const Action::Ptr to_be_undone = m_actions[m_undone_count];
  m_undone_count++;
  return to_be_undone->Inverse()->Execute(*m_annotation);
}

RVizCloudAnnotationUndo::Uint64Vector RVizCloudAnnotationUndo::Redo()
{
  if (m_undone_count == 0)
    return Uint64Vector(); // everything is done

  m_undone_count--;
  const Action::Ptr to_be_done = m_actions[m_undone_count];
  return to_be_done->Execute(*m_annotation);
}

bool RVizCloudAnnotationUndo::IsUndoEnabled() const
{
  return m_undone_count < m_actions.size();
}

bool RVizCloudAnnotationUndo::IsRedoEnabled() const
{
  return m_undone_count > 0;
}

std::string RVizCloudAnnotationUndo::GetUndoDescription() const
{
  if (!IsUndoEnabled())
    return "";
  return m_actions[m_undone_count]->GetDescription();
}

std::string RVizCloudAnnotationUndo::GetRedoDescription() const
{
  if (!IsRedoEnabled())
    return "";
  return m_actions[m_undone_count - 1]->GetDescription();
}

RVizCloudAnnotationUndo::Uint64Vector RVizCloudAnnotationUndo::Clear()
{
  Uint64Vector prev_control_points;
  Uint64Vector prev_labels;
  StringVector names;
  const uint64 max_label = m_annotation->GetMaxLabel();
  names.resize(max_label);
  for (uint64 i = 0; i < max_label; i++)
  {
    names[i] = m_annotation->GetNameForLabel(i + 1);

    const Uint64Vector cp = m_annotation->GetControlPointList(i + 1);
    if (!cp.empty())
    {
      const Uint64Vector lb(cp.size(),i + 1);
      prev_control_points.insert(prev_control_points.end(),cp.begin(),cp.end());
      prev_labels.insert(prev_labels.end(),lb.begin(),lb.end());
    }
  }

  const Action::Ptr action(new ClearAction(prev_labels,prev_control_points,names));
  PushAction(action);
  return m_annotation->Clear();
}

RVizCloudAnnotationUndo::Uint64Vector RVizCloudAnnotationUndo::ClearLabel(const uint32 label)
{
  const Uint64Vector prev_control_points = m_annotation->GetControlPointList(label);
  if (prev_control_points.empty())
    return Uint64Vector();
  const Action::Ptr action(new ClearLabelAction(label,prev_control_points));
  PushAction(action);
  return m_annotation->ClearLabel(label);
}

RVizCloudAnnotationUndo::Uint64Vector RVizCloudAnnotationUndo::SetNameForLabel(const uint32 label,const std::string & new_name)
{
  const std::string prev_name = m_annotation->GetNameForLabel(label);
  if (prev_name == new_name)
    return Uint64Vector(); // nothing to do

  const Action::Ptr action(new SetNameForLabelAction(label,prev_name,new_name));
  PushAction(action);
  return action->Execute(*m_annotation);
}

// -----

RVizCloudAnnotationUndo::Uint64Vector SetControlPointAction::Execute(RVizCloudAnnotationPoints & annotation) const
{
  return annotation.SetControlPoint(m_idx,m_next_label);
}

Action::Ptr SetControlPointAction::Inverse() const
{
  return Action::Ptr(new SetControlPointAction(m_idx,m_next_label,m_prev_label));
}

std::string SetControlPointAction::GetDescription() const
{
  std::ostringstream oss;
  if (m_next_label && !m_prev_label)
    oss << "Set " << m_next_label;
  else if (m_prev_label && !m_next_label)
    oss << "Del " << m_prev_label;
  else
    oss << "Set " << m_prev_label << " -> " << m_next_label;
  return oss.str();
}

SetControlPointAction::SetControlPointAction(const uint64 idx,const uint32 prev_label,const uint32 next_label)
{
  m_idx = idx;
  m_prev_label = prev_label;
  m_next_label = next_label;
}

// -----

SetNameForLabelAction::SetNameForLabelAction(const uint32 label,const std::string & prev_name,const std::string & new_name)
{
  m_label = label;
  m_prev_name = prev_name;
  m_new_name = new_name;
}

RVizCloudAnnotationUndo::Uint64Vector SetNameForLabelAction::Execute(RVizCloudAnnotationPoints & annotation) const
{
  annotation.SetNameForLabel(m_label,m_new_name);
  Uint64Vector result;
  result.push_back(m_label);
  return result;
}

Action::Ptr SetNameForLabelAction::Inverse() const
{
  return Action::Ptr(new SetNameForLabelAction(m_label,m_new_name,m_prev_name));
}

std::string SetNameForLabelAction::GetDescription() const
{
  std::ostringstream oss;
  oss << "Name " << m_label;
  return oss.str();
}

// -----

RVizCloudAnnotationUndo::Uint64Vector ClearLabelAction::Execute(RVizCloudAnnotationPoints & annotation) const
{
  return annotation.ClearLabel(m_label);
}

Action::Ptr ClearLabelAction::Inverse() const
{
  return Action::Ptr(new RestoreLabelAction(m_label,m_prev_control_points));
}

std::string ClearLabelAction::GetDescription() const
{
  std::ostringstream oss;
  oss << "Clear " << m_label;
  return oss.str();
}

ClearLabelAction::ClearLabelAction(const uint32 label,const Uint64Vector & prev_control_points)
{
  m_label = label;
  m_prev_control_points = prev_control_points;
}

RVizCloudAnnotationUndo::Uint64Vector RestoreLabelAction::Execute(RVizCloudAnnotationPoints & annotation) const
{
  return annotation.SetControlPointList(m_control_points,m_label);
}

Action::Ptr RestoreLabelAction::Inverse() const
{
  return Action::Ptr(new ClearLabelAction(m_label,m_control_points));
}

std::string RestoreLabelAction::GetDescription() const
{
  std::ostringstream oss;
  oss << "Restore " << m_label;
  return oss.str();
}

RestoreLabelAction::RestoreLabelAction(const uint32 label,const Uint64Vector & control_points)
{
  m_label = label;
  m_control_points = control_points;
}

// -----

RVizCloudAnnotationUndo::Uint64Vector ClearAction::Execute(RVizCloudAnnotationPoints & annotation) const
{
  return annotation.Clear();
}

Action::Ptr ClearAction::Inverse() const
{
  return Action::Ptr(new RestoreAction(m_labels,m_prev_control_points,m_names));
}

std::string ClearAction::GetDescription() const
{
  return "Clear all";
}

ClearAction::ClearAction(const Uint64Vector & labels,const Uint64Vector & control_points,const StringVector & names)
{
  m_labels = labels;
  m_prev_control_points = control_points;
  m_names = names;
}

RVizCloudAnnotationUndo::Uint64Vector RestoreAction::Execute(RVizCloudAnnotationPoints & annotation) const
{
  const uint64 size = m_names.size();
  Uint64Vector all(size);
  for (uint64 i = 0; i < m_names.size(); i++)
  {
    annotation.SetNameForLabel(i + 1,m_names[i]);
    all[i] = i + 1;
  }

  annotation.SetControlPointList(m_control_points,m_labels);

  return all;
}

Action::Ptr RestoreAction::Inverse() const
{
  return Action::Ptr(new ClearAction(m_labels,m_control_points,m_names));
}

std::string RestoreAction::GetDescription() const
{
  return "Restore all";
}

RestoreAction::RestoreAction(const Uint64Vector & labels,const Uint64Vector & control_points,const StringVector & names)
{
  m_labels = labels;
  m_control_points = control_points;
  m_names = names;
}
