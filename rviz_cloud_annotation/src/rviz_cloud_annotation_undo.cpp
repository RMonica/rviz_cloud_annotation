/*
 * Copyright (c) 2016, Riccardo Monica
 */

#include "rviz_cloud_annotation_undo.h"

#include <sstream>

typedef RVizCloudAnnotationUndo::SetControlPointAction SetControlPointAction;
typedef RVizCloudAnnotationUndo::Action Action;

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
  if (m_next_label)
    oss << "Set label " << m_next_label;
  else
    oss << "Del label " << m_prev_label;
  return oss.str();
}

SetControlPointAction::SetControlPointAction(const uint64 idx,const uint32 prev_label,const uint32 next_label)
{
  m_idx = idx;
  m_prev_label = prev_label;
  m_next_label = next_label;
}
