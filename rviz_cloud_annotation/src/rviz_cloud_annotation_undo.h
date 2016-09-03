/*
 * Copyright (c) 2016, Riccardo Monica
 */

#ifndef RVIZ_CLOUD_ANNOTATION_UNDO_H
#define RVIZ_CLOUD_ANNOTATION_UNDO_H

#include "rviz_cloud_annotation_points.h"

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>

// STL
#include <deque>
#include <vector>
#include <stdint.h>
#include <string>

class RVizCloudAnnotationUndo
{
  public:
  typedef uint64_t uint64;
  typedef uint32_t uint32;
  typedef std::vector<uint64> Uint64Vector;

  RVizCloudAnnotationUndo();

  // interface
  class Action
  {
    public:
    typedef boost::shared_ptr<Action> Ptr;
    typedef boost::shared_ptr<const Action> ConstPtr;

    virtual ~Action() {}

    // returns a list of affected markers
    virtual Uint64Vector Execute(RVizCloudAnnotationPoints & annotation) const = 0;
    virtual Ptr Inverse() const = 0; // get the inverse action from the current one
    virtual std::string GetDescription() const = 0;
  };

  typedef std::deque<Action::Ptr> ActionPtrDeque;

  class SetControlPointAction: public Action
  {
    public:
    Uint64Vector Execute(RVizCloudAnnotationPoints & annotation) const;
    Action::Ptr Inverse() const;
    std::string GetDescription() const;

    SetControlPointAction(const uint64 idx,const uint32 prev_label,const uint32 next_label);

    private:
    uint64 m_idx;
    uint32 m_prev_label;
    uint32 m_next_label;
  };

  Uint64Vector SetControlPoint(const uint64 idx,const uint32 next_label);
  Uint64Vector Clear() {return m_annotation->Clear(); }
  Uint64Vector ClearLabel(const uint32 label) {return m_annotation->ClearLabel(label); }
  Uint64Vector SetNameForLabel(const uint32 label,const std::string & name)
    {m_annotation->SetNameForLabel(label,name); return Uint64Vector(); }

  Uint64Vector Undo();
  Uint64Vector Redo();

  bool IsUndoEnabled() const;
  bool IsRedoEnabled() const;
  std::string GetUndoDescription() const;
  std::string GetRedoDescription() const;

  RVizCloudAnnotationPoints::Ptr GetAnnotation() const {return m_annotation; }

  void SetAnnotation(RVizCloudAnnotationPoints::Ptr annotation);
  void Reset();

  private:
  void PushAction(Action::Ptr action);

  RVizCloudAnnotationPoints::Ptr m_annotation;

  ActionPtrDeque m_actions;
  uint64 m_undone_count;
};

#endif // RVIZ_CLOUD_ANNOTATION_UNDO_H
