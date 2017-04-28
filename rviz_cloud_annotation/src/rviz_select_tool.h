#ifndef RVIZ_SELECT_TOOL_H
#define RVIZ_SELECT_TOOL_H

#include <vector>
#include <stdint.h>

#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <rviz/tool.h>

namespace Ogre
{
  class Viewport;
}

namespace rviz
{
  class MoveTool;
  class ViewportMouseEvent;
  class InteractionTool;
  class RenderPanel;
}

namespace rviz_cloud_annotation
{
  class AnnotationSelectionTool: public rviz::Tool
  {
    public:
    typedef int32_t int32;
    typedef uint32_t uint32;

    AnnotationSelectionTool();
    virtual ~AnnotationSelectionTool();

    virtual void onInitialize();

    virtual void activate();
    virtual void deactivate();

    int processMouseEvent(rviz::ViewportMouseEvent & event);
    int processKeyEvent(QKeyEvent * event,rviz::RenderPanel * panel);

    virtual void update(float wall_dt,float ros_dt);

    private:

    void SendViewportData(const int32 start_x,const int32 start_y,
                          const int32 end_x,const int32 end_y,
                          const Ogre::Viewport * const viewport);

    void onSetEditMode(const std_msgs::UInt32 & mode);
    void onSetToolType(const std_msgs::UInt32 & type);

    void UpdateCursor();

    rviz::MoveTool * m_move_tool;
    rviz::InteractionTool * m_interaction_tool;

    uint32 m_edit_mode;
    bool m_edit_mode_selectable;
    uint32 m_tool_type;

    ros::NodeHandle m_nh;
    ros::Publisher m_annotation_selection_pub;
    ros::Subscriber m_on_set_mode_sub;
    ros::Subscriber m_on_set_tool_type_sub;

    bool m_selecting;
    int32 m_sel_start_x;
    int32 m_sel_start_y;

    bool m_moving;
    bool m_active;
  };

}

#endif // RVIZ_SELECT_TOOL_H
