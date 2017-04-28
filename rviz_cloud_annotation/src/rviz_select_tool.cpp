#include "rviz_select_tool.h"
#include "rviz_cloud_annotation.h"

#include <ros/time.h>

#include <rviz/default_plugin/tools/move_tool.h>
#include <rviz/default_plugin/tools/interaction_tool.h>

#include <rviz/selection/selection_manager.h>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/viewport_mouse_event.h>

#include <rviz_cloud_annotation/RectangleSelectionViewport.h>

#include <cmath>

namespace rviz_cloud_annotation
{
AnnotationSelectionTool::AnnotationSelectionTool()
  : m_nh("~")
  , m_selecting(false)
  , m_sel_start_x(0)
  , m_sel_start_y(0)
  , m_moving(false)
{
  access_all_keys_ = true;

  std::string param_string;

  m_nh.param<std::string>(PARAM_NAME_RECT_SELECTION_TOPIC,param_string,PARAM_DEFAULT_RECT_SELECTION_TOPIC);
  m_annotation_selection_pub = m_nh.advertise<rviz_cloud_annotation::RectangleSelectionViewport>(param_string,1);

  m_nh.param<std::string>(PARAM_NAME_SET_EDIT_MODE_TOPIC2,param_string,PARAM_DEFAULT_SET_EDIT_MODE_TOPIC2);
  m_on_set_mode_sub = m_nh.subscribe(param_string,1,&AnnotationSelectionTool::onSetEditMode,this);

  m_nh.param<std::string>(PARAM_NAME_TOOL_TYPE_TOPIC,param_string,PARAM_DEFAULT_TOOL_TYPE_TOPIC);
  m_on_set_tool_type_sub = m_nh.subscribe(param_string,1,&AnnotationSelectionTool::onSetToolType,this);

  m_move_tool = new rviz::MoveTool;
  m_interaction_tool = new rviz::InteractionTool;

  m_edit_mode = EDIT_MODE_NONE;
  m_edit_mode_selectable = false;
  m_tool_type = TOOL_TYPE_SINGLE_PICK;

  m_active = false;
}

AnnotationSelectionTool::~AnnotationSelectionTool()
{
  delete m_move_tool;
  delete m_interaction_tool;
}

void AnnotationSelectionTool::onInitialize()
{
  m_move_tool->initialize(context_);
  m_interaction_tool->initialize(context_);
}

void AnnotationSelectionTool::activate()
{
  setStatus("Annotation tool.");

  context_->getSelectionManager()->setTextureSize(512);
  m_selecting = false;
  m_moving = false;
  m_active = true;
  UpdateCursor();
}

void AnnotationSelectionTool::deactivate()
{
  context_->getSelectionManager()->removeHighlight();
  m_active = false;
}

void AnnotationSelectionTool::update(float wall_dt, float ros_dt)
{
  if (!m_selecting)
  {
    rviz::SelectionManager * sel_manager = context_->getSelectionManager();
    sel_manager->removeHighlight();
  }
}

void AnnotationSelectionTool::onSetEditMode(const std_msgs::UInt32 & mode)
{
  rviz::SelectionManager* sel_manager = context_->getSelectionManager();

  m_edit_mode = mode.data;
  m_edit_mode_selectable =
    m_edit_mode == EDIT_MODE_CONTROL_POINT ||
    m_edit_mode == EDIT_MODE_ERASER;

  if (!m_edit_mode_selectable)
  {
    sel_manager->removeHighlight();
    m_selecting = false;
  }

  if (m_active)
    UpdateCursor();
}

void AnnotationSelectionTool::onSetToolType(const std_msgs::UInt32 & type)
{
  m_tool_type = type.data;

  if (m_active)
    UpdateCursor();
}

int AnnotationSelectionTool::processKeyEvent(QKeyEvent * event,rviz::RenderPanel * panel)
{
  rviz::SelectionManager* sel_manager = context_->getSelectionManager();

  if(event->key() == Qt::Key_Insert)
  {
    m_moving = !m_moving;
    if (m_moving)
    {
      sel_manager->removeHighlight();
      m_selecting = false;
    }

    UpdateCursor();

    return Render;
  }

  return 0;
}

void AnnotationSelectionTool::UpdateCursor()
{
  if (m_moving)
  {
    setCursor(m_move_tool->getCursor());
  }
  else if (m_edit_mode_selectable &&
          (m_tool_type == TOOL_TYPE_SHALLOW_SQUARE || m_tool_type == TOOL_TYPE_DEEP_SQUARE))
  {
    setCursor(Qt::CrossCursor);
  }
  else if (m_edit_mode == EDIT_MODE_NONE)
  {
    setCursor(m_move_tool->getCursor());
  }
  else
  {
    setCursor(Qt::PointingHandCursor);
  }
}

int AnnotationSelectionTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  rviz::SelectionManager * sel_manager = context_->getSelectionManager();

  if ((m_tool_type == TOOL_TYPE_SINGLE_PICK || !m_edit_mode_selectable) &&
      (event.leftDown() || event.leftUp()))
  {
    return m_interaction_tool->processMouseEvent(event);
  }

  if (event.wheel_delta != 0 && !m_selecting)
  {
    return m_move_tool->processMouseEvent(event);
  }

  if (m_moving)
  {
    m_selecting = false;
    return m_move_tool->processMouseEvent(event);
  }

  if (m_edit_mode == EDIT_MODE_NONE)
  {
    return m_move_tool->processMouseEvent(event);
  }

  if (event.leftDown())
  {
    m_selecting = true;

    m_sel_start_x = event.x;
    m_sel_start_y = event.y;
  }

  if (m_selecting)
  {
    sel_manager->highlight(event.viewport,m_sel_start_x,m_sel_start_y,event.x,event.y);
    if(event.leftUp())
    {
      SendViewportData(m_sel_start_x,m_sel_start_y,event.x,event.y,event.viewport);
      m_selecting = false;
    }
    return Render;
  }

  return 0;
}

void AnnotationSelectionTool::SendViewportData(const int32 start_x,const int32 start_y,
                                         const int32 end_x,const int32 end_y,
                                         const Ogre::Viewport * const viewport)
{
  int32 x1 = std::min(start_x,end_x + 1);
  int32 x2 = std::max(start_x,end_x + 1);
  int32 y1 = std::min(start_y,end_y + 1);
  int32 y2 = std::max(start_y,end_y + 1);

  if (x1 < 0) x1 = 0;
  if (y1 < 0) y1 = 0;

  if (x1 > viewport->getActualWidth() - 1) x1 = viewport->getActualWidth() - 1;
  if (y1 > viewport->getActualHeight() - 1) y1 = viewport->getActualHeight() - 1;

  if (x2 < 0) x2 = 0;
  if (y2 < 0) y2 = 0;
  if (x2 > viewport->getActualWidth() - 1) x2 = viewport->getActualWidth() - 1;
  if (y2 > viewport->getActualHeight() - 1) y2 = viewport->getActualHeight() - 1;

  const Ogre::Matrix4 proj_matrix = viewport->getCamera()->getProjectionMatrix();
  const Ogre::Vector3 camera_position = viewport->getCamera()->getDerivedPosition();
  const Ogre::Quaternion camera_orientation = viewport->getCamera()->getDerivedOrientation();

  const float fovy = viewport->getCamera()->getFOVy().valueRadians();
  const float focal_length_px = viewport->getActualHeight() * std::tan((M_PI / 2) - (fovy / 2));

  rviz_cloud_annotation::RectangleSelectionViewport msg;

  for (uint32 y = 0; y < 4; y++)
    for (uint32 x = 0; x < 4; x++)
      msg.projection_matrix[x + y * 4] = proj_matrix[y][x];

  msg.camera_pose.orientation.x = camera_orientation.x;
  msg.camera_pose.orientation.y = camera_orientation.y;
  msg.camera_pose.orientation.z = camera_orientation.z;
  msg.camera_pose.orientation.w = camera_orientation.w;

  msg.camera_pose.position.x = camera_position.x;
  msg.camera_pose.position.y = camera_position.y;
  msg.camera_pose.position.z = camera_position.z;

  msg.viewport_width = viewport->getActualWidth();
  msg.viewport_height = viewport->getActualHeight();

  msg.focal_length = focal_length_px;

  msg.is_deep_selection = (m_tool_type == TOOL_TYPE_DEEP_SQUARE);

  msg.start_x = x1;
  msg.end_x = x2;
  msg.start_y = y1;
  msg.end_y = y2;

  m_annotation_selection_pub.publish(msg);
}

} // end namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_cloud_annotation::AnnotationSelectionTool,rviz::Tool)
