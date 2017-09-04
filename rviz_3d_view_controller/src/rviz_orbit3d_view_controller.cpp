/*
 * Copyright (c) 2017, Riccardo Monica
 *   RIMLab, Department of Engineering and Architecture
 *   University of Parma, Italy
 *   http://www.rimlab.ce.unipr.it/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "rviz_orbit3d_view_controller.h"

#include <cmath>

#include <rviz/display_context.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/uniform_string_stream.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/load_resource.h>
#include <rviz/render_panel.h>

#include <ros/ros.h>

#define MIN_PIVOT_DISTANCE 0.001
#define DEFAULT_PIVOT_DISTANCE 2.0

#define DEFAULT_SAFE_RADIUS 0.25
#define MIN_SAFE_RADIUS 0.01
#define MAX_SAFE_RADIUS (1.0 - MIN_SAFE_RADIUS)

#define DEFAULT_ZOOM_RATE 0.01
#define MIN_ZOOM_RATE 0.001
#define MAX_ZOOM_RATE 10.0

#define DEFAULT_ROTATION_RATE 0.3
#define MIN_ROTATION_RATE 0.01
#define MAX_ROTATION_RATE 10.0

#define DEFAULT_TRANSLATION_RATE 5.0
#define MIN_TRANSLATION_RATE 0.01
#define MAX_TRANSLATION_RATE 100.0

namespace rviz_3d_view_controller
{

template <typename T>
  T SQR(const T & t) {return t * t; }

float Orbit3DViewController::Clamp(const float v,const float m,const float M) const
{
  if (!std::isfinite(v))
    return M;
  return std::min(std::max(v,m),M);
}

float Orbit3DViewController::IfFiniteOrElse(const float v,const float e) const
{
  if (std::isfinite(v))
    return v;
  return e;
}

Eigen::Vector3f Orbit3DViewController::OgreToEigen(Ogre::Vector3 p) const
{
  Eigen::Vector3f result;
  result.x() = p.x;
  result.y() = p.y;
  result.z() = p.z;
  return result;
}

Eigen::Quaternionf Orbit3DViewController::OgreToEigen(Ogre::Quaternion q) const
{
  Eigen::Quaternionf result;
  result.x() = q.x;
  result.y() = q.y;
  result.z() = q.z;
  result.w() = q.w;
  return result;
}

Ogre::Vector3 Orbit3DViewController::EigenToOgre(Eigen::Vector3f p) const
{
  Ogre::Vector3 result;
  result.x = p.x();
  result.y = p.y();
  result.z = p.z();
  return result;
}

Ogre::Quaternion Orbit3DViewController::EigenToOgre(Eigen::Quaternionf q) const
{
  Ogre::Quaternion result;
  result.x = q.x();
  result.y = q.y();
  result.z = q.z();
  result.w = q.w();
  return result;
}

Eigen::Quaternionf Orbit3DViewController::ReadQuaternionProperties()
{
  Eigen::Quaternionf quat;
  quat.x() = Clamp(m_qx_property->getFloat(),-1.0,1.0);
  quat.y() = Clamp(m_qy_property->getFloat(),-1.0,1.0);
  quat.z() = Clamp(m_qz_property->getFloat(),-1.0,1.0);
  quat.w() = Clamp(m_qw_property->getFloat(),-1.0,1.0);

  quat.normalize();

  return quat;
}

Eigen::Vector3f Orbit3DViewController::ReadTranslationProperties()
{
  Eigen::Vector3f result;
  result.x() = IfFiniteOrElse(m_x_property->getFloat(),0.0);
  result.y() = IfFiniteOrElse(m_y_property->getFloat(),0.0);
  result.z() = IfFiniteOrElse(m_z_property->getFloat(),0.0);

  return result;
}

Eigen::Affine3f Orbit3DViewController::ReadTransformProperties()
{
  Eigen::Affine3f result;
  result.linear() = ReadQuaternionProperties().matrix();
  result.translation() = ReadTranslationProperties();
  return result;
}

float Orbit3DViewController::ReadRotationRateProperty()
{
  return Clamp(m_rotation_rate_property->getFloat(),MIN_ROTATION_RATE,MAX_ROTATION_RATE);
}

float Orbit3DViewController::ReadZoomRateProperty()
{
  return Clamp(m_zoom_rate_property->getFloat(),MIN_ZOOM_RATE,MAX_ZOOM_RATE);
}

float Orbit3DViewController::ReadTranslationRateProperty()
{
  return Clamp(m_translation_rate_property->getFloat(),MIN_TRANSLATION_RATE,MAX_TRANSLATION_RATE);
}

float Orbit3DViewController::ReadRotationSafeRadiusProperty()
{
  return Clamp(m_rotation_safe_radius_property->getFloat(),MIN_SAFE_RADIUS,MAX_SAFE_RADIUS);
}

float Orbit3DViewController::ReadPivotDistanceProperty()
{
  return std::max<float>(m_pivot_distance_property->getFloat(),MIN_PIVOT_DISTANCE);
}

void Orbit3DViewController::WritePivotDistanceProperty(const float distance)
{
  m_pivot_distance_property->setFloat(distance);
}

void Orbit3DViewController::WriteQuaternionProperties(const Eigen::Quaternionf & quat)
{
  Eigen::Quaternionf quat2 = quat.normalized();
  m_qx_property->setFloat(quat2.x());
  m_qy_property->setFloat(quat2.y());
  m_qz_property->setFloat(quat2.z());
  m_qw_property->setFloat(quat2.w());
}

void Orbit3DViewController::WriteTranslationProperties(const Eigen::Vector3f & vec)
{
  m_x_property->setFloat(vec.x());
  m_y_property->setFloat(vec.y());
  m_z_property->setFloat(vec.z());
}

void Orbit3DViewController::WriteTransformProperties(const Eigen::Affine3f & mat)
{
  WriteQuaternionProperties(Eigen::Quaternionf(mat.linear()));
  WriteTranslationProperties(mat.translation());
}

Orbit3DViewController::Orbit3DViewController()
{
  m_dragging = false;

  const Eigen::Quaternionf default_quat(0.353553,0.146447,0.353553,0.853553);
  const Eigen::Vector3f default_transl(4.0,4.0,5.65685);
  m_default_pose = new Eigen::Affine3f(Eigen::Affine3f::Identity());
  m_default_pose->linear() = default_quat.matrix();
  m_default_pose->translation() = default_transl;

  m_qx_property = new rviz::FloatProperty("Quaternion X",default_quat.x(),"Part of rotation quaternion",this);
  m_qx_property->setMin(-1.0);
  m_qx_property->setMax(1.0);

  m_qy_property = new rviz::FloatProperty("Quaternion Y",default_quat.y(),"Part of rotation quaternion",this);
  m_qy_property->setMin(-1.0);
  m_qy_property->setMax(1.0);

  m_qz_property = new rviz::FloatProperty("Quaternion Z",default_quat.z(),"Part of rotation quaternion",this);
  m_qz_property->setMin(-1.0);
  m_qz_property->setMax(1.0);

  m_qw_property = new rviz::FloatProperty("Quaternion W",default_quat.w(),"Part of rotation quaternion",this);
  m_qw_property->setMin(-1.0);
  m_qw_property->setMax(1.0);

  m_rotation_rate_property = new rviz::FloatProperty("Rotation rate",DEFAULT_ROTATION_RATE,
                                                     "Horizontal/vertical rotation rate",this);
  m_rotation_rate_property->setMin(MIN_ROTATION_RATE);
  m_rotation_rate_property->setMax(MAX_ROTATION_RATE);

  m_translation_rate_property = new rviz::FloatProperty("Translation rate",DEFAULT_TRANSLATION_RATE,"Horizontal/vertical translation rate",this);
  m_translation_rate_property->setMin(MIN_TRANSLATION_RATE);
  m_translation_rate_property->setMax(MAX_TRANSLATION_RATE);

  m_pivot_distance_property = new rviz::FloatProperty("Pivot distance",DEFAULT_PIVOT_DISTANCE,
                                                      "Pivot distance from camera",this);
  m_pivot_distance_property->setMin(MIN_PIVOT_DISTANCE);

  m_zoom_rate_property = new rviz::FloatProperty("Zoom rate",DEFAULT_ZOOM_RATE,"Zoom movement speed in Z",this);
  m_zoom_rate_property->setMin(MIN_ZOOM_RATE);
  m_zoom_rate_property->setMax(MAX_ZOOM_RATE);

  m_rotation_safe_radius_property = new rviz::FloatProperty("Rotation safe radius",DEFAULT_SAFE_RADIUS,
                                        "Inside this radius, no Z rotation will take place.",this);
  m_rotation_safe_radius_property->setMin(MIN_SAFE_RADIUS);
  m_rotation_safe_radius_property->setMax(MAX_SAFE_RADIUS);

  m_x_property = new rviz::FloatProperty("Translation X",default_transl.x(),"Part of translation vector",this);
  m_y_property = new rviz::FloatProperty("Translation Y",default_transl.y(),"Part of translation vector",this);
  m_z_property = new rviz::FloatProperty("Translation Z",default_transl.z(),"Part of translation vector",this);
}

void Orbit3DViewController::onInitialize()
{
  rviz::FramePositionTrackingViewController::onInitialize();

  camera_->setProjectionType(Ogre::PT_PERSPECTIVE);

  m_pivot_shape = new rviz::Shape(rviz::Shape::Sphere,context_->getSceneManager(),target_scene_node_);
  m_pivot_shape->setColor(1.0f,1.0f,0.0f,0.5f);
  m_pivot_shape->setScale(Ogre::Vector3(0.1,0.1,0.05));
  m_pivot_shape->getRootNode()->setVisible(false);
  WriteTransformProperties(*m_default_pose);
  WritePivotDistanceProperty(DEFAULT_PIVOT_DISTANCE);

  m_dragging = false;
}

Orbit3DViewController::~Orbit3DViewController()
{
  delete m_default_pose;
}

void Orbit3DViewController::reset()
{
  m_dragging = false;

  WriteTransformProperties(*m_default_pose);
  WritePivotDistanceProperty(DEFAULT_PIVOT_DISTANCE);
}

void Orbit3DViewController::Rotate(const int32 x,const int32 y,const int32 dx,const int32 dy)
{
  const int32 width = camera_->getViewport()->getActualWidth();
  const int32 height = camera_->getViewport()->getActualHeight();
  const int32 hwidth = width / 2;
  const int32 hheight = height / 2;
  if (hwidth <= 0 || hheight <= 0)
    return;
  const int32 mhw = std::min(hwidth,hheight);

  if (!dx && !dy)
    return;
  if (!x && !y)
    return;
  if (x == dx && y == dy)
    return;

  const Eigen::Vector2f curr_2d(float(x - hwidth) / mhw,float(y - hheight) / mhw);
  const Eigen::Vector2f diff_2d(float(dx) / mhw,float(dy) / mhw);
  const Eigen::Vector2f prev_2d = curr_2d - diff_2d;

  const Eigen::Vector2f curr_rel_2d(float(x - hwidth) / hwidth,float(y - hheight) / hheight);

  const float rotation_rate = ReadRotationRateProperty();
  const float safe_radius = ReadRotationSafeRadiusProperty();
  const float mult_radius = curr_rel_2d.norm();

  float effect_multiplier = 0.0;
  if (mult_radius > 1.0 - safe_radius)
    effect_multiplier = 0.0;
  else if (mult_radius < 1.0 - 2.0 * safe_radius)
    effect_multiplier = 1.0;
  else
    effect_multiplier = ((1.0 - safe_radius) - mult_radius) / safe_radius;

  // z rotation
  Eigen::Affine3f z_rot = Eigen::Affine3f::Identity();
  if (effect_multiplier != 1.0)
  {
    const float z_angle_prev = std::atan2(prev_2d.y(),prev_2d.x());
    const float z_angle_curr = std::atan2(curr_2d.y(),curr_2d.x());
    float z_angle_diff = z_angle_curr - z_angle_prev;
    if (std::abs(z_angle_diff + M_PI * 2.0) < std::abs(z_angle_diff))
      z_angle_diff = z_angle_diff + M_PI * 2.0;
    if (std::abs(z_angle_diff - M_PI * 2.0) < std::abs(z_angle_diff))
      z_angle_diff = z_angle_diff - M_PI * 2.0;

    const Eigen::Vector3f axis = Eigen::Vector3f(prev_2d.y(),prev_2d.x(),0.0).normalized();
    const float angle = Eigen::Vector3f(diff_2d.x(),diff_2d.y(),0.0).dot(Eigen::Vector3f(prev_2d.x(),prev_2d.y(),0.0));

    const float multiplier = 1.0 - effect_multiplier;

    z_rot.linear() = Eigen::AngleAxisf(-angle * multiplier,axis).matrix() *
                     Eigen::AngleAxisf(z_angle_diff * multiplier,Eigen::Vector3f::UnitZ()).matrix();
  }

  // xy rotation
  Eigen::Affine3f xy_rot = Eigen::Affine3f::Identity();
  if (effect_multiplier != 0.0)
  {
    const Eigen::Vector3f axis = Eigen::Vector3f(diff_2d.y(),diff_2d.x(),0.0).normalized();
    const float angle = diff_2d.norm() * M_PI * rotation_rate;
    const float multiplier = effect_multiplier;

    xy_rot.linear() = Eigen::AngleAxisf(-angle * multiplier,axis).matrix();
  }

  const Eigen::Affine3f prev_transform = ReadTransformProperties();
  const Eigen::Affine3f new_transform = prev_transform * xy_rot * z_rot;
  WriteTransformProperties(new_transform);
}

void Orbit3DViewController::Translate(const int32 dx,const int32 dy)
{
  if (!dx && !dy)
    return;

  const int32 width = camera_->getViewport()->getActualWidth();
  const int32 height = camera_->getViewport()->getActualHeight();
  const float translation_rate = ReadTranslationRateProperty();
  const float pivot_distance = ReadPivotDistanceProperty();

  const int32 min_wh = std::min(width,height);

  const Eigen::Affine3f prev_transform = ReadTransformProperties();

  const Eigen::Vector3f translation_vector(float(-dx) / min_wh,float(dy) / min_wh,0.0);
  Eigen::Affine3f new_transform = prev_transform;
  new_transform.translation() += prev_transform.linear() *
    translation_vector * translation_rate *
    (pivot_distance / DEFAULT_PIVOT_DISTANCE);

  WriteTransformProperties(new_transform);
}

void Orbit3DViewController::TranslateZ(const int32 delta)
{
  if (!delta)
    return;

  const Eigen::Affine3f prev_transform = ReadTransformProperties();
  const float pivot_distance = ReadPivotDistanceProperty();

  const Eigen::Vector3f translation_vector(0.0,0.0,-delta);
  Eigen::Affine3f new_transform = prev_transform;
  new_transform.translation() += prev_transform.linear() * translation_vector * 0.001 *
    (pivot_distance / DEFAULT_PIVOT_DISTANCE);

  WriteTransformProperties(new_transform);
}

void Orbit3DViewController::Zoom(const int32 delta)
{
  if (!delta)
    return;

  const float zoom_rate = ReadZoomRateProperty();
  const float prev_distance = ReadPivotDistanceProperty();
  const float new_distance = prev_distance * std::pow(0.9,delta * zoom_rate);
  WritePivotDistanceProperty(new_distance);
}

void Orbit3DViewController::handleMouseEvent(rviz::ViewportMouseEvent& event)
{
  if (event.shift())
  {
    setStatus("<b>Left-Click:</b> Move X/Y.  <b>Mouse Wheel:</b> Move Z.");
  }
  else
  {
    setStatus("<b>Left-Click:</b> Rotate.  <b>Middle-Click:</b> Move X/Y.  "
              "<b>Mouse Wheel:</b> Zoom.  <b>Shift:</b> More options.");
  }

  int32 diff_x = 0;
  int32 diff_y = 0;

  bool moved = false;

  if (event.type == QEvent::MouseButtonPress)
  {
    m_pivot_shape->getRootNode()->setVisible(true);
    moved = true;

    m_dragging = true;
  }
  else if (event.type == QEvent::MouseButtonRelease)
  {
    m_pivot_shape->getRootNode()->setVisible(false);
    moved = true;
    m_dragging = false;
  }
  else if (m_dragging && event.type == QEvent::MouseMove)
  {
    diff_x = event.x - event.last_x;
    diff_y = event.y - event.last_y;
    moved = true;
  }

  // regular left-button drag
  if (event.left() && !event.shift())
  {
    setCursor(Rotate3D);
    Rotate(event.x,event.y,diff_x,diff_y);
  }
  // middle or shift-left drag
  else if (event.middle() || (event.shift() && event.left()))
  {
    setCursor(MoveXY);
    Translate(diff_x,diff_y);
  }
  else
  {
    setCursor(event.shift() ? MoveXY : Rotate3D);
  }

  moved = true;

  if (event.wheel_delta != 0)
  {
    const int32 diff = event.wheel_delta;
    if (event.shift())
      TranslateZ(diff);
    else
      Zoom(diff);
    moved = true;
  }

  if (moved)
  {
    context_->queueRender();
  }
}

void Orbit3DViewController::mimic(ViewController* source_view)
{
  FramePositionTrackingViewController::mimic(source_view);

  Ogre::Camera* source_camera = source_view->getCamera();
  Ogre::Vector3 position = source_camera->getPosition();
  Ogre::Quaternion orientation = source_camera->getOrientation();
  const Eigen::Quaternionf eo = OgreToEigen(orientation);

  WriteQuaternionProperties(eo);
  WriteTranslationProperties(OgreToEigen(position) - eo.matrix() * Eigen::Vector3f::UnitZ() * DEFAULT_PIVOT_DISTANCE);
  WritePivotDistanceProperty(DEFAULT_PIVOT_DISTANCE);
}

void Orbit3DViewController::update(float dt,float ros_dt)
{
  FramePositionTrackingViewController::update(dt,ros_dt);
  updateCamera();
}

void Orbit3DViewController::lookAt(const Ogre::Vector3& point)
{
  ROS_WARN("Orbit3DViewController: lookAt: not yet implemented.");
}

void Orbit3DViewController::onTargetFrameChanged(const Ogre::Vector3& old_reference_position,
                                                const Ogre::Quaternion& old_reference_orientation)
{
  Eigen::Affine3f prev_base;
  prev_base.linear() = OgreToEigen(old_reference_orientation).matrix();
  prev_base.translation() = OgreToEigen(old_reference_position);

  Eigen::Affine3f curr_base;
  curr_base.linear() = OgreToEigen(reference_orientation_).matrix();
  curr_base.translation() = OgreToEigen(reference_position_);

  // FIXME: this should be done by parent FramePositionTrackingViewController, but currently broken
  {
    target_scene_node_->setOrientation(reference_orientation_);
    target_scene_node_->setPosition(reference_position_);
    context_->queueRender();
  }

  const Eigen::Affine3f transform = ReadTransformProperties();
  const Eigen::Affine3f new_transform = curr_base.inverse() * prev_base * transform;
  WriteTransformProperties(new_transform);
}

void Orbit3DViewController::updateCamera()
{
  const Eigen::Affine3f pivot_pose = ReadTransformProperties();
  const float pivot_distance = ReadPivotDistanceProperty();
  const Eigen::Vector3f pivot_transl = pivot_distance * Eigen::Vector3f::UnitZ();
  const Eigen::Affine3f camera_pose = pivot_pose * Eigen::Translation<float,3>(pivot_transl);

  camera_->setPosition(EigenToOgre(camera_pose.translation()));
  camera_->setOrientation(EigenToOgre(Eigen::Quaternionf(camera_pose.linear())));

  m_pivot_shape->setPosition(EigenToOgre(pivot_pose.translation()));
}

} // end namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_3d_view_controller::Orbit3DViewController,rviz::ViewController)
