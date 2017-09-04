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

#ifndef RVIZ_ORBIT3D_VIEW_CONTROLLER_H
#define RVIZ_ORBIT3D_VIEW_CONTROLLER_H

#include <OgreVector3.h>

#include <QCursor>

#include <rviz/frame_position_tracking_view_controller.h>

#include <stdint.h>

#include <Eigen/Dense>

namespace rviz
{
  class FloatProperty;
  class Shape;
  class SceneNode;
}

namespace rviz_3d_view_controller
{
  class Orbit3DViewController: public rviz::FramePositionTrackingViewController
  {
    Q_OBJECT
    public:
    typedef int32_t int32;
    typedef uint32_t uint32;

    Orbit3DViewController();
    virtual ~Orbit3DViewController();

    virtual void onInitialize();

    virtual void handleMouseEvent(rviz::ViewportMouseEvent& evt);

    virtual void lookAt(const Ogre::Vector3& point);

    virtual void reset();

    virtual void mimic(ViewController* source_view);

    protected:
    virtual void update(float dt,float ros_dt);
    virtual void onTargetFrameChanged(const Ogre::Vector3& old_reference_position,
                                      const Ogre::Quaternion& old_reference_orientation);

    virtual void updateCamera();

    void Rotate(const int32 x,const int32 y,const int32 dx,const int32 dy);
    void Translate(const int32 dx,const int32 dy);
    void TranslateZ(const int32 delta);
    void Zoom(const int32 delta);

    float Clamp(const float v,const float m,const float M) const;
    float IfFiniteOrElse(const float v,const float e) const;

    Eigen::Vector3f OgreToEigen(Ogre::Vector3 p) const;
    Eigen::Quaternionf OgreToEigen(Ogre::Quaternion q) const;
    Ogre::Vector3 EigenToOgre(Eigen::Vector3f p) const;
    Ogre::Quaternion EigenToOgre(Eigen::Quaternionf q) const;

    Eigen::Quaternionf ReadQuaternionProperties();
    Eigen::Vector3f ReadTranslationProperties();
    Eigen::Affine3f ReadTransformProperties();
    float ReadPivotDistanceProperty();
    float ReadRotationRateProperty();
    float ReadRotationSafeRadiusProperty();
    float ReadTranslationRateProperty();
    float ReadZoomRateProperty();

    void WriteQuaternionProperties(const Eigen::Quaternionf & quat);
    void WriteTranslationProperties(const Eigen::Vector3f & vec);
    void WriteTransformProperties(const Eigen::Affine3f & mat);
    void WritePivotDistanceProperty(const float distance);

    rviz::FloatProperty * m_qx_property;
    rviz::FloatProperty * m_qy_property;
    rviz::FloatProperty * m_qz_property;
    rviz::FloatProperty * m_qw_property;
    rviz::FloatProperty * m_x_property;
    rviz::FloatProperty * m_y_property;
    rviz::FloatProperty * m_z_property;
    rviz::FloatProperty * m_pivot_distance_property;

    rviz::FloatProperty * m_rotation_rate_property;
    rviz::FloatProperty * m_translation_rate_property;
    rviz::FloatProperty * m_zoom_rate_property;
    rviz::FloatProperty * m_rotation_safe_radius_property;

    rviz::Shape * m_pivot_shape;

    Eigen::Affine3f * m_default_pose;

    bool m_dragging;
  };

}

#endif // RVIZ_ORBIT3D_VIEW_CONTROLLER_H
