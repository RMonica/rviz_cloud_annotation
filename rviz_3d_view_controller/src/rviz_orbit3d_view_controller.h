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
