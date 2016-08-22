#ifndef RVIZ_CLOUD_ANNOTATION_PLUGIN_H
#define RVIZ_CLOUD_ANNOTATION_PLUGIN_H

#include <ros/ros.h>
#include <rviz/panel.h>

class QLabel;

namespace rviz_cloud_annotation
{

  class QRVizCloudAnnotation: public rviz::Panel
  {
    Q_OBJECT;

    public:
    QRVizCloudAnnotation(QWidget* parent = NULL);
    virtual ~QRVizCloudAnnotation();

    private Q_SLOTS:
    void onToggleEditMode();

    private:
    void SetCurrentEditMode(bool on);

    bool m_current_edit_mode_status;

    ros::NodeHandle m_nh;
    ros::Publisher m_set_edit_mode_pub;

    QLabel * m_edit_status_label;
  };

}

#endif // RVIZ_CLOUD_ANNOTATION_PLUGIN_H
