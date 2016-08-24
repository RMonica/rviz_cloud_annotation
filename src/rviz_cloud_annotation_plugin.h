#ifndef RVIZ_CLOUD_ANNOTATION_PLUGIN_H
#define RVIZ_CLOUD_ANNOTATION_PLUGIN_H

#include <ros/ros.h>
#include <rviz/panel.h>

#include <stdint.h>

class QLabel;
class QPushButton;
class QButtonGroup;

namespace rviz_cloud_annotation
{

  class QRVizCloudAnnotation: public rviz::Panel
  {
    Q_OBJECT;

    typedef uint64_t uint64;
    typedef std::vector<QPushButton *> PQPushButtonVector;

    public:
    QRVizCloudAnnotation(QWidget* parent = NULL);
    virtual ~QRVizCloudAnnotation();

    private Q_SLOTS:
    void onToggleEditMode(bool on);

    void onLabelButtonSelected(int id);
    void onPlusLabel();
    void onMinusLabel();
    void onPageUp();
    void onPageDown();

    void onSave();
    void onRestore();
    void onClear();

    private:
    void SetCurrentEditMode(bool on);

    void FillColorPageButtons();

    void SetCurrentLabel(const uint64 label,const uint64 page);
    uint64 GetPageForLabel(const uint64 label) const;
    uint64 GetLabelFromPageAndId(const uint64 page,const int id) const;
    uint64 GetFirstLabelForPage(const uint64 page) const;
    uint64 GetLastLabelForPage(const uint64 page) const;

    bool m_current_edit_mode_status;

    // 0 for the eraser
    uint64 m_current_label;
    uint64 m_current_page;

    ros::NodeHandle m_nh;
    ros::Publisher m_set_edit_mode_pub;
    ros::Publisher m_set_current_label_pub;

    ros::Publisher m_save_pub;
    ros::Publisher m_restore_pub;
    ros::Publisher m_clear_pub;

    QPushButton * m_edit_mode_button;
    QPushButton * m_eraser_button;

    QPushButton * m_prev_page_button;
    QPushButton * m_next_page_button;
    QLabel * m_current_page_label;
    QPushButton * m_next_label_button;
    QPushButton * m_prev_label_button;

    PQPushButtonVector m_page_buttons;
    QButtonGroup * m_page_button_group;

    uint64 m_color_cols_per_page;
    uint64 m_color_rows_per_page;
  };

}

#endif // RVIZ_CLOUD_ANNOTATION_PLUGIN_H
