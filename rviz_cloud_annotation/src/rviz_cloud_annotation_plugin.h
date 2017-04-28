/*
 * Copyright (c) 2016, Riccardo Monica
 */

#ifndef RVIZ_CLOUD_ANNOTATION_PLUGIN_H
#define RVIZ_CLOUD_ANNOTATION_PLUGIN_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt64MultiArray.h>
#include <std_msgs/Empty.h>

#include <stdint.h>
#include <vector>

#include <rviz_cloud_annotation/UndoRedoState.h>

class QLabel;
class QPushButton;
class QButtonGroup;
class QLineEdit;
class QAction;
class QToolButton;
class QSlider;
class QMenu;

namespace pcl { class RGB; }

namespace rviz_cloud_annotation
{

  class QRVizCloudAnnotation: public rviz::Panel
  {
    Q_OBJECT;

    typedef uint64_t uint64;
    typedef int64_t int64;
    typedef uint32_t uint32;
    typedef std::vector<uint64> Uint64Vector;
    typedef std::vector<QPushButton *> PQPushButtonVector;

    public:
    QRVizCloudAnnotation(QWidget* parent = NULL);
    virtual ~QRVizCloudAnnotation();

    private Q_SLOTS:
    void onSetEditMode(int mode);
    void onSetToolType(int type);

    void onLabelButtonSelected(int id);
    void onPlusLabel();
    void onMinusLabel();
    void onPageUp();
    void onPageDown();

    void onSave();
    void onRestore();
    void onClear();
    void onClearCurrent();

    void onUndo();
    void onRedo();

    void onSendName();

    void onViewCloudToggled(const bool checked);
    void onViewControlPointsToggled(const bool checked);
    void onViewLabelsToggled(const bool checked);

    void onGotoFirstUnused();
    void onGotoLastUnused();
    void onGotoFirst();
    void onGotoNextUnused();

    void onSmallerPoints();
    void onBiggerPoints();
    void onResetPointsSize();

    void onControlPointWeightSliderMoved(int new_value);
    void onControlPointWeightSliderSet(int new_value);
    void onControlPointWeightInc();
    void onControlPointWeightDec();
    void onControlPointWeightMax();
    void onControlPointWeightMin();
    void onSetControlPointMaxWeight(const std_msgs::UInt32 & msg);

    private:
    void SetCurrentEditMode(const uint64 mode);

    void FillColorPageButtons();
    void FillPointCounts();
    void FillColorPageButtonStylesheet();

    void SetCurrentLabel(const uint64 label,const uint64 page);

    void onSetCurrentLabel(const std_msgs::UInt32 & label);
    void onSetEditMode2(const std_msgs::UInt32 & mode);
    void onPointCountUpdate(const std_msgs::UInt64MultiArray & counters);
    void onUndoRedoState(const rviz_cloud_annotation::UndoRedoState & msg);

    void onSetName(const std_msgs::String & name);

    uint64 GetPageForLabel(const uint64 label) const;
    uint64 GetLabelFromPageAndId(const uint64 page,const int id) const;
    uint64 GetFirstLabelForPage(const uint64 page) const;
    uint64 GetLastLabelForPage(const uint64 page) const;

    void SetUndoText(const std::string & text);
    void SetRedoText(const std::string & text);

    static void ColorToHex(const pcl::RGB & color,char hex[7]);

    uint64 m_current_edit_mode;

    // 0 for the eraser
    uint64 m_current_label;
    uint64 m_current_page;

    ros::NodeHandle m_nh;
    ros::Publisher m_set_edit_mode_pub;
    ros::Publisher m_set_current_label_pub;

    ros::Publisher m_save_pub;
    ros::Publisher m_restore_pub;
    ros::Publisher m_clear_pub;

    ros::Publisher m_goto_first_unused_pub;
    ros::Publisher m_goto_last_unused_pub;
    ros::Publisher m_goto_first_pub;
    ros::Publisher m_goto_next_unused_pub;

    ros::Subscriber m_set_edit_mode_sub;
    ros::Subscriber m_set_current_label_sub;

    ros::Publisher m_set_name_pub;
    ros::Subscriber m_set_name_sub;

    ros::Publisher m_view_cloud_pub;
    ros::Publisher m_view_labels_pub;
    ros::Publisher m_view_control_points_pub;

    ros::Publisher m_redo_pub;
    ros::Publisher m_undo_pub;
    ros::Subscriber m_undo_redo_state_sub;

    ros::Publisher m_point_size_change_pub;

    ros::Publisher m_control_points_weight_pub;
    ros::Subscriber m_control_point_max_weight_sub;

    ros::Publisher m_tool_type_pub;

    QPushButton * m_edit_none_button;
    QPushButton * m_edit_control_point_button;
    QPushButton * m_edit_eraser_button;
    QPushButton * m_edit_color_picker_button;
    QButtonGroup * m_toolbar_group;

    QPushButton * m_tool_single_button;
    QPushButton * m_tool_shallow_square_button;
    QPushButton * m_tool_deep_square_button;
    QButtonGroup * m_tooltype_group;

    QAction * m_prev_page_action;
    QAction * m_next_page_action;
    QAction * m_next_label_action;
    QAction * m_prev_label_action;

    QAction * m_prev_weight_action;
    QAction * m_next_weight_action;
    QAction * m_min_weight_action;
    QAction * m_max_weight_action;
    QMenu * m_weight_menu;

    QAction * m_undo_action;
    QAction * m_redo_action;

    QLabel * m_current_page_label;

    QLabel * m_current_control_point_weight_label;
    QSlider * m_current_control_point_weight_slider;
    uint32 m_control_point_weight_max;

    PQPushButtonVector m_page_buttons;
    QButtonGroup * m_page_button_group;

    Uint64Vector m_point_counters;
    ros::Subscriber m_point_count_update_sub;

    QLineEdit * m_set_name_edit;

    uint64 m_color_cols_per_page;
    uint64 m_color_rows_per_page;
  };

}

#endif // RVIZ_CLOUD_ANNOTATION_PLUGIN_H
