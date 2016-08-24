#ifndef RVIZ_CLOUD_ANNOTATION_H
#define RVIZ_CLOUD_ANNOTATION_H

#define PARAM_NAME_UPDATE_TOPIC                "update_topic"
#define PARAM_DEFAULT_UPDATE_TOPIC             "/rviz_cloud_annotation/update_topic"

#define PARAM_NAME_CLOUD_FILENAME             "cloud_filename"
#define PARAM_DEFAULT_CLOUD_FILENAME          "cloud.pcd"

// will attempt to load this at startup
#define PARAM_NAME_ANN_FILENAME_IN            "annotation_read_filename"
#define PARAM_DEFAULT_ANN_FILENAME_IN         "annotation.annotation"

// will save to this when asked by the GUI
#define PARAM_NAME_ANN_FILENAME_OUT           "annotation_write_filename"
#define PARAM_DEFAULT_ANN_FILENAME_OUT        "annotation.annotation"

// will also save this cloud (XYZRGBL)
#define PARAM_NAME_ANNOTATED_CLOUD            "annotation_cloud"
#define PARAM_DEFAULT_ANNOTATED_CLOUD         "annotation.pcd"

#define PARAM_NAME_FRAME_ID                   "frame_id"
#define PARAM_DEFAULT_FRAME_ID                "base_link"

#define PARAM_NAME_POINT_SIZE                 "point_size"
#define PARAM_DEFAULT_POINT_SIZE              (0.005)

// size of the current label for each point
#define PARAM_NAME_LABEL_SIZE                 "label_size"
#define PARAM_DEFAULT_LABEL_SIZE              (0.0025)

// size of the current label for control points
#define PARAM_NAME_CONTROL_LABEL_SIZE         "control_label_size"
#define PARAM_DEFAULT_CONTROL_LABEL_SIZE      (0.02)

// from interface to backend
#define PARAM_NAME_SET_EDIT_MODE_TOPIC        "rviz_cloud_annotation/set_edit_mode_topic"
#define PARAM_DEFAULT_SET_EDIT_MODE_TOPIC     "/rviz_cloud_annotation/set_edit_mode"

// from backend to interface
#define PARAM_NAME_SET_EDIT_MODE_TOPIC2       "rviz_cloud_annotation/set_edit_mode_topic2"
#define PARAM_DEFAULT_SET_EDIT_MODE_TOPIC2    "/rviz_cloud_annotation/set_edit_mode2"

#define PARAM_NAME_COLORS_COLS_PER_PAGE       "rviz_cloud_annotation/color_columns_per_page"
#define PARAM_DEFAULT_COLOR_COLS_PER_PAGE     (10)

#define PARAM_NAME_COLORS_ROWS_PER_PAGE       "rviz_cloud_annotation/color_rows_per_page"
#define PARAM_DEFAULT_COLOR_ROWS_PER_PAGE     (2)

#define PARAM_NAME_SET_CURRENT_LABEL_TOPIC    "rviz_cloud_annotation/set_current_label_topic"
#define PARAM_DEFAULT_SET_CURRENT_LABEL_TOPIC "/rviz_cloud_annotation/set_current_label"

#define PARAM_NAME_CURRENT_LABEL_TOPIC        "rviz_cloud_annotation/set_current_label_topic2"
#define PARAM_DEFAULT_CURRENT_LABEL_TOPIC     "/rviz_cloud_annotation/set_current_label2"

#define PARAM_NAME_SAVE_TOPIC                 "rviz_cloud_annotation/save_topic"
#define PARAM_DEFAULT_SAVE_TOPIC              "/rviz_cloud_annotation/save"

#define PARAM_NAME_RESTORE_TOPIC              "rviz_cloud_annotation/restore_topic"
#define PARAM_DEFAULT_RESTORE_TOPIC           "/rviz_cloud_annotation/restore"

#define PARAM_NAME_CLEAR_TOPIC                "rviz_cloud_annotation/clear_topic"
#define PARAM_DEFAULT_CLEAR_TOPIC             "/rviz_cloud_annotation/clear"

#define EDIT_MODE_NONE                        (0)
#define EDIT_MODE_CONTROL_POINT               (1)
#define EDIT_MODE_ERASER                      (2)
#define EDIT_MODE_COLOR_PICKER                (3)
#define EDIT_MODE_MAX                         (4)

#endif // RVIZ_CLOUD_ANNOTATION_H
