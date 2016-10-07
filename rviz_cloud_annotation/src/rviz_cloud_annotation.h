/*
 * Copyright (c) 2016, Riccardo Monica
 */

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

#define PARAM_NAME_LABEL_NAMES_FILENAME       "label_names_filename"
#define PARAM_DEFAULT_LABEL_NAMES_FILENAME    "names.txt"

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

// from RViz to backend
#define PARAM_NAME_SET_NAME_TOPIC             "rviz_cloud_annotation/set_name_topic"
#define PARAM_DEFAULT_SET_NAME_TOPIC          "/rviz_cloud_annotation/set_name"

// from backend to RViz
#define PARAM_NAME_SET_NAME_TOPIC2            "rviz_cloud_annotation/set_name_topic2"
#define PARAM_DEFAULT_SET_NAME_TOPIC2         "/rviz_cloud_annotation/set_name2"

// from backend to RViz
#define PARAM_NAME_POINT_COUNT_UPDATE_TOPIC   "rviz_cloud_annotation/point_count_update"
#define PARAM_DEFAULT_POINT_COUNT_UPDATE_TOPIC "/rviz_cloud_annotation/point_count_update"

#define PARAM_NAME_VIEW_LABEL_TOPIC           "rviz_cloud_annotation/view_labels_topic"
#define PARAM_DEFAULT_VIEW_LABEL_TOPIC        "/rviz_cloud_annotation/view_labels"

#define PARAM_NAME_VIEW_CONTROL_POINTS_TOPIC  "rviz_cloud_annotation/view_control_points_topic"
#define PARAM_DEFAULT_VIEW_CONTROL_POINTS_TOPIC "/rviz_cloud_annotation/view_control_points"

#define PARAM_NAME_VIEW_CLOUD_TOPIC           "rviz_cloud_annotation/view_cloud_topic"
#define PARAM_DEFAULT_VIEW_CLOUD_TOPIC        "/rviz_cloud_annotation/view_cloud"

#define PARAM_NAME_UNDO_TOPIC                 "rviz_cloud_annotation/undo_topic"
#define PARAM_DEFAULT_UNDO_TOPIC              "/rviz_cloud_annotation/undo"

#define PARAM_NAME_REDO_TOPIC                 "rviz_cloud_annotation/redo_topic"
#define PARAM_DEFAULT_REDO_TOPIC              "/rviz_cloud_annotation/redo"

#define PARAM_NAME_UNDO_REDO_STATE_TOPIC      "rviz_cloud_annotation/undo_redo_state_topic"
#define PARAM_DEFAULT_UNDO_REDO_STATE_TOPIC   "/rviz_cloud_annotation/undo_redo_state"

// parameters for smart labeling
  // neighborhood graph distance
#define PARAM_NAME_NEIGH_SEARCH_DISTANCE      "neighborhood_search_distance"
#define PARAM_DEFAULT_NEIGH_SEARCH_DISTANCE   (0.02)

#define PARAM_NAME_NEIGH_SEARCH_TYPE          "neigh_search_type"
#define PARAM_DEFAULT_NEIGH_SEARCH_TYPE       (0)
#define PARAM_VALUE_NEIGH_SEARCH_FIXED_DISTANCE (0)
#define PARAM_VALUE_NEIGH_SEARCH_KNN            (1)

#define PARAM_NAME_NEIGH_SEARCH_PARAMS        "neigh_search_params"
#define PARAM_DEFAULT_NEIGH_SEARCH_PARAMS     ""

  // max label size per control point
#define PARAM_NAME_MAX_DISTANCE               "max_label_distance"
#define PARAM_DEFAULT_MAX_DISTANCE            (0.1)

#define PARAM_NAME_COLOR_IMPORTANCE           "color_importance"
#define PARAM_DEFAULT_COLOR_IMPORTANCE        (0.0)

#define PARAM_NAME_POSITION_IMPORTANCE        "position_importance"
#define PARAM_DEFAULT_POSITION_IMPORTANCE     (1.0)

#define PARAM_NAME_NORMAL_IMPORTANCE          "normal_importance"
#define PARAM_DEFAULT_NORMAL_IMPORTANCE       (0.0)
// end parameters for smart labeling

#define EDIT_MODE_NONE                        (0)
#define EDIT_MODE_CONTROL_POINT               (1)
#define EDIT_MODE_ERASER                      (2)
#define EDIT_MODE_COLOR_PICKER                (3)
#define EDIT_MODE_MAX                         (4)

#endif // RVIZ_CLOUD_ANNOTATION_H
