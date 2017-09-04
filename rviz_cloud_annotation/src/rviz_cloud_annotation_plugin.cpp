/*
 * Copyright (c) 2016-2017, Riccardo Monica
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

#include "rviz_cloud_annotation_plugin.h"
#include "rviz_cloud_annotation.h"
#include <rviz_cloud_annotation/UndoRedoState.h>

// QT
#include <QShortcut>
#include <QLabel>
#include <QPushButton>
#include <QKeySequence>
#include <QBoxLayout>
#include <QGridLayout>
#include <QButtonGroup>
#include <QMessageBox>
#include <QLineEdit>
#include <QMenuBar>
#include <QMenu>
#include <QAction>
#include <QToolButton>
#include <QStyle>
#include <QSlider>

// STL
#include <iostream>
#include <string>
#include <sstream>

// ROS
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

// PCL
#include <pcl/common/colors.h>
#include <pcl/point_types.h>

namespace rviz_cloud_annotation
{

  QRVizCloudAnnotation::QRVizCloudAnnotation(QWidget * parent): rviz::Panel(parent), m_nh("~")
  {
    {
      std::string param_string;
      int temp_int;

      m_nh.param<std::string>(PARAM_NAME_SET_EDIT_MODE_TOPIC,param_string,PARAM_DEFAULT_SET_EDIT_MODE_TOPIC);
      m_set_edit_mode_pub = m_nh.advertise<std_msgs::UInt32>(param_string,1);

      m_nh.param<std::string>(PARAM_NAME_SET_EDIT_MODE_TOPIC2,param_string,PARAM_DEFAULT_SET_EDIT_MODE_TOPIC2);
      m_set_edit_mode_sub = m_nh.subscribe(param_string,1,&QRVizCloudAnnotation::onSetEditMode2,this);

      m_current_page = 0;

      m_nh.param<int>(PARAM_NAME_COLORS_COLS_PER_PAGE,temp_int,PARAM_DEFAULT_COLOR_COLS_PER_PAGE);
      if (temp_int > 0 && temp_int < 30)
        m_color_cols_per_page = temp_int;
      else
      {
        m_color_cols_per_page = PARAM_DEFAULT_COLOR_COLS_PER_PAGE;
        ROS_WARN("rviz_cloud_annotation_plugin: %s is out of sane range: %d, using default.",
                 PARAM_NAME_COLORS_COLS_PER_PAGE,temp_int);
      }

      m_nh.param<int>(PARAM_NAME_COLORS_ROWS_PER_PAGE,temp_int,PARAM_DEFAULT_COLOR_ROWS_PER_PAGE);
      if (temp_int > 0 && temp_int < 10)
        m_color_rows_per_page = temp_int;
      else
      {
        m_color_cols_per_page = PARAM_DEFAULT_COLOR_ROWS_PER_PAGE;
        ROS_WARN("rviz_cloud_annotation_plugin: %s is out of sane range: %d, using default.",
                 PARAM_NAME_COLORS_ROWS_PER_PAGE,temp_int);
      }

      m_nh.param<std::string>(PARAM_NAME_SET_CURRENT_LABEL_TOPIC,param_string,PARAM_DEFAULT_SET_CURRENT_LABEL_TOPIC);
      m_set_current_label_pub = m_nh.advertise<std_msgs::UInt32>(param_string,1);

      m_nh.param<std::string>(PARAM_NAME_CURRENT_LABEL_TOPIC,param_string,PARAM_DEFAULT_CURRENT_LABEL_TOPIC);
      m_set_current_label_sub = m_nh.subscribe(param_string,1,&QRVizCloudAnnotation::onSetCurrentLabel,this);

      m_nh.param<std::string>(PARAM_NAME_SAVE_TOPIC,param_string,PARAM_DEFAULT_SAVE_TOPIC);
      m_save_pub = m_nh.advertise<std_msgs::String>(param_string,1);

      m_nh.param<std::string>(PARAM_NAME_RESTORE_TOPIC,param_string,PARAM_DEFAULT_RESTORE_TOPIC);
      m_restore_pub = m_nh.advertise<std_msgs::String>(param_string,1);

      m_nh.param<std::string>(PARAM_NAME_CLEAR_TOPIC,param_string,PARAM_DEFAULT_CLEAR_TOPIC);
      m_clear_pub = m_nh.advertise<std_msgs::UInt32>(param_string,1);

      m_nh.param<std::string>(PARAM_NAME_SET_NAME_TOPIC,param_string,PARAM_DEFAULT_SET_NAME_TOPIC);
      m_set_name_pub = m_nh.advertise<std_msgs::String>(param_string,1);

      m_nh.param<std::string>(PARAM_NAME_SET_NAME_TOPIC2,param_string,PARAM_DEFAULT_SET_NAME_TOPIC2);
      m_set_name_sub = m_nh.subscribe(param_string,1,&QRVizCloudAnnotation::onSetName,this);

      m_nh.param<std::string>(PARAM_NAME_POINT_COUNT_UPDATE_TOPIC,param_string,PARAM_DEFAULT_POINT_COUNT_UPDATE_TOPIC);
      m_point_count_update_sub = m_nh.subscribe(param_string,1,&QRVizCloudAnnotation::onPointCountUpdate,this);

      m_nh.param<std::string>(PARAM_NAME_VIEW_CLOUD_TOPIC,param_string,PARAM_DEFAULT_VIEW_CLOUD_TOPIC);
      m_view_cloud_pub = m_nh.advertise<std_msgs::Bool>(param_string,1);

      m_nh.param<std::string>(PARAM_NAME_VIEW_CONTROL_POINTS_TOPIC,param_string,PARAM_DEFAULT_VIEW_CONTROL_POINTS_TOPIC);
      m_view_control_points_pub = m_nh.advertise<std_msgs::Bool>(param_string,1);

      m_nh.param<std::string>(PARAM_NAME_VIEW_LABEL_TOPIC,param_string,PARAM_DEFAULT_VIEW_LABEL_TOPIC);
      m_view_labels_pub = m_nh.advertise<std_msgs::Bool>(param_string,1);

      m_nh.param<std::string>(PARAM_NAME_UNDO_TOPIC,param_string,PARAM_DEFAULT_UNDO_TOPIC);
      m_undo_pub = m_nh.advertise<std_msgs::Empty>(param_string,1);

      m_nh.param<std::string>(PARAM_NAME_REDO_TOPIC,param_string,PARAM_DEFAULT_REDO_TOPIC);
      m_redo_pub = m_nh.advertise<std_msgs::Empty>(param_string,1);

      m_nh.param<std::string>(PARAM_NAME_UNDO_REDO_STATE_TOPIC,param_string,PARAM_DEFAULT_UNDO_REDO_STATE_TOPIC);
      m_undo_redo_state_sub = m_nh.subscribe(param_string,1,&QRVizCloudAnnotation::onUndoRedoState,this);

      m_nh.param<std::string>(PARAM_NAME_POINT_SIZE_CHANGE_TOPIC,param_string,PARAM_DEFAULT_POINT_SIZE_CHANGE_TOPIC);
      m_point_size_change_pub = m_nh.advertise<std_msgs::Int32>(param_string,1);

      m_nh.param<std::string>(PARAM_NAME_CONTROL_POINT_WEIGHT_TOPIC,param_string,PARAM_DEFAULT_CONTROL_POINT_WEIGHT_TOPIC);
      m_control_points_weight_pub = m_nh.advertise<std_msgs::UInt32>(param_string,1);

      m_nh.param<std::string>(PARAM_NAME_CONTROL_POINT_MAX_WEIGHT_TOPIC,param_string,PARAM_DEFAULT_CONTROL_POINT_MAX_WEIGHT_TOPIC);
      m_control_point_max_weight_sub = m_nh.subscribe(param_string,1,&QRVizCloudAnnotation::onSetControlPointMaxWeight,this);

      m_nh.param<std::string>(PARAM_NAME_GOTO_FIRST_UNUSED_TOPIC,param_string,PARAM_DEFAULT_GOTO_FIRST_UNUSED_TOPIC);
      m_goto_first_unused_pub = m_nh.advertise<std_msgs::Empty>(param_string,1);

      m_nh.param<std::string>(PARAM_NAME_GOTO_FIRST_TOPIC,param_string,PARAM_DEFAULT_GOTO_FIRST_TOPIC);
      m_goto_first_pub = m_nh.advertise<std_msgs::Empty>(param_string,1);

      m_nh.param<std::string>(PARAM_NAME_GOTO_LAST_UNUSED_TOPIC,param_string,PARAM_DEFAULT_GOTO_LAST_UNUSED_TOPIC);
      m_goto_last_unused_pub = m_nh.advertise<std_msgs::Empty>(param_string,1);

      m_nh.param<std::string>(PARAM_NAME_GOTO_NEXT_UNUSED_TOPIC,param_string,PARAM_DEFAULT_GOTO_NEXT_UNUSED_TOPIC);
      m_goto_next_unused_pub = m_nh.advertise<std_msgs::Empty>(param_string,1);

      m_nh.param<std::string>(PARAM_NAME_TOOL_TYPE_TOPIC,param_string,PARAM_DEFAULT_TOOL_TYPE_TOPIC);
      m_tool_type_pub = m_nh.advertise<std_msgs::UInt32>(param_string,1,true);
    }

    QBoxLayout * main_layout = new QBoxLayout(QBoxLayout::TopToBottom,this);

    {
      QMenuBar * menu_bar = new QMenuBar(this);
      menu_bar->setNativeMenuBar(false);
      main_layout->addWidget(menu_bar);

      QMenu * file_menu = menu_bar->addMenu("File");

      QAction * clear_action = new QAction("New",menu_bar);
      file_menu->addAction(clear_action);
      connect(clear_action,&QAction::triggered,this,&QRVizCloudAnnotation::onClear,Qt::QueuedConnection);

      QAction * save_action = new QAction("Save",menu_bar);
      file_menu->addAction(save_action);
      connect(save_action,&QAction::triggered,this,&QRVizCloudAnnotation::onSave,Qt::QueuedConnection);

      QAction * restore_action = new QAction("Restore",menu_bar);
      file_menu->addAction(restore_action);
      connect(restore_action,&QAction::triggered,this,&QRVizCloudAnnotation::onRestore,Qt::QueuedConnection);

      QMenu * edit_menu = menu_bar->addMenu("Edit");

      m_undo_action = new QAction("Undo",menu_bar);
      m_undo_action->setEnabled(false);
      m_undo_action->setShortcut(QKeySequence("U"));
      edit_menu->addAction(m_undo_action);
      connect(m_undo_action,&QAction::triggered,this,&QRVizCloudAnnotation::onUndo);

      m_redo_action = new QAction("Redo",menu_bar);
      m_redo_action->setEnabled(false);
      m_redo_action->setShortcut(QKeySequence("Shift+U"));
      edit_menu->addAction(m_redo_action);
      connect(m_redo_action,&QAction::triggered,this,&QRVizCloudAnnotation::onRedo);

      QMenu * view_menu = menu_bar->addMenu("View");

      QAction * bigger_points_action = new QAction("Increase point size",menu_bar);
      bigger_points_action->setShortcut(QKeySequence("Shift+O"));
      view_menu->addAction(bigger_points_action);
      connect(bigger_points_action,&QAction::triggered,this,&QRVizCloudAnnotation::onBiggerPoints);

      QAction * smaller_points_action = new QAction("Decrease point size",menu_bar);
      smaller_points_action->setShortcut(QKeySequence("O"));
      view_menu->addAction(smaller_points_action);
      connect(smaller_points_action,&QAction::triggered,this,&QRVizCloudAnnotation::onSmallerPoints);

      QAction * reset_points_size_action = new QAction("Reset point size",menu_bar);
      reset_points_size_action->setShortcut(QKeySequence("Alt+O"));
      view_menu->addAction(reset_points_size_action);
      connect(reset_points_size_action,&QAction::triggered,this,&QRVizCloudAnnotation::onResetPointsSize);

      QMenu * label_menu = menu_bar->addMenu("Label");

      QAction * clear_current_action = new QAction("Clear current",menu_bar);
      label_menu->addAction(clear_current_action);
      connect(clear_current_action,&QAction::triggered,this,&QRVizCloudAnnotation::onClearCurrent,Qt::QueuedConnection);

      label_menu->addSeparator();

      m_prev_label_action = new QAction("Previous",menu_bar);
      label_menu->addAction(m_prev_label_action);
      m_prev_label_action->setShortcut(QKeySequence("-"));
      connect(m_prev_label_action,&QAction::triggered,this,&QRVizCloudAnnotation::onMinusLabel);

      m_next_label_action = new QAction("Next",menu_bar);
      label_menu->addAction(m_next_label_action);
      m_next_label_action->setShortcut(QKeySequence("+"));
      connect(m_next_label_action,&QAction::triggered,this,&QRVizCloudAnnotation::onPlusLabel);

      label_menu->addSeparator();

      m_prev_page_action = new QAction("Prev page",menu_bar);
      label_menu->addAction(m_prev_page_action);
      m_prev_page_action->setShortcut(QKeySequence("PgUp"));
      connect(m_prev_page_action,&QAction::triggered,this,&QRVizCloudAnnotation::onPageUp);

      m_next_page_action = new QAction("Next page",menu_bar);
      label_menu->addAction(m_next_page_action);
      m_next_page_action->setShortcut(QKeySequence("PgDown"));
      connect(m_next_page_action,&QAction::triggered,this,&QRVizCloudAnnotation::onPageDown);

      label_menu->addSeparator();

      // go to submenu
      {
        QMenu * goto_menu = label_menu->addMenu("Go to");

        QAction * goto_first_action = new QAction("First",menu_bar);
        goto_menu->addAction(goto_first_action);
        goto_first_action->setShortcut(QKeySequence("Home"));
        connect(goto_first_action,&QAction::triggered,this,&QRVizCloudAnnotation::onGotoFirst);

        QAction * goto_first_unused_action = new QAction("First empty",menu_bar);
        goto_menu->addAction(goto_first_unused_action);
        goto_first_unused_action->setShortcut(QKeySequence("L"));
        connect(goto_first_unused_action,&QAction::triggered,this,&QRVizCloudAnnotation::onGotoFirstUnused);

        QAction * goto_next_unused_action = new QAction("Next empty",menu_bar);
        goto_menu->addAction(goto_next_unused_action);
        goto_next_unused_action->setShortcut(QKeySequence("Alt+L"));
        connect(goto_next_unused_action,&QAction::triggered,this,&QRVizCloudAnnotation::onGotoNextUnused);

        QAction * goto_last_unused_action = new QAction("Last empty",menu_bar);
        goto_menu->addAction(goto_last_unused_action);
        goto_last_unused_action->setShortcut(QKeySequence("End"));
        connect(goto_last_unused_action,&QAction::triggered,this,&QRVizCloudAnnotation::onGotoLastUnused);
      }

      label_menu->addSeparator();

      // weight submenu
      {
        m_weight_menu = label_menu->addMenu("Weight");
        m_weight_menu->setEnabled(false);

        m_prev_weight_action = new QAction("Decrease",menu_bar);
        m_weight_menu->addAction(m_prev_weight_action);
        m_prev_weight_action->setShortcut(QKeySequence("W"));
        connect(m_prev_weight_action,&QAction::triggered,this,&QRVizCloudAnnotation::onControlPointWeightDec);

        m_next_weight_action = new QAction("Increase",menu_bar);
        m_weight_menu->addAction(m_next_weight_action);
        m_next_weight_action->setShortcut(QKeySequence("Shift+W"));
        connect(m_next_weight_action,&QAction::triggered,this,&QRVizCloudAnnotation::onControlPointWeightInc);

        m_min_weight_action = new QAction("Minimum",menu_bar);
        m_weight_menu->addAction(m_min_weight_action);
        m_min_weight_action->setShortcut(QKeySequence("Alt+W"));
        connect(m_min_weight_action,&QAction::triggered,this,&QRVizCloudAnnotation::onControlPointWeightMin);

        m_max_weight_action = new QAction("Maximum",menu_bar);
        m_weight_menu->addAction(m_max_weight_action);
        m_max_weight_action->setShortcut(QKeySequence("Alt+Shift+W"));
        connect(m_max_weight_action,&QAction::triggered,this,&QRVizCloudAnnotation::onControlPointWeightMax);
      }
    }

    {
      QBoxLayout * toolbar_layout = new QBoxLayout(QBoxLayout::LeftToRight);
      main_layout->addLayout(toolbar_layout);

      m_toolbar_group = new QButtonGroup(this);

      m_edit_none_button = new QPushButton("&None",this);
      m_edit_none_button->setCheckable(true);
      m_edit_none_button->setChecked(true);
      m_edit_none_button->setShortcut(QKeySequence("N"));
      toolbar_layout->addWidget(m_edit_none_button);
      m_toolbar_group->addButton(m_edit_none_button,EDIT_MODE_NONE);

      m_edit_control_point_button = new QPushButton("S&et",this);
      m_edit_control_point_button->setShortcut(QKeySequence("E"));
      m_edit_control_point_button->setCheckable(true);
      toolbar_layout->addWidget(m_edit_control_point_button);
      m_toolbar_group->addButton(m_edit_control_point_button,EDIT_MODE_CONTROL_POINT);

      m_edit_eraser_button = new QPushButton("Del",this);
      m_edit_eraser_button->setShortcut(QKeySequence("Del"));
      m_edit_eraser_button->setCheckable(true);
      toolbar_layout->addWidget(m_edit_eraser_button);
      m_toolbar_group->addButton(m_edit_eraser_button,EDIT_MODE_ERASER);

      m_edit_color_picker_button = new QPushButton("P&ick",this);
      m_edit_color_picker_button->setShortcut(QKeySequence("I"));
      m_edit_color_picker_button->setCheckable(true);
      toolbar_layout->addWidget(m_edit_color_picker_button);
      m_toolbar_group->addButton(m_edit_color_picker_button,EDIT_MODE_COLOR_PICKER);

      void (QButtonGroup::* button_clicked_function_pointer)(int) = &QButtonGroup::buttonClicked;
      connect(m_toolbar_group,button_clicked_function_pointer,this,&QRVizCloudAnnotation::onSetEditMode);
    }

    {
      QBoxLayout * tooltype_layout = new QBoxLayout(QBoxLayout::LeftToRight);
      main_layout->addLayout(tooltype_layout);

      m_tooltype_group = new QButtonGroup(this);

      m_tool_single_button = new QPushButton("Point",this);
      m_tool_single_button->setCheckable(true);
      m_tool_single_button->setChecked(true);
      tooltype_layout->addWidget(m_tool_single_button);
      m_tooltype_group->addButton(m_tool_single_button,TOOL_TYPE_SINGLE_PICK);

      m_tool_shallow_square_button = new QPushButton("Shallow rect",this);
      m_tool_shallow_square_button->setCheckable(true);
      tooltype_layout->addWidget(m_tool_shallow_square_button);
      m_tooltype_group->addButton(m_tool_shallow_square_button,TOOL_TYPE_SHALLOW_SQUARE);

      m_tool_deep_square_button = new QPushButton("Deep rect",this);
      m_tool_deep_square_button->setCheckable(true);
      tooltype_layout->addWidget(m_tool_deep_square_button);
      m_tooltype_group->addButton(m_tool_deep_square_button,TOOL_TYPE_DEEP_SQUARE);

      m_tool_shallow_poly_button = new QPushButton("Polyline",this);
      m_tool_shallow_poly_button->setCheckable(true);
      tooltype_layout->addWidget(m_tool_shallow_poly_button);
      m_tooltype_group->addButton(m_tool_shallow_poly_button,TOOL_TYPE_SHALLOW_POLY);

      void (QButtonGroup::* button_clicked_function_pointer)(int) = &QButtonGroup::buttonClicked;
      connect(m_tooltype_group,button_clicked_function_pointer,this,&QRVizCloudAnnotation::onSetToolType);
    }

    {
      QBoxLayout * page_shift_layout = new QBoxLayout(QBoxLayout::LeftToRight);
      main_layout->addLayout(page_shift_layout);

      m_current_page_label = new QLabel("0/0",this);
      m_current_page_label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
      page_shift_layout->addWidget(m_current_page_label);

      QLabel * view_label = new QLabel("Show:",this);
      view_label->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
      page_shift_layout->addWidget(view_label);

      QToolButton * view_cloud_button = new QToolButton(this);
      view_cloud_button->setText("Cloud");
      view_cloud_button->setCheckable(true);
      view_cloud_button->setChecked(true);
      view_cloud_button->setSizePolicy(QSizePolicy::Minimum,QSizePolicy::Preferred);
      page_shift_layout->addWidget(view_cloud_button);
      connect(view_cloud_button,&QToolButton::toggled,this,&QRVizCloudAnnotation::onViewCloudToggled);

      QToolButton * view_labels_button = new QToolButton(this);
      view_labels_button->setText("Labels");
      view_labels_button->setCheckable(true);
      view_labels_button->setChecked(true);
      view_labels_button->setSizePolicy(QSizePolicy::Minimum,QSizePolicy::Preferred);
      page_shift_layout->addWidget(view_labels_button);
      connect(view_labels_button,&QToolButton::toggled,this,&QRVizCloudAnnotation::onViewLabelsToggled);

      QToolButton * view_control_points_button = new QToolButton(this);
      view_control_points_button->setText("Seeds");
      view_control_points_button->setCheckable(true);
      view_control_points_button->setChecked(true);
      view_control_points_button->setSizePolicy(QSizePolicy::Minimum,QSizePolicy::Preferred);
      page_shift_layout->addWidget(view_control_points_button);
      connect(view_control_points_button,&QToolButton::toggled,this,&QRVizCloudAnnotation::onViewControlPointsToggled);
    }

    {
      QGridLayout * page_layout = new QGridLayout();
      main_layout->addLayout(page_layout);

      page_layout->setVerticalSpacing(1);
      page_layout->setHorizontalSpacing(1);

      m_page_button_group = new QButtonGroup(this);

      m_page_buttons.resize(m_color_cols_per_page * m_color_rows_per_page);
      for (uint64 y = 0; y < m_color_rows_per_page; y++)
        for (uint64 x = 0; x < m_color_cols_per_page; x++)
        {
          QPushButton * button = new QPushButton(this);
          const uint64 id = x + y * m_color_cols_per_page;
          m_page_buttons[id] = button;
          button->setCheckable(true);
          m_page_button_group->addButton(button,id + 1);
          button->setMinimumWidth(1);
          page_layout->addWidget(button,y,x);
        }

      // this is needed since buttonClicked is overloaded (we must specify which one)
      void (QButtonGroup::* button_clicked_function_pointer)(int) = &QButtonGroup::buttonClicked;
      connect(m_page_button_group,button_clicked_function_pointer,this,&QRVizCloudAnnotation::onLabelButtonSelected);
    }

    {
      QBoxLayout * control_point_weight_layout = new QBoxLayout(QBoxLayout::LeftToRight);
      main_layout->addLayout(control_point_weight_layout);

      QLabel * weight_label = new QLabel("Weight:",this);
      weight_label->setSizePolicy(QSizePolicy::Minimum,QSizePolicy::Preferred);
      control_point_weight_layout->addWidget(weight_label);

      m_current_control_point_weight_label = new QLabel("---/---",this);
      m_current_control_point_weight_label->setFixedWidth(m_current_control_point_weight_label->sizeHint().width());
      m_current_control_point_weight_label->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
      m_current_control_point_weight_label->setSizePolicy(QSizePolicy::Fixed,QSizePolicy::Preferred);
      control_point_weight_layout->addWidget(m_current_control_point_weight_label);

      m_current_control_point_weight_slider = new QSlider(Qt::Horizontal,this);
      QSlider * & weight_slider = m_current_control_point_weight_slider;
      weight_slider->setFocusPolicy(Qt::NoFocus);
      weight_slider->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Preferred);
      weight_slider->setMinimum(0);
      weight_slider->setMaximum(100);
      weight_slider->setValue(100);
      weight_slider->setPageStep(1);
      weight_slider->setEnabled(false);
      weight_slider->setTracking(false);
      weight_slider->setSingleStep(1);
      weight_slider->setTickPosition(QSlider::NoTicks);
      control_point_weight_layout->addWidget(weight_slider);
      connect(weight_slider,&QSlider::sliderMoved,this,&QRVizCloudAnnotation::onControlPointWeightSliderMoved);
      connect(weight_slider,&QSlider::valueChanged,this,&QRVizCloudAnnotation::onControlPointWeightSliderSet);
      connect(weight_slider,&QSlider::valueChanged,this,&QRVizCloudAnnotation::onControlPointWeightSliderMoved);
    }

    {
      QBoxLayout * set_name_layout = new QBoxLayout(QBoxLayout::LeftToRight);
      main_layout->addLayout(set_name_layout);

      QPushButton * set_name_button = new QPushButton("Set name: ",this);
      set_name_layout->addWidget(set_name_button);

      m_set_name_edit = new QLineEdit("",this);
      set_name_layout->addWidget(m_set_name_edit);
      connect(m_set_name_edit,SIGNAL(returnPressed()),set_name_button,SLOT(animateClick())); // won't work with qt5 syntax
      connect(set_name_button,&QPushButton::clicked,this,&QRVizCloudAnnotation::onSendName);
    }

    m_current_edit_mode = 1;
    SetCurrentEditMode(EDIT_MODE_NONE);

    m_current_page = 1;
    m_current_label = 0;
    SetCurrentLabel(1,0);

    FillColorPageButtonStylesheet();
  }

  QRVizCloudAnnotation::~QRVizCloudAnnotation()
  {

  }

  void QRVizCloudAnnotation::onUndo()
  {
    m_undo_pub.publish(std_msgs::Empty());
  }

  void QRVizCloudAnnotation::onRedo()
  {
    m_redo_pub.publish(std_msgs::Empty());
  }

  void QRVizCloudAnnotation::onUndoRedoState(const rviz_cloud_annotation::UndoRedoState & msg)
  {
    m_undo_action->setEnabled(msg.undo_enabled);
    SetUndoText(msg.undo_description);
    m_redo_action->setEnabled(msg.redo_enabled);
    SetRedoText(msg.redo_description);
  }

  void QRVizCloudAnnotation::SetUndoText(const std::string & text)
  {
    std::string t = "Undo";
    if (!text.empty())
      t += " (" + text + ")";
    m_undo_action->setText(t.c_str());
  }

  void QRVizCloudAnnotation::SetRedoText(const std::string & text)
  {
    std::string t = "Redo";
    if (!text.empty())
      t += " (" + text + ")";
    m_redo_action->setText(t.c_str());
  }

  void QRVizCloudAnnotation::onSetName(const std_msgs::String & name)
  {
    const std::string n = name.data;
    m_set_name_edit->setText(QString::fromUtf8(n.c_str()));
  }

  void QRVizCloudAnnotation::onSendName()
  {
    std_msgs::String msg;
    msg.data = m_set_name_edit->text().toUtf8().constData();
    m_set_name_pub.publish(msg);
  }

  void QRVizCloudAnnotation::onViewCloudToggled(const bool checked)
  {
    std_msgs::Bool msg;
    msg.data = checked;
    m_view_cloud_pub.publish(msg);
  }

  void QRVizCloudAnnotation::onViewControlPointsToggled(const bool checked)
  {
    std_msgs::Bool msg;
    msg.data = checked;
    m_view_control_points_pub.publish(msg);
  }

  void QRVizCloudAnnotation::onViewLabelsToggled(const bool checked)
  {
    std_msgs::Bool msg;
    msg.data = checked;
    m_view_labels_pub.publish(msg);
  }

  void QRVizCloudAnnotation::onPointCountUpdate(const std_msgs::UInt64MultiArray & counters)
  {
    const uint64 count = counters.data.size() / 2;
    for (uint64 i = 0; i < count; i++)
    {
      const uint64 index = counters.data[i * 2];
      const uint64 value = counters.data[i * 2 + 1];

      if (index > m_point_counters.size())
        m_point_counters.resize(index,0);
      m_point_counters[index - 1] = value;
    }

    FillPointCounts();
  }

  void QRVizCloudAnnotation::onControlPointWeightInc()
  {
    const uint32 value = m_current_control_point_weight_slider->value();
    if (value >= m_control_point_weight_max)
      return;
    m_current_control_point_weight_slider->setValue(value + 1);
  }

  void QRVizCloudAnnotation::onControlPointWeightDec()
  {
    const uint32 value = m_current_control_point_weight_slider->value();
    if (value == 0)
      return;
    m_current_control_point_weight_slider->setValue(value - 1);
  }

  void QRVizCloudAnnotation::onControlPointWeightMax()
  {
    m_current_control_point_weight_slider->setValue(m_control_point_weight_max);
  }

  void QRVizCloudAnnotation::onControlPointWeightMin()
  {
    m_current_control_point_weight_slider->setValue(0);
  }

  void QRVizCloudAnnotation::onControlPointWeightSliderMoved(int new_value)
  {
    const std::string text = boost::lexical_cast<std::string>(new_value) + "/" +
      boost::lexical_cast<std::string>(m_control_point_weight_max);
    m_current_control_point_weight_label->setText(text.c_str());
  }

  void QRVizCloudAnnotation::onControlPointWeightSliderSet(int new_value)
  {
    m_prev_weight_action->setEnabled(uint32(new_value) > 0);
    m_next_weight_action->setEnabled(uint32(new_value) < m_control_point_weight_max);
    m_max_weight_action->setEnabled(uint32(new_value) < m_control_point_weight_max);
    m_min_weight_action->setEnabled(uint32(new_value) > 0);

    std_msgs::UInt32 msg;
    msg.data = new_value;
    m_control_points_weight_pub.publish(msg);
  }

  void QRVizCloudAnnotation::onSetControlPointMaxWeight(const std_msgs::UInt32 & msg)
  {
    m_control_point_weight_max = msg.data;

    // compute maximum length of label
    {
      const std::string number = boost::lexical_cast<std::string>(m_control_point_weight_max);
      const std::string number_zeros(number.size(),'0');
      const std::string text = number_zeros + "/" + number_zeros;
      m_current_control_point_weight_label->setText(text.c_str());
      m_current_control_point_weight_label->setFixedWidth(m_current_control_point_weight_label->sizeHint().width());
    }

    m_current_control_point_weight_slider->setMaximum(m_control_point_weight_max);
    m_current_control_point_weight_slider->setValue(m_control_point_weight_max);
    m_current_control_point_weight_slider->setEnabled(true);
    onControlPointWeightSliderMoved(m_control_point_weight_max);

    m_weight_menu->setEnabled(true);

    ROS_INFO("rviz_cloud_annotation_plugin: set max weight to %u",(unsigned int)(m_control_point_weight_max));
  }

  void QRVizCloudAnnotation::ColorToHex(const pcl::RGB & color,char color_hex[7])
  {
    static const char HEX[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
    color_hex[0] = HEX[color.r / 16];
    color_hex[1] = HEX[color.r % 16];
    color_hex[2] = HEX[color.g / 16];
    color_hex[3] = HEX[color.g % 16];
    color_hex[4] = HEX[color.b / 16];
    color_hex[5] = HEX[color.b % 16];
    color_hex[6] = 0;
  }

  void QRVizCloudAnnotation::FillColorPageButtonStylesheet()
  {
    std::ostringstream stylesheet;
    stylesheet << "QPushButton[ColorPageButton=\"true\"] {\n";
    stylesheet << "  min-width: 1em;\n";
    stylesheet << "  padding: 2px;\n";
    stylesheet << "  margin: 1px;\n";
    stylesheet << "}\n\n";

    for (uint64 i = 0; i < 256; i++)
    {
      stylesheet << "QPushButton[ColorPageButtonColorId=\"" << i << "\"] {\n";

      const pcl::RGB color = pcl::GlasbeyLUT::at(i);

      const uint64 luminance = color.r / 3 + color.g / 2 + color.b / 6; // approximate
      const char * text_color = "black";
      if (luminance < 80)
        text_color = "white";

      char color_hex[7];
      ColorToHex(color,color_hex);
      stylesheet << "  color: " << text_color << ";\n";
      stylesheet << "  background-color: #" << color_hex << ";\n";
      stylesheet << "}\n\n";

      stylesheet << "QPushButton[ColorPageButton=\"true\"][ColorPageButtonColorId=\"" << i << "\"]:checked {\n";
      stylesheet << "  color: black;\n";
      stylesheet << "}\n\n";
    }

    const std::string str = stylesheet.str();
    setStyleSheet(str.c_str());
  }

  void QRVizCloudAnnotation::FillPointCounts()
  {
    const uint64 page_size = m_color_cols_per_page * m_color_rows_per_page;
    for (uint64 i = 0; i < page_size; i++)
    {
      const uint64 label = i + m_current_page * page_size + 1;

      const uint64 count = (label > m_point_counters.size()) ? 0 : m_point_counters[label - 1];
      const std::string str = boost::lexical_cast<std::string>(count);
      m_page_buttons[i]->setText(str.c_str());
    }
  }

  void QRVizCloudAnnotation::FillColorPageButtons()
  {
    const uint64 size = m_page_buttons.size();

    for (uint64 i = 0; i < size; i++)
    {
      m_page_buttons[i]->setProperty("ColorPageButton",true);
      m_page_buttons[i]->setProperty("ColorPageButtonColorId",int((i + m_current_page * size) % 256));
      style()->unpolish(m_page_buttons[i]);
      style()->polish(m_page_buttons[i]);
      update();
    }

  }

  void QRVizCloudAnnotation::onSetCurrentLabel(const std_msgs::UInt32 & label)
  {
    const uint64 new_label = label.data;
    if (new_label == m_current_label)
      return;

    SetCurrentLabel(new_label,GetPageForLabel(new_label));
  }

  void QRVizCloudAnnotation::onSetEditMode2(const std_msgs::UInt32 & mode)
  {
    SetCurrentEditMode(mode.data);
  }

  void QRVizCloudAnnotation::onSetEditMode(int edit_mode)
  {
    SetCurrentEditMode(edit_mode);
  }

  void QRVizCloudAnnotation::onSetToolType(int tool_type)
  {
    std_msgs::UInt32 msg;
    msg.data = tool_type;
    m_tool_type_pub.publish(msg);
  }

  void QRVizCloudAnnotation::SetCurrentEditMode(const uint64 mode)
  {
    if (m_current_edit_mode == mode)
      return;

    m_current_edit_mode = mode;

    std_msgs::UInt32 edit_mode_out;
    edit_mode_out.data = mode;
    m_set_edit_mode_pub.publish(edit_mode_out);

    switch (mode)
    {
      case EDIT_MODE_NONE:
        if (!m_edit_none_button->isChecked())
          m_edit_none_button->setChecked(true);
        break;
      case EDIT_MODE_CONTROL_POINT:
        if (!m_edit_control_point_button->isChecked())
          m_edit_control_point_button->setChecked(true);
        break;
      case EDIT_MODE_ERASER:
        if (!m_edit_eraser_button->isChecked())
          m_edit_eraser_button->setChecked(true);
        break;
      case EDIT_MODE_COLOR_PICKER:
        if (!m_edit_color_picker_button->isChecked())
          m_edit_color_picker_button->setChecked(true);
        break;
      default:
        break;
    }
  }

  void QRVizCloudAnnotation::onLabelButtonSelected(int id)
  {
    SetCurrentLabel(GetLabelFromPageAndId(m_current_page,id),m_current_page);
  }

  void QRVizCloudAnnotation::onPlusLabel()
  {
    SetCurrentLabel(m_current_label + 1,GetPageForLabel(m_current_label + 1));
  }

  void QRVizCloudAnnotation::onMinusLabel()
  {
    if (m_current_label > 1)
      SetCurrentLabel(m_current_label - 1,GetPageForLabel(m_current_label - 1));
  }

  void QRVizCloudAnnotation::onPageDown()
  {
    SetCurrentLabel(GetFirstLabelForPage(m_current_page + 1),m_current_page + 1);
  }

  void QRVizCloudAnnotation::onPageUp()
  {
    if (m_current_page > 0)
      SetCurrentLabel(GetFirstLabelForPage(m_current_page - 1),m_current_page - 1);
  }

  void QRVizCloudAnnotation::onGotoFirstUnused()
  {
    m_goto_first_unused_pub.publish(std_msgs::Empty());
  }

  void QRVizCloudAnnotation::onGotoLastUnused()
  {
    m_goto_last_unused_pub.publish(std_msgs::Empty());
  }

  void QRVizCloudAnnotation::onGotoFirst()
  {
    m_goto_first_pub.publish(std_msgs::Empty());
  }

  void QRVizCloudAnnotation::onGotoNextUnused()
  {
    m_goto_next_unused_pub.publish(std_msgs::Empty());
  }

  void QRVizCloudAnnotation::onBiggerPoints()
  {
    std_msgs::Int32 msg;
    msg.data = POINT_SIZE_CHANGE_BIGGER;
    m_point_size_change_pub.publish(msg);
  }

  void QRVizCloudAnnotation::onSmallerPoints()
  {
    std_msgs::Int32 msg;
    msg.data = POINT_SIZE_CHANGE_SMALLER;
    m_point_size_change_pub.publish(msg);
  }

  void QRVizCloudAnnotation::onResetPointsSize()
  {
    std_msgs::Int32 msg;
    msg.data = POINT_SIZE_CHANGE_RESET;
    m_point_size_change_pub.publish(msg);
  }

  void QRVizCloudAnnotation::onSave()
  {
    std_msgs::String filename; // NYI: select filename from GUI
    m_save_pub.publish(filename);
  }

  void QRVizCloudAnnotation::onRestore()
  {
    const QMessageBox::StandardButton result =
      QMessageBox::question(this,"Restore","Are you sure you want to restore from last save?\n"
                                           "Current progress will be lost!",QMessageBox::Yes | QMessageBox::No);
    if (result == QMessageBox::Yes)
    {
      std_msgs::String filename; // NYI: select filename from GUI
      m_restore_pub.publish(filename);
    }
  }

  void QRVizCloudAnnotation::onClear()
  {
    const QMessageBox::StandardButton result =
      QMessageBox::question(this,"New","Are you sure you want to clear all labels?\n"
                                       "Current annotation will be lost!",QMessageBox::Yes | QMessageBox::No);
    if (result == QMessageBox::Yes)
    {
      std_msgs::UInt32 label;
      label.data = 0;
      m_clear_pub.publish(label);
    }
  }

  void QRVizCloudAnnotation::onClearCurrent()
  {
    const std::string msg = std::string("Are you sure you want to clear label ") +
      boost::lexical_cast<std::string>(m_current_label) + "?";
    const QMessageBox::StandardButton result =
      QMessageBox::question(this,"Clear",msg.c_str(),QMessageBox::Yes | QMessageBox::No);
    if (result == QMessageBox::Yes)
    {
      std_msgs::UInt32 label;
      label.data = m_current_label;
      m_clear_pub.publish(label);
    }
  }

  QRVizCloudAnnotation::uint64 QRVizCloudAnnotation::GetPageForLabel(const uint64 label) const
  {
    if (label == 0)
      return 0; // should never happen

    const uint64 size = m_color_cols_per_page * m_color_rows_per_page;
    return ((label - 1) / size);
  }

  QRVizCloudAnnotation::uint64 QRVizCloudAnnotation::GetLabelFromPageAndId(const uint64 page,const int id) const
  {
    if (id == 0)
      return 0;
    const uint64 size = m_color_cols_per_page * m_color_rows_per_page;
    return (page * size) + id;
  }

  QRVizCloudAnnotation::uint64 QRVizCloudAnnotation::GetFirstLabelForPage(const uint64 page) const
  {
    const uint64 size = m_color_cols_per_page * m_color_rows_per_page;
    return (page * size) + 1;
  }

  QRVizCloudAnnotation::uint64 QRVizCloudAnnotation::GetLastLabelForPage(const uint64 page) const
  {
    const uint64 size = m_color_cols_per_page * m_color_rows_per_page;
    return (page + 1) * size;
  }

  void QRVizCloudAnnotation::SetCurrentLabel(const uint64 label,const uint64 page)
  {
    const uint64 page_size = m_color_cols_per_page * m_color_rows_per_page;

    if (m_current_page != page)
    {
      m_current_page = page;
      FillColorPageButtons();
      FillPointCounts();
    }

    if (label != m_current_label)
    {
      m_current_label = label;

      const uint64 current_id = (m_current_label - 1) % page_size;
      if (!m_page_buttons[current_id]->isChecked())
        m_page_buttons[current_id]->setChecked(true);

      std_msgs::UInt32 msg;
      msg.data = m_current_label;
      m_set_current_label_pub.publish(msg);
    }

    {
      std::ostringstream text;
      text << std::setw(3) << m_current_label << " ";
      text << std::setw(3) << GetFirstLabelForPage(m_current_page) << "-";
      text << std::setw(3) << GetLastLabelForPage(m_current_page);
      const std::string str = text.str();
      m_current_page_label->setText(str.c_str());
    }

    m_prev_page_action->setEnabled(m_current_page > 0);
    m_prev_label_action->setEnabled(m_current_label > 1);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_cloud_annotation::QRVizCloudAnnotation,rviz::Panel);
