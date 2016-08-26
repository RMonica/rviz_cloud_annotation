#include "rviz_cloud_annotation_plugin.h"
#include "rviz_cloud_annotation.h"

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

// STL
#include <iostream>
#include <string>
#include <sstream>

// ROS
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>

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
    }

    QBoxLayout * main_layout = new QBoxLayout(QBoxLayout::TopToBottom,this);

    {
      QMenuBar * menu_bar = new QMenuBar(this);
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

      QMenu * label_menu = menu_bar->addMenu("Label");

      QAction * clear_current_action = new QAction("Clear current",menu_bar);
      label_menu->addAction(clear_current_action);
      connect(clear_current_action,&QAction::triggered,this,&QRVizCloudAnnotation::onClearCurrent,Qt::QueuedConnection);
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
      QBoxLayout * page_shift_layout = new QBoxLayout(QBoxLayout::LeftToRight);
      main_layout->addLayout(page_shift_layout);

      m_prev_page_button = new QPushButton("<<",this);
      m_prev_page_button->setShortcut(QKeySequence("PgDown"));
      m_prev_page_button->setMinimumWidth(2);
      connect(m_prev_page_button,&QPushButton::clicked,this,&QRVizCloudAnnotation::onPageDown);
      page_shift_layout->addWidget(m_prev_page_button);
      m_next_page_button = new QPushButton(">>",this);
      m_next_page_button->setShortcut(QKeySequence("PgUp"));
      m_next_page_button->setMinimumWidth(2);
      connect(m_next_page_button,&QPushButton::clicked,this,&QRVizCloudAnnotation::onPageUp);
      page_shift_layout->addWidget(m_next_page_button);
      m_current_page_label = new QLabel("0/0",this);
      page_shift_layout->addWidget(m_current_page_label);

      m_prev_label_button = new QPushButton("-",this);
      m_prev_label_button->setShortcut(QKeySequence("-"));
      m_prev_label_button->setMinimumWidth(2);
      connect(m_prev_label_button,&QPushButton::clicked,this,&QRVizCloudAnnotation::onMinusLabel);
      page_shift_layout->addWidget(m_prev_label_button);
      m_next_label_button = new QPushButton("+",this);
      m_next_label_button->setShortcut(QKeySequence("+"));
      m_next_label_button->setMinimumWidth(2);
      connect(m_next_label_button,&QPushButton::clicked,this,&QRVizCloudAnnotation::onPlusLabel);
      page_shift_layout->addWidget(m_next_label_button);
    }

    {
      QGridLayout * page_layout = new QGridLayout();
      main_layout->addLayout(page_layout);

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
      QBoxLayout * set_name_layout = new QBoxLayout(QBoxLayout::LeftToRight);
      main_layout->addLayout(set_name_layout);

      QPushButton * set_name_button = new QPushButton("Set name: ",this);
      set_name_layout->addWidget(set_name_button);

      m_set_name_edit = new QLineEdit("",this);
      set_name_layout->addWidget(m_set_name_edit);
      connect(m_set_name_edit,SIGNAL(returnPressed()),set_name_button,SLOT(animateClick())); // won't work with qt5 syntax
      connect(set_name_button,&QPushButton::clicked,this,&QRVizCloudAnnotation::onSendName);
    }

    SetCurrentEditMode(EDIT_MODE_NONE);

    m_current_page = 1;
    m_current_label = 0;
    SetCurrentLabel(1,0);
  }

  QRVizCloudAnnotation::~QRVizCloudAnnotation()
  {

  }

  void QRVizCloudAnnotation::onSetName(const std_msgs::String & name)
  {
    const std::string n = name.data;
    m_set_name_edit->setText(n.c_str());
  }

  void QRVizCloudAnnotation::onSendName()
  {
    std_msgs::String msg;
    msg.data = m_set_name_edit->text().toUtf8().constData();
    m_set_name_pub.publish(msg);
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
    static const char HEX[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
    const uint64 size = m_page_buttons.size();

    for (uint64 i = 0; i < size; i++)
    {
      const pcl::RGB color = pcl::GlasbeyLUT::at((i + m_current_page * size) % 256);
      char color_hex[6];
      color_hex[0] = HEX[color.r / 16];
      color_hex[1] = HEX[color.r % 16];
      color_hex[2] = HEX[color.g / 16];
      color_hex[3] = HEX[color.g % 16];
      color_hex[4] = HEX[color.b / 16];
      color_hex[5] = HEX[color.b % 16];
      const std::string stylesheet = std::string("background-color: #") + color_hex;
      m_page_buttons[i]->setStyleSheet(stylesheet.c_str());
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

  void QRVizCloudAnnotation::onPageUp()
  {
    SetCurrentLabel(GetFirstLabelForPage(m_current_page + 1),m_current_page + 1);
  }

  void QRVizCloudAnnotation::onPageDown()
  {
    if (m_current_page > 0)
      SetCurrentLabel(GetFirstLabelForPage(m_current_page - 1),m_current_page - 1);
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

    m_prev_page_button->setEnabled(m_current_page > 0);
    m_prev_label_button->setEnabled(m_current_label > 1);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_cloud_annotation::QRVizCloudAnnotation,rviz::Panel);
