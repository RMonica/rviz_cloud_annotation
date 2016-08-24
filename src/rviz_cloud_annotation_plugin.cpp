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
      std::string temp_string;
      int temp_int;

      m_nh.param<std::string>(PARAM_NAME_SET_EDIT_MODE_TOPIC,temp_string,PARAM_DEFAULT_SET_EDIT_MODE_TOPIC);
      m_set_edit_mode_pub = m_nh.advertise<std_msgs::UInt32>(temp_string,1);

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

      m_nh.param<std::string>(PARAM_NAME_SET_CURRENT_LABEL_TOPIC,temp_string,PARAM_DEFAULT_SET_CURRENT_LABEL_TOPIC);
      m_set_current_label_pub = m_nh.advertise<std_msgs::UInt32>(temp_string,1);

      m_nh.param<std::string>(PARAM_NAME_SAVE_TOPIC,temp_string,PARAM_DEFAULT_SAVE_TOPIC);
      m_save_pub = m_nh.advertise<std_msgs::String>(temp_string,1);

      m_nh.param<std::string>(PARAM_NAME_RESTORE_TOPIC,temp_string,PARAM_DEFAULT_RESTORE_TOPIC);
      m_restore_pub = m_nh.advertise<std_msgs::String>(temp_string,1);

      m_nh.param<std::string>(PARAM_NAME_CLEAR_TOPIC,temp_string,PARAM_DEFAULT_CLEAR_TOPIC);
      m_clear_pub = m_nh.advertise<std_msgs::UInt32>(temp_string,1);
    }

    QBoxLayout * main_layout = new QBoxLayout(QBoxLayout::TopToBottom,this);

    {
      QBoxLayout * file_layout = new QBoxLayout(QBoxLayout::LeftToRight);
      main_layout->addLayout(file_layout);

      QPushButton * save_button = new QPushButton("Save",this);
      file_layout->addWidget(save_button);
      connect(save_button,&QPushButton::clicked,this,&QRVizCloudAnnotation::onSave,Qt::QueuedConnection);

      QPushButton * restore_button = new QPushButton("Restore",this);
      file_layout->addWidget(restore_button);
      connect(restore_button,&QPushButton::clicked,this,&QRVizCloudAnnotation::onRestore,Qt::QueuedConnection);

      QPushButton * clear_button = new QPushButton("Clear",this);
      file_layout->addWidget(clear_button);
      connect(clear_button,&QPushButton::clicked,this,&QRVizCloudAnnotation::onClear,Qt::QueuedConnection);
    }

    {
      QBoxLayout * toolbar_layout = new QBoxLayout(QBoxLayout::LeftToRight);
      main_layout->addLayout(toolbar_layout);

      m_edit_mode_button = new QPushButton("&Edit mode",this);
      toolbar_layout->addWidget(m_edit_mode_button);
      m_edit_mode_button->setCheckable(true);
      m_edit_mode_button->setShortcut(QKeySequence("E"));
      connect(m_edit_mode_button,&QPushButton::toggled,this,&QRVizCloudAnnotation::onToggleEditMode);

      m_eraser_button = new QPushButton("Eraser (Del)",this);
      m_eraser_button->setShortcut(QKeySequence("Del"));
      m_eraser_button->setCheckable(true);
      toolbar_layout->addWidget(m_eraser_button);
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
      m_page_button_group->addButton(m_eraser_button,0);

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

    SetCurrentEditMode(false);

    m_current_page = 1;
    m_current_label = 0;
    SetCurrentLabel(1,0);
  }

  QRVizCloudAnnotation::~QRVizCloudAnnotation()
  {

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

  void QRVizCloudAnnotation::onToggleEditMode(bool on)
  {
    SetCurrentEditMode(on);
  }

  void QRVizCloudAnnotation::SetCurrentEditMode(bool on)
  {
    m_current_edit_mode_status = on;

    std_msgs::UInt32 edit_mode_out;
    edit_mode_out.data = m_current_edit_mode_status ? 1 : 0;
    m_set_edit_mode_pub.publish(edit_mode_out);

    if (m_edit_mode_button->isChecked() != on)
      m_edit_mode_button->setChecked(on);
  }

  void QRVizCloudAnnotation::onLabelButtonSelected(int id)
  {
    if (id == 0)
    {
      SetCurrentLabel(0,m_current_page);
      return;
    }

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
    if (m_current_label == 0)
      SetCurrentLabel(m_current_label,m_current_page + 1);
    else
      SetCurrentLabel(GetFirstLabelForPage(m_current_page + 1),m_current_page + 1);
  }

  void QRVizCloudAnnotation::onPageDown()
  {
    if (m_current_label == 0)
    {
      if (m_current_page > 0)
        SetCurrentLabel(m_current_label,m_current_page - 1);
      return;
    }

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
      QMessageBox::question(this,"Restore","Do you really want to restore from saved?",QMessageBox::Yes | QMessageBox::No);
    if (result == QMessageBox::Yes)
    {
      std_msgs::String filename; // NYI: select filename from GUI
      m_restore_pub.publish(filename);
    }
  }

  void QRVizCloudAnnotation::onClear()
  {
    const QMessageBox::StandardButton result =
      QMessageBox::question(this,"Clear","Do you really want to clear all the labels?",QMessageBox::Yes | QMessageBox::No);
    if (result == QMessageBox::Yes)
    {
      std_msgs::UInt32 label; // NYI: allow for labels selectively
      label.data = 0;
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
    }

    if (label != m_current_label)
    {
      m_current_label = label;
      if (m_current_label == 0)
      {
        if (!m_eraser_button->isChecked())
          m_eraser_button->setChecked(true);
      }
      else
      {
        const uint64 current_id = (m_current_label - 1) % page_size;
        if (!m_page_buttons[current_id]->isChecked())
          m_page_buttons[current_id]->setChecked(true);
      }

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
