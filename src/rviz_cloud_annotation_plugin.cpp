#include "rviz_cloud_annotation_plugin.h"
#include "rviz_cloud_annotation.h"

#include <QShortcut>
#include <QLabel>
#include <QPushButton>
#include <QKeySequence>
#include <QBoxLayout>

#include <iostream>

#include <std_msgs/UInt32.h>

namespace rviz_cloud_annotation
{

  QRVizCloudAnnotation::QRVizCloudAnnotation(QWidget * parent): rviz::Panel(parent), m_nh("~")
  {
    {
      std::string temp_string;
      m_nh.param<std::string>(PARAM_NAME_SET_EDIT_MODE_TOPIC,temp_string,PARAM_DEFAULT_SET_EDIT_MODE_TOPIC);
      m_set_edit_mode_pub = m_nh.advertise<std_msgs::UInt32>(temp_string,1);
    }

    QBoxLayout * main_layout = new QBoxLayout(QBoxLayout::TopToBottom,this);

    {
      QBoxLayout * edit_mode_layout = new QBoxLayout(QBoxLayout::LeftToRight,this);
      main_layout->addLayout(edit_mode_layout);

      QLabel * edit_mode_label = new QLabel("Edit mode: ",this);
      edit_mode_layout->addWidget(edit_mode_label);

      m_edit_status_label = new QLabel(this);
      edit_mode_layout->addWidget(m_edit_status_label);

      QPushButton * edit_mode_button = new QPushButton("Toggle (E)",this);
      edit_mode_button->setShortcut(QKeySequence("E"));
      edit_mode_layout->addWidget(edit_mode_button);
      connect(edit_mode_button,&QPushButton::clicked,this,&QRVizCloudAnnotation::onToggleEditMode);
    }

    SetCurrentEditMode(false);
  }

  QRVizCloudAnnotation::~QRVizCloudAnnotation()
  {

  }

  void QRVizCloudAnnotation::onToggleEditMode()
  {
    SetCurrentEditMode(!m_current_edit_mode_status);
  }

  void QRVizCloudAnnotation::SetCurrentEditMode(bool on)
  {
    m_current_edit_mode_status = on;

    std_msgs::UInt32 edit_mode_out;
    edit_mode_out.data = m_current_edit_mode_status ? 1 : 0;
    m_set_edit_mode_pub.publish(edit_mode_out);

    m_edit_status_label->setText(m_current_edit_mode_status ?
                                 "<font color='red'>ON</font>" :
                                 "<font color='blue'>OFF</font>"
                                 );
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_cloud_annotation::QRVizCloudAnnotation,rviz::Panel);
