#include "waypoint_type.h"

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <waypoint_type/wp_type.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QCheckBox>
#include <QLineEdit>
#include <QGroupBox>
#include <QRadioButton>
#include <QButtonGroup>


namespace waypoint_type{
WaypointTypePanel::WaypointTypePanel( QWidget* parent )
  : rviz::Panel( parent )
{
  QVBoxLayout* layout = new QVBoxLayout;

  QHBoxLayout* layout_1 = new QHBoxLayout;
  is_stop_point = new QCheckBox("Stop point");
  layout_1->addWidget(is_stop_point);
  layout->addLayout(layout_1);

  QHBoxLayout* layout_2 = new QHBoxLayout;
  is_changemap_point = new QCheckBox("Change map point");
  layout_2->addWidget(is_changemap_point);
  layout->addLayout(layout_2);

  QHBoxLayout* layout_3 = new QHBoxLayout;
  is_lineup_point = new QCheckBox("Line-up point");
  layout_3->addWidget(is_lineup_point);
  layout->addLayout(layout_3);

  QHBoxLayout* layout_4 = new QHBoxLayout;
  is_slowdown_point = new QCheckBox("Slow down point");
  layout_4->addWidget(is_slowdown_point);
  layout->addLayout(layout_4);

  QHBoxLayout* layout_5 = new QHBoxLayout;
  is_skippable_point = new QCheckBox("Skippable point");
  layout_5->addWidget(is_skippable_point);
  layout->addLayout(layout_5);

  setLayout(layout);

  QTimer* output_timer = new QTimer(this);
  connect( output_timer, SIGNAL(timeout()), this, SLOT(tick()));
  output_timer->start(100);
  wp_type_pub = nh_.advertise<waypoint_type::wp_type>("wp_type", 10);
}
WaypointTypePanel::~WaypointTypePanel(){
  if(wp_type_pub){
    wp_type_pub.shutdown();
  }
}

void WaypointTypePanel::tick()
{
  if( ros::ok())
  {
    waypoint_type::wp_type data;
    if(is_stop_point->isChecked()){
      data.type |= static_cast<int>(STOP_POINT);
    }
    if(is_changemap_point->isChecked()){
      data.type |= static_cast<int>(CHANGE_MAP_POINT);
    }
    if(is_lineup_point->isChecked()){
      data.type |= static_cast<int>(LINE_UP_POINT);
    }
    if(is_slowdown_point->isChecked()){
      data.type |= static_cast<int>(SLOW_DOWN_POINT);
    }
    if(is_skippable_point->isChecked()){
      data.type |= static_cast<int>(SKIPPABLE_POINT);
    }
    wp_type_pub.publish(data);
  }
}

}
PLUGINLIB_EXPORT_CLASS(waypoint_type::WaypointTypePanel,rviz::Panel )
