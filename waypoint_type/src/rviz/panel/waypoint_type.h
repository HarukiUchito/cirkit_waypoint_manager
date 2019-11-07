#ifndef Q_MOC_RUN
# include <ros/ros.h>
# include <rviz/panel.h>
#endif

#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QCheckBox>
#include <QLineEdit>
#include <QRadioButton>
#include <QButtonGroup>
#include <QTimer>

#include <string>

namespace waypoint_type{
class WaypointTypePanel: public rviz::Panel{
Q_OBJECT
public:
  WaypointTypePanel( QWidget* parent = 0 );
  ~WaypointTypePanel();
public Q_SLOTS:
  void tick();

public:
  enum  AreaType
  {
      STOP_POINT          = 1 << 0, //1
      CHANGE_MAP_POINT    = 1 << 1, //2
      LINE_UP_POINT       = 1 << 2, //4
      SLOW_DOWN_POINT     = 1 << 3, //8
      SKIPPABLE_POINT     = 1 << 4,
  } ;
  // The ROS node handle.
  ros::NodeHandle nh_;
  // The ROS publisher for the command velocity.
  ros::Publisher wp_type_pub;

  QCheckBox* is_stop_point;
  QCheckBox* is_changemap_point;
  QCheckBox* is_lineup_point;
  QCheckBox* is_slowdown_point;
  QCheckBox* is_skippable_point;
};
}