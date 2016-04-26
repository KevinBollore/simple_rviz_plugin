#ifndef SIMPLE_PANEL_WIDGET_H
#define SIMPLE_PANEL_WIDGET_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/service.h>
#include <rviz/panel.h>
#endif

#include <simple_node/SendOffset.h> // Description of the service we will use

#include "x_widget.h" // Description of each tab in simple panel
#include "y_widget.h"
#include "z_widget.h"

class QTabWidget;
class QLabel;
class QVBoxLayout;
class QPushButton;

namespace simple_rviz_plugin
{
class SimplePanelWidget : public rviz::Panel
{
Q_OBJECT
public:
  SimplePanelWidget(QWidget* parent = 0);
  virtual ~SimplePanelWidget();

Q_SIGNALS:
  void enablePanel(bool);
  void displayStatus(const QString);

public Q_SLOTS:
  virtual void moveRobot();

protected Q_SLOTS:
  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

  virtual void triggerSave();

  void moveRobotButtonHandler();
  void displayStatusHandler(const QString);
  void enablePanelHandler(bool);
  void connectToServices();

protected:
  // Description of ROS NodeHandle, service client and service
  ros::NodeHandle nh_;
  ros::ServiceClient offset_move_robot_;
  simple_node::SendOffset srv_;

  QTabWidget* tab_widget_;
  QLabel* status_label_;
  QLabel* status_;
  QPushButton* move_robot_button_;
  XWidget* widget_x_;
  YWidget* widget_y_;
  ZWidget* widget_z_;
};

}  // end namespace

#endif // SIMPLE_PANEL_WIDGET_H
