#ifndef SIMPLE_PANEL_H
#define SIMPLE_PANEL_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/service.h>
#include <rviz/panel.h>
#endif

#include <simple_node/SendOffset.h> // Cannot be forward declared

class QDoubleSpinBox;
class QSpinBox;
class QCheckBox;
class QPushButton;
class QLabel;

namespace simple_rviz_plugin
{
class SimplePanel : public rviz::Panel
{
Q_OBJECT
public:
  SimplePanel(QWidget* parent = 0);

  virtual void
  load(const rviz::Config& config);
  virtual void
  save(rviz::Config config) const;

Q_SIGNALS:
  void enablePanel(bool);
  void displayStatus();

public Q_SLOTS:
  virtual void
  moveRobot();

protected Q_SLOTS:
  virtual void
  triggerSave();

  void moveRobotButtonHandler();
  void displayStatusHandler();
  void enablePanelHandler(bool);

protected:
  ros::NodeHandle nh_;
  ros::ServiceClient offset_move_robot_;
  simple_node::SendOffset srv_;
  QDoubleSpinBox* offset_;
  QPushButton* move_robot_;
  QLabel* status_label_;
};

}  // end namespace

#endif // SIMPLE_PANEL_H
