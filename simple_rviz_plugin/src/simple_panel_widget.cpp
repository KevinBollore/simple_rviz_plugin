#include "simple_panel_widget.h"

#include <QTabWidget>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QDoubleSpinBox>
#include <QFuture>
#include <QtConcurrentRun>

#include <simple_node/SendOffset.h> //Description of the service we will use

namespace simple_rviz_plugin
{
SimplePanelWidget::SimplePanelWidget(QWidget* parent) : rviz::Panel(parent)
{
  // Create UI and layout element
  tab_widget_ = new QTabWidget();

  widget_x_ = new XWidget(); // Create each tab in another class
  widget_y_ = new YWidget();
  widget_z_ = new ZWidget();

  tab_widget_->addTab(widget_x_,"Offset X");
  tab_widget_->addTab(widget_y_,"Offset Y");
  tab_widget_->addTab(widget_z_,"Offset Z");
  QLabel* status_label_ = new QLabel;
  status_label_->setText("Status:");
  status_ = new QLabel;
  QPushButton* move_robot_button_ = new QPushButton;
  move_robot_button_->setText("Move robot");

  QVBoxLayout* widget_layout = new QVBoxLayout();
  widget_layout->addWidget(tab_widget_);
  widget_layout->addWidget(move_robot_button_);
  widget_layout->addSpacing(10);
  widget_layout->addWidget(status_label_);
  widget_layout->addWidget(status_);
  setLayout(widget_layout);

  // Connect UI element to update the config file when the values are changed
  // The value is saved only if the user exits RViz saving the config file
  connect(widget_x_, SIGNAL(GUIChanged()), this, SLOT(triggerSave()));
  connect(widget_y_, SIGNAL(GUIChanged()), this, SLOT(triggerSave()));
  connect(widget_z_, SIGNAL(GUIChanged()), this, SLOT(triggerSave()));

  // Connect handlers
  connect(move_robot_button_, SIGNAL(released()), this, SLOT(moveRobotButtonHandler()));
  connect(this, SIGNAL(enablePanel(bool)), this, SLOT(enablePanelHandler(bool)));
  connect(this, SIGNAL(displayStatus(QString)), this, SLOT(displayStatusHandler(QString)));

  //setup client
  offset_move_robot_ = nh_.serviceClient<simple_node::SendOffset>("move_robot_offset");

  // Check connection of client
  QFuture<void> future = QtConcurrent::run(this, &SimplePanelWidget::connectToServices);
}

SimplePanelWidget::~SimplePanelWidget()
{}

void SimplePanelWidget::connectToServices()
{
  Q_EMIT enablePanel(false);

  // Check offset_move_robot_ connection
  Q_EMIT displayStatus("Connecting to service");
  while (ros::ok())
  {
    if (offset_move_robot_.waitForExistence(ros::Duration(2)))
    {
      ROS_INFO_STREAM("RViz panel connected to the service " << offset_move_robot_.getService());
      Q_EMIT displayStatus(QString::fromStdString("RViz panel connected to the service: " + offset_move_robot_.getService()));
      break;
    }
    else
    {
      ROS_ERROR_STREAM("RViz panel could not connect to ROS service:\n\t" << offset_move_robot_.getService());
      Q_EMIT displayStatus(QString::fromStdString("RViz panel could not connect to ROS service: " + offset_move_robot_.getService()));
      sleep(1);
    }
  }

  ROS_WARN_STREAM("Service connection have been made");
  Q_EMIT displayStatus("Ready to take commands");
  Q_EMIT enablePanel(true);
}

void SimplePanelWidget::enablePanelHandler(bool status)
{
  this->setEnabled(status);
}

void SimplePanelWidget::displayStatusHandler(const QString status)
{
  status_->setText(status);
}

void SimplePanelWidget::moveRobotButtonHandler()
{
  // Fill the service request
  srv_.request.Message="";
  if (tab_widget_->currentIndex() == 0)
  {
    srv_.request = widget_x_->getParams();
  }
  else if (tab_widget_->currentIndex() == 1)
  {
    srv_.request = widget_y_->getParams();
  }
  else if (tab_widget_->currentIndex() == 2)
  {
    srv_.request = widget_z_->getParams();
  }
  else
  {
    Q_EMIT displayStatus("This tab is not implemented!");
    return;
  }
  // run moveRobot function in a thread
  QFuture<void> future = QtConcurrent::run(this, &SimplePanelWidget::moveRobot);
}

void SimplePanelWidget::moveRobot()
{
  //WARNING: DO NOT modify UI in the thread!
  //Only Q_EMIT are authorized
  Q_EMIT enablePanel(false);
  // Call service
  offset_move_robot_.call(srv_);
  Q_EMIT displayStatus(QString::fromStdString(srv_.response.ReturnMessage));
  Q_EMIT enablePanel(true);
}

void SimplePanelWidget::triggerSave()
{
  Q_EMIT configChanged();
}

void SimplePanelWidget::save(rviz::Config config) const
{
  rviz::Panel::save(config);

  //Each tab call his own save function
  widget_x_->save(config);
  widget_y_->save(config);
  widget_z_->save(config);
}

// Load all configuration data for this panel from the given Config object.
void SimplePanelWidget::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
  // Each tab call his own load function
  widget_x_->load(config);
  widget_y_->load(config);
  widget_z_->load(config);
}

}  // end namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(simple_rviz_plugin::SimplePanelWidget, rviz::Panel)
