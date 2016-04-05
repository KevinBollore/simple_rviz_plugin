#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QMessageBox>
#include <QFuture>
#include <QtConcurrentRun>

#include "simple_panel.h"
#include <simple_node/SendOffset.h> // Description of the Service we will use

namespace simple_rviz_plugin
{
SimplePanel::SimplePanel(QWidget* parent) :
    rviz::Panel(parent)
{
  // Create UI and layout elements
  QHBoxLayout* offset_layout = new QHBoxLayout;
  offset_layout->addWidget(new QLabel("Offset:"));
  offset_ = new QDoubleSpinBox;
  offset_->setSuffix(" mm");
  offset_->setMinimum(-1000.0);
  offset_->setMaximum(1000.0);
  offset_->setSingleStep(50.0);
  offset_->setValue(50.0);
  offset_layout->addWidget(offset_);

  move_robot_ = new QPushButton;
  move_robot_->setText("Move robot");

  QVBoxLayout* status_layout = new QVBoxLayout;
  status_layout->addWidget(new QLabel("Status:"));
  status_label_ = new QLabel;
  status_layout->addWidget(status_label_);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(offset_layout);
  layout->addWidget(move_robot_);
  layout->addStretch(10);
  layout->addLayout(status_layout);
  setLayout(layout);

  // Connect UI elements to update the config file when the values are changed, the value is saved only if
  // the user exits RViz saving the config file.
  connect(offset_, SIGNAL(editingFinished()), this, SLOT(triggerSave()));

  // Connect handlers
  connect(move_robot_, SIGNAL(released()), this, SLOT(moveRobotButtonHandler()));
  connect(this, SIGNAL(enablePanel(bool)), this, SLOT(enablePanelHandler(bool)));
  connect(this, SIGNAL(displayStatus()), this, SLOT(displayStatusHandler()));

  // Setup client
  offset_move_robot_ = nh_.serviceClient<simple_node::SendOffset>("move_robot_offset");
}

void SimplePanel::enablePanelHandler(bool status)
{
  // Enable or disable panel
  this->setEnabled(status);
}

void SimplePanel::displayStatusHandler()
{
  status_label_->setText(QString::fromStdString(srv_.response.ReturnMessage));
}

void SimplePanel::moveRobotButtonHandler()
{
  // Fill in the request
  srv_.request.Message = "Hello from Qt! (SimplePanel::moveRobotButtonHandler)";
  srv_.request.Offset = offset_->value();
  // Start client service call in an other thread
  QFuture<void> future = QtConcurrent::run(this, &SimplePanel::moveRobot);
}

void SimplePanel::moveRobot()
{
  // Disable UI
  Q_EMIT enablePanel(false);

  // Call service to move the robot (srv_ has been filled in moveRobotButtonHandler)
  offset_move_robot_.call(srv_);
  // Display return message in Qt panel
  Q_EMIT displayStatus();

  // Re-enable UI
  Q_EMIT enablePanel(true); // Enable UI
}

void SimplePanel::triggerSave()
{
  Q_EMIT configChanged();
}

// Save all configuration data from this panel to the given Config object
void SimplePanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
  // Save offset value into the config file
  config.mapSetValue("offset", offset_->value());
}

// Load all configuration data for this panel from the given Config object.
void SimplePanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
  float tmp;
  // Load offset value from config file (if it exists)
  if (config.mapGetFloat("offset", &tmp))
    offset_->setValue(tmp);
}

}  // end namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(simple_rviz_plugin::SimplePanel, rviz::Panel)
