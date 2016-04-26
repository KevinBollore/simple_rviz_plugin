
#include <QHBoxLayout>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QFuture>
#include <QtConcurrentRun>

#include "x_widget.h"

simple_rviz_plugin::XWidget::XWidget(QWidget* parent) : QWidget(parent)
{
  // Create UI of x tab widget
  this->setObjectName("XWidget_");
  x_offset_label_ = new QLabel;
  x_offset_label_->setText("Offset X:");
  x_offset_ = new QDoubleSpinBox;
  x_offset_->setSuffix(" mm");
  x_offset_->setMinimum(-1000.0);
  x_offset_->setMaximum(1000.0);
  x_offset_->setSingleStep(50.0);
  x_offset_->setValue(50.0);
  x_offset_layout_ = new QHBoxLayout(this);
  x_offset_layout_->addWidget(x_offset_label_);
  x_offset_layout_->addWidget(x_offset_);

  params_.Axis = "x";

  connect(x_offset_, SIGNAL(valueChanged(double)), this, SLOT(triggerSave()));
}

void simple_rviz_plugin::XWidget::triggerSave()
{
  Q_EMIT GUIChanged();
  updateInternalValues();
  updateGUI();
}

simple_node::SendOffset::Request simple_rviz_plugin::XWidget::getParams()
{
  return params_;
}

void simple_rviz_plugin::XWidget::setParams(simple_node::SendOffset::Request params)
{
  // DO NOT modify params_.Axis here
  params_.Offset = params.Offset;
  updateGUI();
}

void simple_rviz_plugin::XWidget::updateGUI()
{
  x_offset_->setValue(params_.Offset);
}

void simple_rviz_plugin::XWidget::updateInternalValues()
{
  params_.Offset = x_offset_->value();
}

// Save all configuration data from this panel to the given Config object
void simple_rviz_plugin::XWidget::save(rviz::Config config)
{
  // Save offset value into the config file
  config.mapSetValue(this->objectName() + "x_offset_", x_offset_->value());
}

// Load all configuration data for this panel from the given Config object.
void simple_rviz_plugin::XWidget::load(const rviz::Config& config)
{
  float tmp;
  // Load offset value from config file (if it exists)
  if (config.mapGetFloat(this->objectName() + "x_offset_", &tmp))
    x_offset_->setValue(tmp);
}
