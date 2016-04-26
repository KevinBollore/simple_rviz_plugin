
#include <QHBoxLayout>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QFuture>
#include <QtConcurrentRun>

#include "y_widget.h"

simple_rviz_plugin::YWidget::YWidget(QWidget* parent) : QWidget(parent)
{
  // Create UI of y tab widget
  this->setObjectName("YWidget_");
  y_offset_label_ = new QLabel;
  y_offset_label_->setText(QString::fromStdString("Offset Y:"));
  y_offset_ = new QDoubleSpinBox;
  y_offset_->setSuffix(" mm");
  y_offset_->setMinimum(-1000.0);
  y_offset_->setMaximum(1000.0);
  y_offset_->setSingleStep(50.0);
  y_offset_->setValue(50.0);
  y_offset_layout_ = new QHBoxLayout(this);
  y_offset_layout_->addWidget(y_offset_label_);
  y_offset_layout_->addWidget(y_offset_);

  params_.Axis = "y";

  connect(y_offset_, SIGNAL(valueChanged(double)), this, SLOT(triggerSave()));
}

void simple_rviz_plugin::YWidget::triggerSave()
{
  Q_EMIT GUIChanged();
  updateInternalValues();
  updateGUI();
}

simple_node::SendOffset::Request simple_rviz_plugin::YWidget::getParams()
{
  return params_;
}

void simple_rviz_plugin::YWidget::setParams(simple_node::SendOffset::Request params)
{
  // DO NOT modify params_.Axis here
  params_.Offset = params.Offset;
  updateGUI();
}

void simple_rviz_plugin::YWidget::updateGUI()
{
  y_offset_->setValue(params_.Offset);
}

void simple_rviz_plugin::YWidget::updateInternalValues()
{
  params_.Offset = y_offset_->value();
}

// Save all configuration data from this panel to the given Config object
void simple_rviz_plugin::YWidget::save(rviz::Config config)
{
  // Save offset value into the config file
  config.mapSetValue(this->objectName() + "y_offset_", y_offset_->value());
}

// Load all configuration data for this panel from the given Config object.
void simple_rviz_plugin::YWidget::load(const rviz::Config& config)
{
  float tmp;
  // Load offset value from config file (if it exists)
  if (config.mapGetFloat(this->objectName() + "y_offset_", &tmp))
    y_offset_->setValue(tmp);
}

