
#include <QHBoxLayout>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QFuture>
#include <QtConcurrentRun>

#include "z_widget.h"
#include <simple_node/SendOffset.h> // Description of the Service we will use

simple_rviz_plugin::ZWidget::ZWidget(QWidget* parent) : QWidget(parent)
{
  // Create UI of z tab widget
  this->setObjectName("ZWidget_");
  z_offset_label_ = new QLabel;
  z_offset_label_->setText(QString::fromStdString("Offset Z:"));
  z_offset_ = new QDoubleSpinBox;
  z_offset_->setSuffix(" mm");
  z_offset_->setMinimum(-1000.0);
  z_offset_->setMaximum(1000.0);
  z_offset_->setSingleStep(50.0);
  z_offset_->setValue(50.0);
  z_offset_layout_ = new QHBoxLayout(this);
  z_offset_layout_->addWidget(z_offset_label_);
  z_offset_layout_->addWidget(z_offset_);

  params_.Axis = "z";

  connect(z_offset_, SIGNAL(valueChanged(double)), this, SLOT(triggerSave()));
}

void simple_rviz_plugin::ZWidget::triggerSave()
{
  Q_EMIT GUIChanged();
  updateInternalValues();
  updateGUI();
}

simple_node::SendOffset::Request simple_rviz_plugin::ZWidget::getParams()
{
  return params_;
}

void simple_rviz_plugin::ZWidget::setParams(simple_node::SendOffset::Request params)
{
  // DO NOT modify params_.Axis here
  params_.Offset = params.Offset;
  updateGUI();
}

void simple_rviz_plugin::ZWidget::updateGUI()
{
  z_offset_->setValue(params_.Offset);
}

void simple_rviz_plugin::ZWidget::updateInternalValues()
{
  params_.Offset = z_offset_->value();
}

// Save all configuration data from this panel to the given Config object
void simple_rviz_plugin::ZWidget::save(rviz::Config config)
{
  // Save offset value into the config file
  config.mapSetValue(this->objectName() + "z_offset_", z_offset_->value());
}

// Load all configuration data for this panel from the given Config object.
void simple_rviz_plugin::ZWidget::load(const rviz::Config& config)
{
  float tmp;
  // Load offset value from config file (if it exists)
  if (config.mapGetFloat(this->objectName() + "z_offset_", &tmp))
    z_offset_->setValue(tmp);
}
