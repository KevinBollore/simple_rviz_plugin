#ifndef X_WIDGET_H
#define X_WIDGET_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/service.h>
#include <rviz/panel.h>
#endif

class QDoubleSpinBox;
class QLabel;
class QWidget;
class QHBoxLayout;

#include <simple_node/SendOffset.h>

namespace simple_rviz_plugin
{
class XWidget : public QWidget
{

  Q_OBJECT
public:

  XWidget(QWidget* parent =  NULL);

  simple_node::SendOffset::Request getParams();
  void setParams(simple_node::SendOffset::Request params_);

  void load(const rviz::Config& config);
  void save(rviz::Config config);

Q_SIGNALS:
  void GUIChanged();

protected Q_SLOTS:
  virtual void triggerSave();
  void updateGUI();
  void updateInternalValues();

protected:
  QDoubleSpinBox* x_offset_;
  QLabel* x_offset_label_;
  QHBoxLayout* x_offset_layout_;
  simple_node::SendOffset::Request params_;
};

} // End namespace


#endif // X_WIDGET_H
