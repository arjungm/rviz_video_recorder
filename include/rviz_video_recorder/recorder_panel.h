#ifndef RECORDER_PANEL_H
#define RECORDER_PANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <QLineEdit>

namespace rviz_recorder
{

  class RecorderPanel : public rviz::Panel
  {
    Q_OBJECT
    public:
      RecorderPanel( QWidget* parent = 0 );
      virtual void load( const rviz::Config& config );
      virtual void save(rviz::Config config ) const;
      void setTopic( const QString& topic );
    
    protected Q_SLOTS:
      void updateTopic();

    protected:
      ros::NodeHandle nh_;
      ros::Subscriber command_listener_;
      QString command_topic_;
      QLineEdit* command_topic_editor_;
  };
}

#endif
