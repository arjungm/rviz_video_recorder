#include "rviz_video_recorder/recorder_panel.h"

#include <QLabel>
#include <QHBoxLayout>

namespace rviz_recorder
{
  RecorderPanel::RecorderPanel( QWidget* parent ) : 
    rviz::Panel( parent )
  {
    QHBoxLayout* topic_layout = new QHBoxLayout;
    topic_layout->addWidget( new QLabel( "Output Topic:" ));
    command_topic_editor_ = new QLineEdit;
    topic_layout->addWidget( command_topic_editor_ );
    setLayout( topic_layout );

    connect( command_topic_editor_ , SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));

    ROS_INFO("Completed setting up the panel!");
  }

  void RecorderPanel::load( const rviz::Config& config )
  {
    rviz::Panel::load( config );
    QString topic;
    if( config.mapGetString( "RecorderCommandTopic", &topic ))
    {
      command_topic_editor_->setText( topic );
    }
  }

  void RecorderPanel::save( rviz::Config config ) const
  {
    rviz::Panel::save( config );
    config.mapSetValue( "RecorderCommandTopic", command_topic_ );
  }
  void RecorderPanel::updateTopic()
  {
    setTopic( command_topic_editor_->text() );
  }
  void RecorderPanel::setTopic( const QString& new_topic )
  {
    // Only take action if the name has changed.
    if( new_topic != command_topic_ )
    {
      command_topic_ = new_topic;
      // If the topic is the empty string, don't publish anything.
      if( command_topic_ == "" )
      {
      }
      else
      {
      }
      Q_EMIT configChanged();
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_recorder::RecorderPanel,rviz::Panel )
