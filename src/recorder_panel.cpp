#include "rviz_video_recorder/recorder_panel.h"

#include <rviz/visualization_manager.h>

#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>

#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>

namespace rviz_recorder
{
  RecorderPanel::RecorderPanel( QWidget* parent ) : 
    rviz::Panel( parent ),
    is_recording_(false)
  {
    record_button_ = new QPushButton("Start Recording");

    QHBoxLayout* topic_layout = new QHBoxLayout;
    topic_layout->addWidget( new QLabel( "Command Topic:" ));
    command_topic_editor_ = new QLineEdit;
    topic_layout->addWidget( command_topic_editor_ );
    
    // outermost
    QVBoxLayout* layout = new QVBoxLayout;
    layout->addLayout( topic_layout );
    layout->addWidget( record_button_ );

    setLayout( layout );

    ROS_INFO("Constructed!");
  }

  void RecorderPanel::onInitialize()
  {
    connect( command_topic_editor_ , SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));
    connect( record_button_, SIGNAL( clicked() ), this, SLOT( toggleRecording() ) );
    connect( vis_manager_, SIGNAL( preUpdate() ), this, SLOT( saveSnapshotToQueue() ));

    ROS_INFO("Initialized!");
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
  void RecorderPanel::toggleRecording()
  {
    is_recording_ = !is_recording_;
    ROS_INFO("Recording is %s.", is_recording_?"started":"stopped");
    if(is_recording_)
      record_button_->setText("Stop Recording");
    else
      record_button_->setText("Start Recording");
  }
  void RecorderPanel::saveSnapshotToQueue()
  {
    if(is_recording_)
    {
      ROS_INFO("Recording!");
    }
  }

  void RecorderPanel::createVideo()
  {
    AVFormatContext* format_context_ptr;
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_recorder::RecorderPanel,rviz::Panel )
