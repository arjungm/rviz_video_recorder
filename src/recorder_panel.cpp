#include "rviz_video_recorder/recorder_panel.h"

#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>

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
    save_images_button_ = new QPushButton("Write images");

    QHBoxLayout* topic_layout = new QHBoxLayout;
    topic_layout->addWidget( new QLabel( "Command Topic:" ));
    command_topic_editor_ = new QLineEdit;
    topic_layout->addWidget( command_topic_editor_ );
    
    // outermost
    QVBoxLayout* layout = new QVBoxLayout;
    layout->addLayout( topic_layout );
    layout->addWidget( record_button_ );
    layout->addWidget( save_images_button_ );

    setLayout( layout );
  }

  void RecorderPanel::onInitialize()
  {
    connect( command_topic_editor_ , SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));
    connect( record_button_, SIGNAL( clicked() ), this, SLOT( toggleRecording() ) );
    connect( save_images_button_, SIGNAL( clicked() ), this, SLOT( writeImages() ) );
    connect( vis_manager_, SIGNAL( preUpdate() ), this, SLOT( saveSnapshotToQueue() ));
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
    {
      record_button_->setText("Stop Recording");
      ROS_INFO("Queue size: %d", (int)snapshot_queue_.size());
    }
    else
      record_button_->setText("Start Recording");
  }
  void RecorderPanel::saveSnapshotToQueue()
  {
    if(is_recording_)
      snapshot_queue_.push(QPixmap::grabWindow( vis_manager_->getRenderPanel()->winId() ));
  }
  void RecorderPanel::writeImages()
  {
    std::string filename, image_directory = "/tmp/images";
    size_t image_count = 0;

    while(!snapshot_queue_.empty())
    {
      filename = boost::str(boost::format("%s/%s%d.jpg") % image_directory % "img" % image_count++);
      QString filepath( filename.c_str() );
      ROS_INFO("Saving to: %s", filename.c_str());
      bool result = snapshot_queue_.front().save( filepath, "JPG", 100 );
      snapshot_queue_.pop();
      if(!result)
        ROS_WARN("Failed saving image #%d", (int)image_count);
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_recorder::RecorderPanel,rviz::Panel )
