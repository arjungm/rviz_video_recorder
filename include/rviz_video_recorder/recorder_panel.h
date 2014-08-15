#ifndef RECORDER_PANEL_H
#define RECORDER_PANEL_H

#include <queue>

#include <ros/ros.h>
#include <rviz/panel.h>

#include <QLineEdit>
#include <QPushButton>
#include <QPixmap>

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
      virtual void onInitialize();
    
    protected Q_SLOTS:
      void updateTopic();
      void saveSnapshotToQueue();
      void toggleRecording();
      void createVideo();

    protected:
      ros::NodeHandle nh_;
      ros::Subscriber command_listener_;
      QString command_topic_;
      QLineEdit* command_topic_editor_;

      QPushButton* record_button_;

      std::queue<QPixmap> snapshot_queue_;
      bool is_recording_;
  };
}

#endif
