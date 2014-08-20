#include "rviz_video_recorder/ffmpeg_video_creator.h"

#include <iostream>

int main( int argc, char** argv)
{
  FFMPEGImageToVideoCreator fvc;
  std::cout << "START!" << std::endl;
  fvc.testFun();
  std::cout << "END!" << std::endl;
  return 0;
}
