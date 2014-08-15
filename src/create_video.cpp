#ifdef __cplusplus
#define __STDC_CONSTANT_MACROS
#ifdef _STDINT_H
#undef _STDINT_H
#endif
extern "C" {
#include <stdint.h>
}
#endif

#include <iostream>
#include <string>
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>

int main(int argc, char** argv)
{
  AVFormatContext* format_context_ptr;
  std::string filename = "/home/amenon/Pictures/title_slide.jpg";

  if(av_open_input_file( &format_context_ptr, filename.c_str(), NULL, 0, NULL)!=0)
  {
    std::cout << "Can't open image" << std::endl;    
  }

  delete format_context_ptr;
  return 0;
}
