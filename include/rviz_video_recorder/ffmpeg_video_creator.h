#ifdef __cplusplus
#define __STDC_CONSTANT_MACROS
#ifdef _STDINT_H
#undef _STDINT_H
#endif
extern "C" {
#include <stdint.h>
#include <math.h>
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/mathematics.h>
#include <libswscale/swscale.h>
}
#endif

#include <iostream>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

struct FFMPEGFrame
{
  AVFrame* frame;
  uint8_t* buffer;
  int buffer_size;
};

class FFMPEGImageToVideoCreator
{
  public:
    FFMPEGImageToVideoCreator();
    ~FFMPEGImageToVideoCreator();
    void loadImages( const std::string& directory, const std::string& prefix );
    void createVideo( const std::string& video_filename );
    void testFun();
  protected:
    void open_video();
    void close_video();
    void add_video_stream(enum CodecID codec_id);
    void write_video_frame();
    void read_image( const std::string& filename);
    void alloc_frame(PixelFormat pix_fmt, int width, int height, FFMPEGFrame& fframe);

  private:
    size_t frame_rate_;
    int write_frame_count_;
    std::vector< AVFrame* > image_list_;

    // image reading
    AVCodecContext* image_codec_context_;
    AVFormatContext* image_format_context_;
    FFMPEGFrame image_frame_;
    FFMPEGFrame video_frame_;

    // video creation
    AVOutputFormat* video_output_format_;
    AVCodecContext* video_codec_context_;
    AVFormatContext* video_format_context_;
    AVStream* video_stream_;
    uint8_t* video_buffer_;
    int video_buffer_size_;
};
