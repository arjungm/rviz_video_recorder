#include <boost/make_shared.hpp>
#include "rviz_video_recorder/ffmpeg_video_creator.h"

FFMPEGImageToVideoCreator::FFMPEGImageToVideoCreator()
{
  av_register_all();
}

FFMPEGImageToVideoCreator::~FFMPEGImageToVideoCreator() {}

void FFMPEGImageToVideoCreator::loadImages( const std::string& directory, const std::string& prefix )
{
}

void FFMPEGImageToVideoCreator::createVideo( const std::string& video_filename )
{
}

boost::shared_ptr<FFMPEGFrame> FFMPEGImageToVideoCreator::alloc_frame(PixelFormat pix_fmt, int width, int height)
{
  boost::shared_ptr<FFMPEGFrame> fframe = boost::make_shared<FFMPEGFrame>();
  
  // allocate frame
  fframe->frame.reset(avcodec_alloc_frame());

  if(!fframe->frame)
    std::cout << "Unable to allocate frame!" << std::endl;
  
  // make buffer
  fframe->buffer_size = avpicture_get_size( pix_fmt, width, height);
  fframe->buffer.reset( (uint8_t *) av_malloc( fframe->buffer_size * sizeof(uint8_t)) );
  
  // connect buffer to frame
  avpicture_fill((AVPicture *) fframe->frame.get(), fframe->buffer.get(), pix_fmt, width, height);

  return fframe;
}

boost::shared_ptr<FFMPEGFrame> FFMPEGImageToVideoCreator::read_image( const std::string& filename )
{
  image_format_context_.reset(avformat_alloc_context());
  AVFormatContext* tmp = image_format_context_.get();
  if(avformat_open_input( &tmp , filename.c_str(), NULL, NULL)!=0)
    std::cout << "Can't open image" << std::endl;    

  // get codec
  boost::shared_ptr<AVCodec> image_codec;
  image_codec_context_.reset(image_format_context_->streams[0]->codec);
  image_codec_context_->width = 1600;
  image_codec_context_->height = 1200;
    // (*codec_context)->pix_fmt = PIX_FMT_YUV420P;
  image_codec.reset(avcodec_find_decoder(image_codec_context_->codec_id));
  if(!image_codec)
    std::cout << "Failed to find codec" << std::endl;
  if(avcodec_open2( image_codec_context_.get(), image_codec.get(), NULL)<0)
    std::cout << "Failed to open codec" << std::endl;

  // get frame
  boost::shared_ptr<FFMPEGFrame> fframe = alloc_frame(image_codec_context_->pix_fmt, image_codec_context_->width, image_codec_context_->height);

  // read image into the frame
  AVPacket packet;
  int frame_number = 0;
  while (av_read_frame(image_format_context_.get(), &packet) >= 0)
  {
    // image stream (only 1)
    if(packet.stream_index != 0)
      continue;

    int frame_finished; 
    int ret = avcodec_decode_video2(image_codec_context_.get(), fframe->frame.get(), &frame_finished, &packet);
    if (ret > 0)
      std::cout << "Decoded" << std::endl;
    else 
      std::cout << "Failed to decode" << std::endl;
  }
   
  av_free(image_format_context_.get());
  image_format_context_.reset();
  avcodec_close(image_codec_context_.get());
  image_codec_context_.reset();

  return fframe;
}

void FFMPEGImageToVideoCreator::open_video()
{
  boost::shared_ptr<AVCodec> video_codec;
  boost::shared_ptr<AVCodecContext> video_codec_context;
  video_codec_context.reset( video_stream_->codec );

  // locate codec
  video_codec.reset( avcodec_find_encoder( video_codec_context->codec_id ) );
  if(!video_codec)
    std::cout << "Can't find video codec" << std::endl;

  // open codec
  if( avcodec_open2( video_codec_context.get(), video_codec.get(), NULL)<0 )
    std::cout << "Can't open the video codec" << std::endl;

  // create video buffer
  if (!(video_format_context_->oformat->flags & AVFMT_RAWPICTURE))
  {
    video_buffer_size_ = 200000+video_codec_context->width*video_codec_context->height;
    video_buffer_.reset((uint8_t *)av_malloc(video_buffer_size_));
  }
}

void FFMPEGImageToVideoCreator::close_video()
{
}

void FFMPEGImageToVideoCreator::add_video_stream()
{
}

void FFMPEGImageToVideoCreator::write_video_frame()
{
}
